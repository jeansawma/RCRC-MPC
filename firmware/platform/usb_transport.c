/* Native USB CDC glue using STM32 HAL and ST USB Device middleware APIs.
 * No heap, no blocking transmit, bounded line reception and copied TX queues.
 * Application code (this file) is MIT licensed; vendor code retains ST licenses. */
#include "usb_transport.h"
#include "usbd_desc.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define LINE_BYTES 256U
#define LINE_COUNT 8U
#define TX_BYTES 512U
#define TX_COUNT 8U
#define USB_SOF_TIMEOUT_MS 10U

static USBD_HandleTypeDef device;
PCD_HandleTypeDef hpcd_USB_OTG_HS;
static USBD_CDC_HandleTypeDef cdc_state;
static uint8_t rx_packet[512];
static char rx_lines[LINE_COUNT][LINE_BYTES];
static volatile unsigned rx_head, rx_tail;
static char assembling[LINE_BYTES];
static unsigned assembling_length;
static int discard_line;
static uint8_t tx_packets[TX_COUNT][TX_BYTES];
/* Independent of queued slots: closing/reopening DTR can reset the queue while
 * the peripheral is still completing an earlier packet. Never reuse its data. */
static uint8_t tx_inflight[TX_BYTES];
static uint16_t tx_lengths[TX_COUNT];
static unsigned tx_head, tx_tail;
static int tx_active;
static uint8_t line_coding[7] = {0x00,0xc2,0x01,0x00,0x00,0x00,0x08};
static volatile uint32_t session_epoch=1U;
static volatile uint32_t last_sof_ms;
static volatile uint8_t sof_seen, dtr_asserted, link_live;

/* Called in the USB ISR or with interrupts masked. No stale command may cross
 * a transport session boundary. SOF loss is detected once, not on every poll. */
static void invalidate_session(void) {
  ++session_epoch;
  if (!session_epoch) ++session_epoch;
  link_live=0; dtr_asserted=0; sof_seen=0;
  assembling_length=0; discard_line=0; rx_head=rx_tail=0;
  tx_head=tx_tail=0; tx_active=0;
}
static int connection_locked(void) {
  int fresh=sof_seen && (uint32_t)(HAL_GetTick()-last_sof_ms)<=USB_SOF_TIMEOUT_MS;
  int connected=device.dev_state==USBD_STATE_CONFIGURED && dtr_asserted && fresh;
  if (link_live && !connected) invalidate_session();
  else if (connected) link_live=1;
  return connected;
}

void *usb_static_alloc(uint32_t size) {
  if (size > sizeof cdc_state) return NULL;
  memset(&cdc_state, 0, sizeof cdc_state);
  return &cdc_state;
}
void usb_static_free(void *ptr) { (void)ptr; }

static int8_t cdc_init(void) {
  USBD_CDC_SetTxBuffer(&device, tx_inflight, 0);
  USBD_CDC_SetRxBuffer(&device, rx_packet);
  invalidate_session();
  return USBD_OK;
}
static int8_t cdc_deinit(void) {
  invalidate_session();
  return USBD_OK;
}
static int8_t cdc_control(uint8_t cmd, uint8_t *buf, uint16_t len) {
  switch (cmd) {
    case CDC_SET_LINE_CODING:
      if (len == 7U) memcpy(line_coding, buf, 7);
      break;
    case CDC_GET_LINE_CODING:
      if (len == 7U) memcpy(buf, line_coding, 7);
      break;
    case CDC_SET_CONTROL_LINE_STATE: {
      /* For zero-length requests ST passes the setup request, not a data byte. */
      if (len!=0U) return USBD_FAIL;
      const USBD_SetupReqTypedef *request=(const USBD_SetupReqTypedef *)buf;
      uint8_t asserted=(uint8_t)(request->wValue&1U);
      if (asserted!=dtr_asserted) {
        uint8_t had_sof=sof_seen; uint32_t previous_sof=last_sof_ms;
        invalidate_session();
        dtr_asserted=asserted;
        /* A new DTR session can use the already running 1 kHz SOF clock. */
        if (asserted) { sof_seen=had_sof; last_sof_ms=previous_sof; }
      }
      break;
    }
    default: break; /* CDC DTR/RTS never directly energize the converter. */
  }
  return USBD_OK;
}
static int8_t cdc_receive(uint8_t *buf, uint32_t *length) {
  if (!connection_locked()) {
    USBD_CDC_SetRxBuffer(&device,rx_packet);
    USBD_CDC_ReceivePacket(&device);
    return USBD_OK;
  }
  for (uint32_t i=0; i<*length; ++i) {
    uint8_t c=buf[i];
    if (c=='\r') continue;
    if (c=='\n') {
      if (!discard_line && assembling_length) {
        unsigned next=(rx_head+1U)%LINE_COUNT;
        if (next!=rx_tail) {
          memcpy(rx_lines[rx_head],assembling,assembling_length);
          rx_lines[rx_head][assembling_length]=0;
          __DMB(); rx_head=next;
        }
      }
      assembling_length=0; discard_line=0;
    } else if (c < 0x20U || c > 0x7eU) {
      discard_line=1;
    } else if (!discard_line) {
      if (assembling_length < LINE_BYTES-1U) assembling[assembling_length++]=(char)c;
      else discard_line=1;
    }
  }
  USBD_CDC_SetRxBuffer(&device,rx_packet);
  USBD_CDC_ReceivePacket(&device);
  return USBD_OK;
}
static int8_t cdc_transmit_complete(uint8_t *buf, uint32_t *length, uint8_t epnum) {
  (void)buf; (void)length; (void)epnum; return USBD_OK;
}
static USBD_CDC_ItfTypeDef cdc_interface={
  cdc_init, cdc_deinit, cdc_control, cdc_receive, cdc_transmit_complete
};

int usb_transport_init(void) {
  HAL_PWREx_EnableUSBVoltageDetector();
  if (USBD_Init(&device, &RCRC_Descriptors, 0)!=USBD_OK) return 0;
  if (USBD_RegisterClass(&device, &USBD_CDC)!=USBD_OK) return 0;
  if (USBD_CDC_RegisterInterface(&device, &cdc_interface)!=USBD_OK) return 0;
  return USBD_Start(&device)==USBD_OK;
}
void usb_transport_poll(void) {
  uint32_t primask=__get_PRIMASK(); __disable_irq();
  USBD_CDC_HandleTypeDef *state=(USBD_CDC_HandleTypeDef *)device.pClassData;
  if (!connection_locked() || state==NULL) {
    tx_head=tx_tail=0; tx_active=0;
  } else {
    if (tx_active && !state->TxState) {
      tx_active=0;
    }
    if (!tx_active && tx_head!=tx_tail && !state->TxState) {
      memcpy(tx_inflight,tx_packets[tx_tail],tx_lengths[tx_tail]);
      USBD_CDC_SetTxBuffer(&device,tx_inflight,tx_lengths[tx_tail]);
      if (USBD_CDC_TransmitPacket(&device)==USBD_OK) {
        tx_active=1; tx_tail=(tx_tail+1U)%TX_COUNT;
      }
    }
  }
  __set_PRIMASK(primask);
}
int usb_transport_readline(char *dst, size_t capacity) {
  if (!dst || !capacity) return 0;
  uint32_t primask=__get_PRIMASK(); __disable_irq();
  if (!connection_locked() || rx_head==rx_tail) { __set_PRIMASK(primask); return 0; }
  size_t n=strlen(rx_lines[rx_tail]);
  if (n+1U>capacity) {
    rx_tail=(rx_tail+1U)%LINE_COUNT; __set_PRIMASK(primask); dst[0]=0; return -1;
  }
  memcpy(dst,rx_lines[rx_tail],n+1U); rx_tail=(rx_tail+1U)%LINE_COUNT;
  __set_PRIMASK(primask); return (int)n;
}
int usb_transport_write(const char *text, size_t length) {
  if (!text || !length || length>TX_BYTES) return 0;
  uint32_t primask=__get_PRIMASK(); __disable_irq();
  unsigned next=(tx_head+1U)%TX_COUNT;
  if (!connection_locked() || next==tx_tail) {
    __set_PRIMASK(primask); return 0;
  }
  memcpy(tx_packets[tx_head],text,length); tx_lengths[tx_head]=(uint16_t)length;
  tx_head=next; __set_PRIMASK(primask); return 1;
}
int usb_transport_connected(void) {
  uint32_t primask=__get_PRIMASK(); __disable_irq();
  int connected=connection_locked();
  __set_PRIMASK(primask); return connected;
}
uint32_t usb_transport_session(void) {
  uint32_t primask=__get_PRIMASK(); __disable_irq();
  (void)connection_locked();
  uint32_t result=session_epoch;
  __set_PRIMASK(primask); return result;
}

/* PCD / USB Device bridge. Hardware uses the HS controller's internal FS PHY;
 * these PA11/PA12 pins are the board's USB_OTG connector, not its CH340 port. */
void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd) {
  if (pcd->Instance!=USB_OTG_HS) return;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio={0};
  gpio.Pin=GPIO_PIN_11|GPIO_PIN_12; gpio.Mode=GPIO_MODE_AF_PP;
  gpio.Pull=GPIO_NOPULL; gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate=GPIO_AF10_OTG1_FS; HAL_GPIO_Init(GPIOA,&gpio);
  __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();
  HAL_NVIC_SetPriority(OTG_HS_IRQn,6,0); HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcd) {
  if (pcd->Instance!=USB_OTG_HS) return;
  HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
  __HAL_RCC_USB1_OTG_HS_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA,GPIO_PIN_11|GPIO_PIN_12);
}
void OTG_HS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS); }
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *pcd) {
  USBD_LL_SetupStage((USBD_HandleTypeDef *)pcd->pData,(uint8_t *)pcd->Setup);
}
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *pcd,uint8_t epnum) {
  USBD_LL_DataOutStage((USBD_HandleTypeDef *)pcd->pData,epnum,pcd->OUT_ep[epnum].xfer_buff);
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *pcd,uint8_t epnum) {
  USBD_LL_DataInStage((USBD_HandleTypeDef *)pcd->pData,epnum,pcd->IN_ep[epnum].xfer_buff);
}
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *pcd) {
  last_sof_ms=HAL_GetTick(); sof_seen=1;
  USBD_LL_SOF((USBD_HandleTypeDef *)pcd->pData);
}
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *pcd) {
  invalidate_session();
  USBD_LL_SetSpeed((USBD_HandleTypeDef *)pcd->pData,USBD_SPEED_FULL);
  USBD_LL_Reset((USBD_HandleTypeDef *)pcd->pData);
}
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *pcd) {
  invalidate_session();
  USBD_LL_Suspend((USBD_HandleTypeDef *)pcd->pData);
}
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *pcd) {
  USBD_LL_Resume((USBD_HandleTypeDef *)pcd->pData);
}
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *pcd,uint8_t epnum) {
  USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)pcd->pData,epnum);
}
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *pcd,uint8_t epnum) {
  USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)pcd->pData,epnum);
}
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *pcd) {
  USBD_LL_DevConnected((USBD_HandleTypeDef *)pcd->pData);
}
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *pcd) {
  invalidate_session();
  USBD_LL_DevDisconnected((USBD_HandleTypeDef *)pcd->pData);
}
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *dev) {
  hpcd_USB_OTG_HS.Instance=USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints=9;
  hpcd_USB_OTG_HS.Init.speed=PCD_SPEED_FULL;
  hpcd_USB_OTG_HS.Init.dma_enable=DISABLE;
  hpcd_USB_OTG_HS.Init.phy_itface=PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_HS.Init.Sof_enable=ENABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable=DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable=DISABLE;
  hpcd_USB_OTG_HS.Init.battery_charging_enable=DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable=DISABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1=DISABLE;
  hpcd_USB_OTG_HS.Init.use_external_vbus=DISABLE;
  hpcd_USB_OTG_HS.pData=dev; dev->pData=&hpcd_USB_OTG_HS;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS)!=HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_HS,128)!=HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS,0,64)!=HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS,1,128)!=HAL_OK) return USBD_FAIL;
  if (HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS,2,64)!=HAL_OK) return USBD_FAIL;
  return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *dev) {
  return HAL_PCD_DeInit((PCD_HandleTypeDef *)dev->pData)==HAL_OK ? USBD_OK:USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *dev) {
  return HAL_PCD_Start((PCD_HandleTypeDef *)dev->pData)==HAL_OK ? USBD_OK:USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *dev) {
  return HAL_PCD_Stop((PCD_HandleTypeDef *)dev->pData)==HAL_OK ? USBD_OK:USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *dev,uint8_t address,uint8_t type,uint16_t size) {
  return HAL_PCD_EP_Open((PCD_HandleTypeDef *)dev->pData,address,size,type)==HAL_OK ? USBD_OK:USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *dev,uint8_t address) {
  return HAL_PCD_EP_Close((PCD_HandleTypeDef *)dev->pData,address)==HAL_OK ? USBD_OK:USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *dev,uint8_t address) {
  return HAL_PCD_EP_Flush((PCD_HandleTypeDef *)dev->pData,address)==HAL_OK ? USBD_OK:USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *dev,uint8_t address) {
  return HAL_PCD_EP_SetStall((PCD_HandleTypeDef *)dev->pData,address)==HAL_OK ? USBD_OK:USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *dev,uint8_t address) {
  return HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *)dev->pData,address)==HAL_OK ? USBD_OK:USBD_FAIL;
}
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *dev,uint8_t address) {
  PCD_HandleTypeDef *pcd=(PCD_HandleTypeDef *)dev->pData;
  return address&0x80U ? pcd->IN_ep[address&0x7fU].is_stall : pcd->OUT_ep[address&0x7fU].is_stall;
}
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *dev,uint8_t address) {
  return HAL_PCD_SetAddress((PCD_HandleTypeDef *)dev->pData,address)==HAL_OK ? USBD_OK:USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *dev,uint8_t address,uint8_t *buffer,uint32_t size) {
  return HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)dev->pData,address,buffer,size)==HAL_OK ? USBD_OK:USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *dev,uint8_t address,uint8_t *buffer,uint32_t size) {
  return HAL_PCD_EP_Receive((PCD_HandleTypeDef *)dev->pData,address,buffer,size)==HAL_OK ? USBD_OK:USBD_FAIL;
}
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *dev,uint8_t address) {
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)dev->pData,address);
}
void USBD_LL_Delay(uint32_t milliseconds) { HAL_Delay(milliseconds); }
