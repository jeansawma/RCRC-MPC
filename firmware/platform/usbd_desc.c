/* USB descriptors for the RCRC laboratory controller. ST's development
 * VID/PID (0483:5740) is for development; obtain a VID/PID before distribution. */
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "stm32h7xx_hal.h"

static uint8_t device_desc[USB_LEN_DEV_DESC] = {
  18, USB_DESC_TYPE_DEVICE, 0x00, 0x02, 0x02, 0x02, 0x00, 64,
  0x83, 0x04, 0x40, 0x57, 0x00, 0x01, 1, 2, 3, 1
};
static uint8_t language_desc[USB_LEN_LANGID_STR_DESC] = {4, USB_DESC_TYPE_STRING, 0x09, 0x04};
static uint8_t string_desc[USBD_MAX_STR_DESC_SIZ];
static uint8_t *get_device(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; *length = sizeof device_desc; return device_desc;
}
static uint8_t *get_language(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; *length = sizeof language_desc; return language_desc;
}
static uint8_t *get_manufacturer(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; USBD_GetString((uint8_t *)"RCRC Laboratory", string_desc, length); return string_desc;
}
static uint8_t *get_product(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; USBD_GetString((uint8_t *)"RCRC Control STM32H723", string_desc, length); return string_desc;
}
static uint8_t *get_serial(USBD_SpeedTypeDef speed, uint16_t *length) {
  static const char hex[] = "0123456789ABCDEF";
  char serial[25];
  uint32_t uid[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
  (void)speed;
  for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<8; ++j)
    serial[i*8+j] = hex[(uid[i] >> (28-j*4)) & 15U];
  serial[24] = 0;
  USBD_GetString((uint8_t *)serial, string_desc, length); return string_desc;
}
static uint8_t *get_configuration(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; USBD_GetString((uint8_t *)"CDC Control", string_desc, length); return string_desc;
}
static uint8_t *get_interface(USBD_SpeedTypeDef speed, uint16_t *length) {
  (void)speed; USBD_GetString((uint8_t *)"CDC Data", string_desc, length); return string_desc;
}
USBD_DescriptorsTypeDef RCRC_Descriptors = {
  get_device, get_language, get_manufacturer, get_product, get_serial,
  get_configuration, get_interface
};
