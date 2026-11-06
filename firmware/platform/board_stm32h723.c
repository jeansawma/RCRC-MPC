/* Board support for the supplied UC_PowerE-V2 + Conv_Edu schematics.
 * Application-specific glue; upstream HAL remains unmodified. */
#include "board.h"
#include "stm32h7xx_hal.h"
#include "usb_transport.h"
#include <string.h>
#include <math.h>
#include <ctype.h>

static ADC_HandleTypeDef adc;
static DMA_HandleTypeDef adc_dma;
static TIM_HandleTypeDef sampling_timer;
static TIM_HandleTypeDef pwm_timer;
static I2C_HandleTypeDef oled_i2c;
/* AXI SRAM, not DTCM. D-cache deliberately remains disabled. */
static volatile uint16_t adc_raw[3] __attribute__((aligned(32)));
static volatile uint32_t faults, capture_us;
static volatile uint8_t sample_pending, control_busy, adc_inflight;
static volatile uint8_t output_enabled;
static volatile uint8_t last_scan_valid;
static uint32_t control_session;
static uint32_t sample_period_us = 5000;
static uint8_t pa13_output;
static uint8_t peripherals_ready;
static float captured[3];

/* Platform fault bits: init, deadline, ADC, bad actuator, watchdog reboot. */
#define FAULT_INIT 1U
#define FAULT_DEADLINE 2U
#define FAULT_ADC 4U
#define FAULT_ACTUATOR 8U
#define FAULT_WATCHDOG 16U

static void output_off(void)
{
    GPIOA->BSRR = (uint32_t)GPIO_PIN_13 << 16U;
    TIM1->CCR2 = 0;
    output_enabled = 0;
}
static void latch_fault(uint32_t bit)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    faults |= bit; output_off();
    __set_PRIMASK(mask);
}
static int checked(HAL_StatusTypeDef s)
{
    if (s != HAL_OK) { latch_fault(FAULT_INIT); return 0; }
    return 1;
}

static int clocks_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef periph = {0};
    if (!checked(HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY))) return 0;
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    uint32_t start = HAL_GetTick();
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
        if (HAL_GetTick() - start > 100U) { latch_fault(FAULT_INIT); return 0; }
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 4;
    osc.PLL.PLLN = 100;
    osc.PLL.PLLP = 2;
    osc.PLL.PLLQ = 4;
    osc.PLL.PLLR = 2;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    osc.PLL.PLLFRACN = 0;
    if (!checked(HAL_RCC_OscConfig(&osc))) return 0;
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1; /* CPU 200 MHz */
    clk.AHBCLKDivider = RCC_HCLK_DIV2;   /* AHB 100 MHz */
    clk.APB1CLKDivider = RCC_APB1_DIV2;  /* APB 50 MHz, timer kernels 100 MHz */
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;
    if (!checked(HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2))) return 0;
    periph.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1;
    periph.PLL2.PLL2M = 4;
    periph.PLL2.PLL2N = 50;
    periph.PLL2.PLL2P = 8; /* 16/4*50/8 = 25 MHz ADC kernel */
    periph.PLL2.PLL2Q = 2;
    periph.PLL2.PLL2R = 2;
    periph.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
    periph.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
    periph.PLL2.PLL2FRACN = 0;
    periph.PLL3.PLL3M = 4;
    periph.PLL3.PLL3N = 96;
    periph.PLL3.PLL3P = 2;
    periph.PLL3.PLL3Q = 8; /* HSE-derived exact 48 MHz for native USB */
    periph.PLL3.PLL3R = 2;
    periph.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
    periph.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    periph.PLL3.PLL3FRACN = 0;
    periph.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    periph.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
    periph.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (!checked(HAL_RCCEx_PeriphCLKConfig(&periph))) return 0;
    HAL_PWREx_EnableUSBVoltageDetector(); /* Board supplies VDD33USB directly. */
    return 1;
}

static void gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    /* Shut all unused L298 inputs down; PA13 remains SWD until ARM. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_2, GPIO_PIN_RESET);
    g.Pin = GPIO_PIN_8; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_9 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_2; HAL_GPIO_Init(GPIOE, &g);
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLDOWN; HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1; HAL_GPIO_Init(GPIOE, &g);
    g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3; g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOA, &g);
}

static int pwm_init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    pwm_timer.Instance = TIM1;
    pwm_timer.Init.Prescaler = 0;
    pwm_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm_timer.Init.Period = 4999; /* 100 MHz / 5000 = 20 kHz */
    pwm_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pwm_timer.Init.RepetitionCounter = 0;
    pwm_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (!checked(HAL_TIM_PWM_Init(&pwm_timer))) return 0;
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1; oc.Pulse = 0; oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE; oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    if (!checked(HAL_TIM_PWM_ConfigChannel(&pwm_timer, &oc, TIM_CHANNEL_2))) return 0;
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_11; g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = GPIO_AF1_TIM1; HAL_GPIO_Init(GPIOE, &g);
    return checked(HAL_TIM_PWM_Start(&pwm_timer, TIM_CHANNEL_2));
}

static int adc_init(void)
{
    __HAL_RCC_ADC12_CLK_ENABLE(); __HAL_RCC_DMA1_CLK_ENABLE();
    adc_dma.Instance = DMA1_Stream0;
    adc_dma.Init.Request = DMA_REQUEST_ADC1;
    adc_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    adc_dma.Init.PeriphInc = DMA_PINC_DISABLE; adc_dma.Init.MemInc = DMA_MINC_ENABLE;
    adc_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    adc_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    adc_dma.Init.Mode = DMA_CIRCULAR; adc_dma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    adc_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (!checked(HAL_DMA_Init(&adc_dma))) return 0;
    adc.Instance = ADC1;
    __HAL_LINKDMA(&adc, DMA_Handle, adc_dma);
    adc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    adc.Init.Resolution = ADC_RESOLUTION_16B;
    adc.Init.ScanConvMode = ADC_SCAN_ENABLE;
    adc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    adc.Init.LowPowerAutoWait = DISABLE;
    adc.Init.ContinuousConvMode = DISABLE;
    adc.Init.NbrOfConversion = 3;
    adc.Init.DiscontinuousConvMode = DISABLE;
    adc.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
    adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    adc.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    adc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    adc.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    adc.Init.OversamplingMode = DISABLE;
    if (!checked(HAL_ADC_Init(&adc))) return 0;
    if (!checked(HAL_ADCEx_Calibration_Start(&adc, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED))) return 0;
    ADC_ChannelConfTypeDef ch = {0};
    ch.SamplingTime = ADC_SAMPLETIME_64CYCLES_5; ch.SingleDiff = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE; ch.Offset = 0; ch.OffsetSignedSaturation = DISABLE;
    const uint32_t channels[] = {ADC_CHANNEL_14, ADC_CHANNEL_15, ADC_CHANNEL_17};
    const uint32_t ranks[] = {ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3};
    for (int i = 0; i < 3; ++i) { ch.Channel = channels[i]; ch.Rank = ranks[i]; if (!checked(HAL_ADC_ConfigChannel(&adc, &ch))) return 0; }
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0); HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 1); HAL_NVIC_EnableIRQ(ADC_IRQn);
    return checked(HAL_ADC_Start_DMA(&adc, (uint32_t *)(uintptr_t)adc_raw, 3));
}

static int sampling_init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
    sampling_timer.Instance = TIM6;
    sampling_timer.Init.Prescaler = 99; /* 1 MHz microsecond timebase */
    sampling_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    sampling_timer.Init.Period = sample_period_us - 1;
    sampling_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (!checked(HAL_TIM_Base_Init(&sampling_timer))) return 0;
    TIM_MasterConfigTypeDef master = {0};
    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (!checked(HAL_TIMEx_MasterConfigSynchronization(&sampling_timer, &master))) return 0;
    /* Discard HAL's setup update before starting external ADC trigger. */
    __HAL_TIM_CLEAR_FLAG(&sampling_timer, TIM_FLAG_UPDATE);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0); HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    return checked(HAL_TIM_Base_Start_IT(&sampling_timer));
}

/* SSD1306 rendering uses a simple 5x7 ASCII font; no framebuffer heap. */
static const uint8_t font[64][5] = {
 {0,0,0,0,0},{0,0,95,0,0},{0,7,0,7,0},{20,127,20,127,20},
 {36,42,127,42,18},{35,19,8,100,98},{54,73,85,34,80},{0,5,3,0,0},
 {0,28,34,65,0},{0,65,34,28,0},{20,8,62,8,20},{8,8,62,8,8},
 {0,80,48,0,0},{8,8,8,8,8},{0,96,96,0,0},{32,16,8,4,2},
 {62,81,73,69,62},{0,66,127,64,0},{66,97,81,73,70},{33,65,69,75,49},
 {24,20,18,127,16},{39,69,69,69,57},{60,74,73,73,48},{1,113,9,5,3},
 {54,73,73,73,54},{6,73,73,41,30},{0,54,54,0,0},{0,86,54,0,0},
 {8,20,34,65,0},{20,20,20,20,20},{0,65,34,20,8},{2,1,81,9,6},
 {50,73,121,65,62},{126,17,17,17,126},{127,73,73,73,54},{62,65,65,65,34},
 {127,65,65,34,28},{127,73,73,73,65},{127,9,9,9,1},{62,65,73,73,122},
 {127,8,8,8,127},{0,65,127,65,0},{32,64,65,63,1},{127,8,20,34,65},
 {127,64,64,64,64},{127,2,12,2,127},{127,4,8,16,127},{62,65,65,65,62},
 {127,9,9,9,6},{62,65,81,33,94},{127,9,25,41,70},{70,73,73,73,49},
 {1,1,127,1,1},{63,64,64,64,63},{31,32,64,32,31},{63,64,56,64,63},
 {99,20,8,20,99},{7,8,112,8,7},{97,81,73,69,67},{0,127,65,65,0},
 {2,4,8,16,32},{0,65,65,127,0},{4,2,1,2,4},{64,64,64,64,64}
};
static uint8_t oled_frame[1024], oled_tx[129], oled_cmd[4];
static uint8_t oled_online, oled_dirty, oled_page, oled_phase;
static volatile uint8_t oled_i2c_busy;

static void oled_init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7; g.Mode = GPIO_MODE_AF_OD; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW; g.Alternate = GPIO_AF4_I2C1; HAL_GPIO_Init(GPIOB, &g);
    oled_i2c.Instance = I2C1;
    /* 50 MHz kernel, 100 kHz nominal, conservative rise/fall allowance. */
    oled_i2c.Init.Timing = 0x40922731;
    oled_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    oled_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    oled_i2c.Init.OwnAddress1 = 0; oled_i2c.Init.OwnAddress2 = 0;
    oled_i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    oled_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    oled_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&oled_i2c) != HAL_OK) return; /* Optional display. */
    (void)HAL_I2CEx_ConfigAnalogFilter(&oled_i2c, I2C_ANALOGFILTER_ENABLE);
    (void)HAL_I2CEx_ConfigDigitalFilter(&oled_i2c, 0);
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0); HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0); HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    HAL_Delay(100); /* Power-on only, before control timer starts. */
    const uint8_t init[] = {0x00,0xae,0xd5,0x80,0xa8,0x3f,0xd3,0x00,0x40,0x8d,0x14,0x20,0x02,0xa1,0xc8,0xda,0x12,0x81,0x7f,0xd9,0xf1,0xdb,0x40,0xa4,0xa6,0xaf};
    oled_online = HAL_I2C_Master_Transmit(&oled_i2c, 0x3c << 1, (uint8_t *)init, sizeof(init), 20) == HAL_OK;
}
static void oled_poll(void)
{
    if (!oled_online || oled_i2c_busy || !oled_dirty) return;
    HAL_StatusTypeDef status;
    oled_i2c_busy = 1;
    if (oled_phase == 0) {
        oled_cmd[0] = 0; oled_cmd[1] = 0xb0 | oled_page; oled_cmd[2] = 0; oled_cmd[3] = 0x10;
        status = HAL_I2C_Master_Transmit_IT(&oled_i2c, 0x3c << 1, oled_cmd, sizeof(oled_cmd));
        if (status == HAL_OK) oled_phase = 1;
    } else {
        oled_tx[0] = 0x40; memcpy(oled_tx + 1, oled_frame + 128U * oled_page, 128);
        status = HAL_I2C_Master_Transmit_IT(&oled_i2c, 0x3c << 1, oled_tx, sizeof(oled_tx));
        if (status == HAL_OK) { oled_phase = 0; if (++oled_page == 8) { oled_page = 0; oled_dirty = 0; } }
    }
    if (status != HAL_OK) { oled_i2c_busy = 0; if (status != HAL_BUSY) oled_online = 0; }
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *h) { if (h->Instance == I2C1) oled_i2c_busy = 0; }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h) { if (h->Instance == I2C1) { oled_i2c_busy = 0; oled_online = 0; } }
void I2C1_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&oled_i2c); }
void I2C1_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&oled_i2c); }

void board_init(void)
{
    SCB->VTOR = 0x08000000U;
    SCB_EnableICache(); /* Keep D-cache disabled: ADC DMA is coherent. */
    HAL_Init();
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_TIM1_CLK_ENABLE();
    if (!clocks_init()) return;
    gpio_init();
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST)) faults |= FAULT_WATCHDOG;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    /* Dedicated 32-bit free-running 1 MHz timer: no SysTick/DWT wrap races. */
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = 99; TIM2->ARR = 0xffffffffU; TIM2->CNT = 0;
    TIM2->EGR = TIM_EGR_UG; TIM2->SR = 0; TIM2->CR1 = TIM_CR1_CEN;
    if (!pwm_init()) return;
    oled_init();
    if (!checked(usb_transport_init() ? HAL_OK : HAL_ERROR)) return;
    if (!adc_init()) return;
    /* Independent watchdog approximately 0.5 s at nominal 32 kHz LSI.
     * Reset releases PA13, and converter external 10k pulldown disables ENB. */
    IWDG1->KR = 0xcccc; IWDG1->KR = 0x5555;
    IWDG1->PR = 4; IWDG1->RLR = 249;
    uint32_t watchdog_start = HAL_GetTick();
    while (IWDG1->SR) if (HAL_GetTick() - watchdog_start > 50U) { latch_fault(FAULT_INIT); break; }
    IWDG1->KR = 0xaaaa;
    /* Arm the sample timer last: watchdog register synchronization must not
     * consume a live control period during boot. */
    if (!sampling_init()) return;
    peripherals_ready = 1;
}
void board_poll(void)
{
    if (!peripherals_ready) return;
    IWDG1->KR = 0xaaaa;
    usb_transport_poll(); oled_poll();
}
uint32_t board_millis(void) { return HAL_GetTick(); }
uint32_t board_micros(void) { return TIM2->CNT; }
void SysTick_Handler(void) { HAL_IncTick(); }

void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&adc_dma); }
void ADC_IRQHandler(void) { HAL_ADC_IRQHandler(&adc); }
void TIM6_DAC_IRQHandler(void) { HAL_TIM_IRQHandler(&sampling_timer); }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *h)
{
    if (h->Instance != TIM6) return;
    if (sample_pending || control_busy || adc_inflight) latch_fault(FAULT_DEADLINE);
    adc_inflight = 1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *h)
{
    if (h->Instance != ADC1) return;
    const float scale = 3.3f * (55700.0f / 4700.0f) / 65535.0f;
    last_scan_valid = 1;
    for (int i = 0; i < 3; ++i) {
        captured[i] = adc_raw[i] * scale;
        if (adc_raw[i] >= 65500U) { last_scan_valid = 0; latch_fault(FAULT_ADC); }
    }
    capture_us = board_micros(); sample_pending = 1; adc_inflight = 0;
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *h) { if (h->Instance == ADC1) { last_scan_valid = 0; latch_fault(FAULT_ADC); } }
int board_sample_ready(float *v1, float *v2, float *vbus)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (!sample_pending) { __set_PRIMASK(mask); return 0; }
    *v1 = captured[0]; *v2 = captured[1]; *vbus = captured[2];
    sample_pending = 0; control_busy = 1; control_session = usb_transport_session();
    __set_PRIMASK(mask); return 1;
}
void board_set_output(float duty, int enable, int finite_set)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (!isfinite(duty) || duty < 0 || duty > 1 || (finite_set && duty != 0.0f && duty != 1.0f)) latch_fault(FAULT_ACTUATOR);
    if (control_busy && board_micros() - capture_us >= sample_period_us - 50U) latch_fault(FAULT_DEADLINE);
    if (!enable || faults || !peripherals_ready || !usb_transport_connected() || control_session != usb_transport_session()) { output_off(); control_busy = 0; __set_PRIMASK(mask); return; }
    if (!pa13_output) {
        GPIO_InitTypeDef g = {0}; g.Pin = GPIO_PIN_13; g.Mode = GPIO_MODE_OUTPUT_PP;
        g.Pull = GPIO_PULLDOWN; g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET); HAL_GPIO_Init(GPIOA, &g); pa13_output = 1;
    }
    TIM1->CCR2 = (uint32_t)(duty * 5000.0f + 0.5f);
    GPIOA->BSRR = GPIO_PIN_13; output_enabled = 1; control_busy = 0;
    __set_PRIMASK(mask);
}
int board_set_sample_us(uint32_t us)
{
    if (output_enabled || us < 500 || us > 10000 || !peripherals_ready) return 0;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    (void)HAL_TIM_Base_Stop_IT(&sampling_timer);
    (void)HAL_ADC_Stop_DMA(&adc);
    sample_period_us = us;
    TIM6->ARR = us - 1; TIM6->CNT = 0;
    /* UG is an ADC trigger: issue it only while ADC is stopped. */
    TIM6->EGR = TIM_EGR_UG; TIM6->SR = 0;
    sample_pending = control_busy = adc_inflight = 0;
    HAL_StatusTypeDef a = HAL_ADC_Start_DMA(&adc, (uint32_t *)(uintptr_t)adc_raw, 3);
    HAL_StatusTypeDef b = HAL_TIM_Base_Start_IT(&sampling_timer);
    __set_PRIMASK(mask);
    if (a != HAL_OK || b != HAL_OK) { latch_fault(FAULT_ADC); return 0; }
    return 1;
}
uint32_t board_faults(void) { return faults; }
void board_clear_faults(void)
{
    if (output_enabled || !peripherals_ready) return;
    if (adc.ErrorCode & (HAL_ADC_ERROR_INTERNAL | HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA)) {
        /* Re-arm acquisition first; require a subsequent healthy scan and CLEAR. */
        (void)board_set_sample_us(sample_period_us);
        return;
    }
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (last_scan_valid && !(adc.ErrorCode & (HAL_ADC_ERROR_INTERNAL | HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA)) &&
        board_micros() - capture_us < sample_period_us * 3U) {
        faults &= FAULT_INIT; sample_pending = control_busy = 0;
    }
    __set_PRIMASK(mask);
}
int board_usb_connected(void) { return usb_transport_connected(); }
uint32_t board_usb_session(void) { return usb_transport_session(); }
int board_usb_readline(char *dst, size_t capacity) { return usb_transport_readline(dst, capacity); }
int board_usb_write(const char *text, size_t length) { return usb_transport_write(text, length); }
uint8_t board_buttons(void)
{
    return ((GPIOB->IDR & GPIO_PIN_8) ? 1 : 0) | ((GPIOB->IDR & GPIO_PIN_9) ? 2 : 0) |
           ((GPIOE->IDR & GPIO_PIN_0) ? 4 : 0) | ((GPIOE->IDR & GPIO_PIN_1) ? 8 : 0);
}
void board_oled_lines(const char *const lines[8])
{
    if (!oled_online || oled_dirty || oled_i2c_busy) return; /* Drop old display frames. */
    memset(oled_frame, 0, sizeof(oled_frame));
    for (int page = 0; page < 8; ++page) {
        if (!lines[page]) continue;
        for (int col = 0; col < 21 && lines[page][col]; ++col) {
            unsigned char c = (unsigned char)toupper((unsigned char)lines[page][col]);
            if (c < 32 || c > 95) c = '?';
            memcpy(oled_frame + page * 128 + col * 6, font[c - 32], 5);
        }
    }
    oled_page = oled_phase = 0; oled_dirty = 1;
}
void NMI_Handler(void) { output_off(); for (;;) {} }
void HardFault_Handler(void) { output_off(); for (;;) {} }
void MemManage_Handler(void) { output_off(); for (;;) {} }
void BusFault_Handler(void) { output_off(); for (;;) {} }
void UsageFault_Handler(void) { output_off(); for (;;) {} }
