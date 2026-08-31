#include "main.h"
#include <stdio.h>

typedef struct
{
  uint32_t perioda_otkucaja;
  uint32_t impuls_otkucaja;
} Mjerenje_t;

typedef struct
{
  uint32_t frekvencija_hz;
  uint32_t radni_ciklus;
} RadnaTocka_t;

#define PWM_PREDDJELITELJ   83U
#define RAZMAK_ISPISA_MS    250U
#define TRAJANJE_TOCKE_MS   3000U

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

static volatile Mjerenje_t mjerenje      = {0};
static volatile uint8_t    novo_mjerenje = 0;
static volatile uint32_t   broj_prekida  = 0;

static const RadnaTocka_t radne_tocke[] =
{
  {  500U, 10U },
  { 1000U, 25U },
  { 2000U, 50U },
  { 5000U, 75U },
};
#define BROJ_RADNIH_TOCAKA (sizeof(radne_tocke) / sizeof(radne_tocke[0]))

static uint32_t takt_tim2_hz = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static uint32_t TaktTimeraAPB1(void);
static uint32_t TaktTimeraAPB2(void);
static void     PostaviMjerac(void);
static void     PostaviPWM(uint32_t frekvencija_hz, uint32_t radni_ciklus);
static void     IspisiMjerenje(const Mjerenje_t *m, const RadnaTocka_t *zadano);

static void PostaviMjerac(void)
{
  GPIO_InitTypeDef        gpio   = {0};
  TIM_SlaveConfigTypeDef  slave  = {0};
  TIM_IC_InitTypeDef      kanal  = {0};
  TIM_MasterConfigTypeDef master = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin       = GPIO_PIN_0;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLDOWN;
  gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 0xFFFFFFFFUL;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  slave.SlaveMode       = TIM_SLAVEMODE_RESET;
  slave.InputTrigger    = TIM_TS_TI1FP1;
  slave.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  slave.TriggerFilter   = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &slave) != HAL_OK)
  {
    Error_Handler();
  }

  kanal.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  kanal.ICSelection = TIM_ICSELECTION_DIRECTTI;
  kanal.ICPrescaler = TIM_ICPSC_DIV1;
  kanal.ICFilter    = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &kanal, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  kanal.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  kanal.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &kanal, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK)
  {
    Error_Handler();
  }
}

int __io_putchar(int znak)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&znak, 1, HAL_MAX_DELAY);
  return znak;
}

static uint32_t TaktTimeraAPB1(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  return ((RCC->CFGR & RCC_CFGR_PPRE1) == 0U) ? pclk1 : (pclk1 * 2U);
}

static uint32_t TaktTimeraAPB2(void)
{
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  return ((RCC->CFGR & RCC_CFGR_PPRE2) == 0U) ? pclk2 : (pclk2 * 2U);
}

static void PostaviPWM(uint32_t frekvencija_hz, uint32_t radni_ciklus)
{
  uint32_t otkucaja_hz = TaktTimeraAPB2() / (PWM_PREDDJELITELJ + 1U);
  uint32_t arr         = (otkucaja_hz / frekvencija_hz) - 1U;
  uint32_t ccr         = ((arr + 1U) * radni_ciklus) / 100U;

  __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

static void IspisiMjerenje(const Mjerenje_t *m, const RadnaTocka_t *zadano)
{
  uint64_t perioda_ns      = ((uint64_t)m->perioda_otkucaja * 1000000000ULL) / takt_tim2_hz;
  uint64_t frekvencija_mhz = ((uint64_t)takt_tim2_hz * 1000ULL) / m->perioda_otkucaja;
  uint32_t ciklus_d10      = (uint32_t)(((uint64_t)m->impuls_otkucaja * 1000ULL) / m->perioda_otkucaja);

  printf("T = %lu.%03lu us | f = %lu.%03lu Hz | D = %lu.%lu %% | CCR1=%lu CCR2=%lu | zadano: %lu Hz / %lu %%\r\n",
         (unsigned long)(perioda_ns / 1000ULL),
         (unsigned long)(perioda_ns % 1000ULL),
         (unsigned long)(frekvencija_mhz / 1000ULL),
         (unsigned long)(frekvencija_mhz % 1000ULL),
         (unsigned long)(ciklus_d10 / 10U),
         (unsigned long)(ciklus_d10 % 10U),
         (unsigned long)m->perioda_otkucaja,
         (unsigned long)m->impuls_otkucaja,
         (unsigned long)zadano->frekvencija_hz,
         (unsigned long)zadano->radni_ciklus);
}

int main(void)
{
  uint32_t t_ispis = 0;
  uint32_t t_tocka = 0;
  uint32_t tocka   = 0;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  PostaviMjerac();

  setvbuf(stdout, NULL, _IONBF, 0);
  takt_tim2_hz = TaktTimeraAPB1();

  printf("\r\n=============================================\r\n");
  printf(" URS - PWM generacija i mjerenje ulaznim hvatanjem\r\n");
  printf(" STM32F446RE @ %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
  printf(" TIM1_CH1 (PA8) --zica--> TIM2_CH1 (PA0)\r\n");
  printf(" Takt TIM1 = %lu Hz, takt TIM2 = %lu Hz\r\n",
         (unsigned long)TaktTimeraAPB2(), (unsigned long)takt_tim2_hz);
  printf(" Rezolucija mjerenja = %lu ps\r\n",
         (unsigned long)(1000000000000ULL / takt_tim2_hz));
  printf("=============================================\r\n\r\n");

  PostaviPWM(radne_tocke[0].frekvencija_hz, radne_tocke[0].radni_ciklus);
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  printf("Generator i mjerac pokrenuti.\r\n");
  printf("\r\n>>> Radna tocka 1/%lu: %lu Hz, %lu %%\r\n",
         (unsigned long)BROJ_RADNIH_TOCAKA,
         (unsigned long)radne_tocke[0].frekvencija_hz,
         (unsigned long)radne_tocke[0].radni_ciklus);

  while (1)
  {
    uint32_t sada = HAL_GetTick();

    if ((sada - t_tocka) >= TRAJANJE_TOCKE_MS)
    {
      t_tocka = sada;
      tocka   = (tocka + 1U) % BROJ_RADNIH_TOCAKA;
      PostaviPWM(radne_tocke[tocka].frekvencija_hz, radne_tocke[tocka].radni_ciklus);
      printf("\r\n>>> Radna tocka %lu/%lu: %lu Hz, %lu %%\r\n",
             (unsigned long)(tocka + 1U),
             (unsigned long)BROJ_RADNIH_TOCAKA,
             (unsigned long)radne_tocke[tocka].frekvencija_hz,
             (unsigned long)radne_tocke[tocka].radni_ciklus);
    }

    if ((sada - t_ispis) >= RAZMAK_ISPISA_MS)
    {
      Mjerenje_t kopija;
      uint8_t    ima_podatka;

      __disable_irq();
      kopija        = *(Mjerenje_t *)&mjerenje;
      ima_podatka   = novo_mjerenje;
      novo_mjerenje = 0U;
      __enable_irq();

      t_ispis = sada;

      if (ima_podatka != 0U)
      {
        IspisiMjerenje(&kopija, &radne_tocke[tocka]);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }
      else
      {
        printf("Nema mjerenja - provjeri zicu PA8 (D7) <-> PA0 (A0).\r\n");
      }
    }
  }
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2)
  {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uint32_t perioda = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t impuls  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    broj_prekida++;

    if (perioda != 0U)
    {
      mjerenje.perioda_otkucaja = perioda;
      mjerenje.impuls_otkucaja  = impuls;
      novo_mjerenje             = 1U;
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
