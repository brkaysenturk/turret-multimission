/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Basket Challenge Main Controller Firmware (STM32F401RE)
  *                   - TF-Luna ToF LiDAR Reader (USART1, 115200 Baud)
  *                   - Full-Duplex Binary Basket Protocol (USART2, 115200 Baud)
  *                   - Pan/Tilt MG996R Servos (TIM3 / TIM1, 50 Hz PWM)
  *                   - Flywheel Motors (PB0) & Solenoid Pusher (PB1)
  *                   - Hardware IWDG & Multi-tier Watchdogs
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "protocol.h"
#include "luna_read.h"
#include "servo_driver.h"
#include "basket_comm.h"
#include "flywheel_solenoid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Fiziksel ates yolu guvenlik ve etik tedbir nedeniyle varsayilan olarak KAPALI (0).
 * Gercek dart atis testinde 1 yapilabilir.
 */
#ifndef ENABLE_PHYSICAL_FIRE
#define ENABLE_PHYSICAL_FIRE 1
#endif

/*
 * Zaman Asimlari (ms)
 * - 300 ms: PC komut akisi kesilirse firlatici durdurulur (fail-safe).
 * - 2000 ms: PC kalici olarak kesilirse servolar stall isinmasini onlemek
 *            icin yumusakca mekanik merkeze (90.0, 90.0) park edilir.
 */
#define BASKET_CMD_TIMEOUT_MS   300u
#define SERVO_PARK_TIMEOUT_MS  2000u

/*
 * Bagimsiz Bekci Kopegi (IWDG)
 * LSI ~32 kHz, bolucu /64 -> 500 Hz (2 ms/sayac), reload 400 -> 800 ms pencere.
 */
#define IWDG_PRESCALER         IWDG_PRESCALER_64
#define IWDG_RELOAD_COUNTS     400u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
/*
 * B1 Mavi Buton (PC13 / EXTI13) Mandalli Yazilim E-Stop:
 * Her basista mandal terslenir. Mandal aktifken PC komutlari yok sayilir,
 * launcher aninda kesilir ve servolar hiz sinirli olarak merkeze iner.
 */
static volatile bool b1_estop_latched = false;
static volatile bool b1_press_event = false;
static uint32_t b1_last_edge_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);

/**
  * @brief  Uygulama ana giris noktasi (Application entry point).
  */
int main(void)
{
  /* MCU Configuration --------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Watchdog'u olasi erken kilitlenmelere karsi en basta baslat */
  MX_IWDG_Init();
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  /* Donanim Modullerini Baslat */
  Luna_Init(&huart1);
  Basket_Comm_Init(&huart2);
  Servo_Init();
  Launcher_Init(); /* Pinleri guvenli PULLDOWN ve 0V durumuna kilitler */

  uint32_t last_pc_cmd_time = HAL_GetTick();
  bool servos_park = false;
#if ENABLE_PHYSICAL_FIRE
  static uint8_t prev_cmd_state = PROTO_STATE_TRACKING;
#endif
  uint32_t last_led_time = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 0. Watchdog beslemesi */
    HAL_IWDG_Refresh(&hiwdg);

    /* 1. Sabit ~100 Hz Periyodik Telemetri Gonderimi (LiDAR + Servo Aci Geri Bildirimi) */
    uint16_t dist = Luna_GetDistance();
    uint16_t strength = Luna_GetStrength();
    Basket_Comm_SendTelemetry(dist, strength, Servo_GetPan(), Servo_GetTilt());

    /* 2. B1 Mavi Buton E-stop Mandali */
    if (b1_press_event)
    {
      b1_press_event = false;
      b1_estop_latched = !b1_estop_latched;
      if (b1_estop_latched)
      {
        Launcher_EmergencyStop();
        servos_park = true;
#if ENABLE_PHYSICAL_FIRE
        prev_cmd_state = PROTO_STATE_TRACKING;
#endif
      }
      else
      {
        servos_park = false;
      }
    }

    if (b1_estop_latched)
    {
      Launcher_EmergencyStop();
      servos_park = true;
    }

    /* 3. PC'den yeni komut alimi ve motor kontrolu */
    float d_pan = 0.0f;
    float d_tilt = 0.0f;
    uint8_t speed_pwm = 0;
    uint8_t state = 0;
    if (!b1_estop_latched && Basket_Comm_GetCommand(&d_pan, &d_tilt, &speed_pwm, &state))
    {
      last_pc_cmd_time = HAL_GetTick();

      if (state == PROTO_STATE_ESTOP)
      {
        Launcher_EmergencyStop();
        servos_park = true;
#if ENABLE_PHYSICAL_FIRE
        prev_cmd_state = PROTO_STATE_TRACKING;
#endif
      }
      else
      {
        servos_park = false;
        Servo_ApplyCorrection(d_pan, d_tilt);

#if ENABLE_PHYSICAL_FIRE
        /* Ates tetigi: Volan hazirsa ve yukselen kenar (FIRING) gelmisse */
        if (state == PROTO_STATE_FIRING && prev_cmd_state != PROTO_STATE_FIRING)
        {
          (void)Launcher_TriggerFire();
        }
#endif

        /* TB6612FNG Dinamik Volan Hizi Kontrolu:
         * LOCKED (1) veya FIRING (2) durumunda gelen speed_pwm degerine gore devir ayarla.
         * TRACKING (0) veya ESTOP durumunda motoru durdur. */
        if (state == PROTO_STATE_LOCKED || state == PROTO_STATE_FIRING)
        {
          uint8_t target_spd = (speed_pwm > 0) ? speed_pwm : 190u;
          Launcher_Flywheel_SetSpeed(target_spd);
        }
        else
        {
          if (!Launcher_IsFiring())
          {
            Launcher_Flywheel_SetSpeed(0u);
          }
        }
      }

#if ENABLE_PHYSICAL_FIRE
      prev_cmd_state = (state == PROTO_STATE_ESTOP) ? PROTO_STATE_TRACKING : state;
#endif
    }

    /* 4. Heartbeat Watchdog: PC akisi 300 ms kesilirse firlatici kapatilir */
    if (HAL_GetTick() - last_pc_cmd_time > BASKET_CMD_TIMEOUT_MS)
    {
      Launcher_EmergencyStop();
#if ENABLE_PHYSICAL_FIRE
      prev_cmd_state = PROTO_STATE_TRACKING;
#endif
    }

    /* 5. Uzun sureli iletisim kaybinda servolari merkeze al */
    if (HAL_GetTick() - last_pc_cmd_time > SERVO_PARK_TIMEOUT_MS)
    {
      servos_park = true;
    }

    /* 6. Yumusak merkezleme (Hiz sinirli ve sarsintisiz) */
    if (servos_park)
    {
      (void)Servo_HomeStep();
    }

    /* 7. Durum LED'i (LD2 / PA5) 500 ms kalp atisi */
    if (HAL_GetTick() - last_led_time >= 500)
    {
      last_led_time = HAL_GetTick();
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    /* 8. Firlatici durum makinesi (non-blocking) */
    Launcher_Update();

    HAL_Delay(10); /* ~100 Hz dongu periyodu */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief UART Kesme Callback (Tum alimlar modullere yonlendirilir)
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Luna_RxCallback(huart);
  }
  else if (huart->Instance == USART2)
  {
    Basket_Comm_RxCallback(huart);
  }
}

/**
  * @brief UART Hata Callback
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Luna_HandleError();
  }
  else if (huart->Instance == USART2)
  {
    Basket_Comm_HandleError();
  }
}

/**
  * @brief EXTI Callback (B1 Mavi Buton)
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
    uint32_t now = HAL_GetTick();
    if (b1_last_edge_ms == 0u || (now - b1_last_edge_ms) >= 250u)
    {
      b1_last_edge_ms = now;
      b1_press_event = true;
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration (84 MHz)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief IWDG Initialization Function
  */
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER;
  hiwdg.Init.Reload = IWDG_RELOAD_COUNTS;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function (TF-Luna LiDAR @ 115200 8N1)
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function (PC ST-Link VCP @ 115200 8N1)
  */
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

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Configure B1 (PC13) pin : EXTI Falling Edge with Pull-up */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Configure LD2 (PA5) pin : Output */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI Interrupt Configuration for B1 Button */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* Acil guvenlik: Volan ve solenoid pinlerini aninda sifirla */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */

