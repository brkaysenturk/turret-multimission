/*
 * ==============================================================================
 * MODUL: VOLAN VE SOLENOID FIRLATICI SURUCUSU (TB6612FNG + FF-180 & SOLENOID)
 * ==============================================================================
 * PB0: TB6612FNG PWMA+PWMB girisleri (TIM1_CH2N, AF1, 20 kHz sessiz PWM)
 * PB1: Solenoid itici MOSFET/Role (GPIO_Output Pull-down)
 *
 * SOFT-START & GUVENLIK:
 * - 0'dan nominal %85 duty'ye (10 m/s atis hizi) 150 ms lineer rampa ile kalkar.
 *   Boylece inrush akim piki, voltaj cökmesi ve STM32 kahverengi resetleri onlenir.
 * - E-stop durumunda beklemeden aninda %0 PWM ve solenoid kesilir.
 * - Atis tahrigi (Launcher_TriggerFire), volan devri hedef devrin %75'inin
 *   altindaysa dartin namluda sikismasini onlemek icin atisi reddeder.
 * ==============================================================================
 */
#include "flywheel_solenoid.h"

static LauncherState_t current_state = LAUNCHER_STATE_IDLE;
static uint32_t state_timer = 0;

static TIM_HandleTypeDef htim_flywheel;
static bool flywheel_enabled = false;
static uint8_t configured_duty_pct = FLYWHEEL_DEFAULT_DUTY_PCT;
static uint32_t target_pulse = FLYWHEEL_DEFAULT_PULSE;
static uint32_t current_pulse = 0;

static uint32_t ramp_start_ms = 0;
static uint32_t ramp_start_pulse = 0;

void Launcher_Init(void)
{
    // GPIO ve TIM1 Saatlerini aktif et
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* 1. PB1: Solenoid GPIO Cikisi */
    GPIO_InitTypeDef gpio_sol = {0};
    gpio_sol.Pin = SOLENOID_PIN;
    gpio_sol.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_sol.Pull = GPIO_PULLDOWN;
    gpio_sol.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SOLENOID_GPIO_PORT, &gpio_sol);
    HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_RESET);

    /* 2. PB0: TB6612FNG PWMA+PWMB 20 kHz PWM Cikisi (TIM1_CH2N, AF1) */
    GPIO_InitTypeDef gpio_pwm = {0};
    gpio_pwm.Pin = FLYWHEEL_PIN;
    gpio_pwm.Mode = GPIO_MODE_AF_PP;
    gpio_pwm.Pull = GPIO_PULLDOWN;
    gpio_pwm.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_pwm.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(FLYWHEEL_GPIO_PORT, &gpio_pwm);

    htim_flywheel.Instance = TIM1;
    htim_flywheel.Init.Prescaler = 0;              /* 84 MHz tick */
    htim_flywheel.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_flywheel.Init.Period = FLYWHEEL_PWM_ARR;  /* 84 MHz / 4200 = 20.000 Hz PWM */
    htim_flywheel.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_flywheel.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim_flywheel);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;                                  /* %0 duty ile basla */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim_flywheel, &oc, TIM_CHANNEL_2);

    TIM_BreakDeadTimeConfigTypeDef bdt = {0};
    bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim_flywheel, &bdt);

    HAL_TIMEx_PWMN_Start(&htim_flywheel, TIM_CHANNEL_2);

    current_state = LAUNCHER_STATE_IDLE;
    flywheel_enabled = false;
    current_pulse = 0;
    configured_duty_pct = FLYWHEEL_DEFAULT_DUTY_PCT;
    target_pulse = FLYWHEEL_DEFAULT_PULSE;
}

void Launcher_EmergencyStop(void)
{
    // Acil durum: Volan PWM'ini ve solenoidi aninda kes (sifirla)
    flywheel_enabled = false;
    current_pulse = 0;
    __HAL_TIM_SET_COMPARE(&htim_flywheel, TIM_CHANNEL_2, 0);

    HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_RESET);
    current_state = LAUNCHER_STATE_IDLE;
}

void Launcher_Flywheel_SetDuty(uint8_t duty_255)
{
    configured_duty_pct = duty_255;
    target_pulse = ((uint32_t)FLYWHEEL_PWM_ARR * configured_duty_pct) / 255u;
}

uint8_t Launcher_Flywheel_GetDuty(void)
{
    return configured_duty_pct;
}

void Launcher_Flywheel_Enable(bool enable)
{
    if (enable)
    {
        if (!flywheel_enabled)
        {
            flywheel_enabled = true;
            ramp_start_ms = HAL_GetTick();
            ramp_start_pulse = current_pulse;
        }
    }
    else
    {
        flywheel_enabled = false;
        current_pulse = 0;
        __HAL_TIM_SET_COMPARE(&htim_flywheel, TIM_CHANNEL_2, 0);
    }
}

bool Launcher_IsFlywheelActive(void)
{
    // Volan donuyor ve en az %75 devire ulasti mi?
    return (flywheel_enabled && (current_pulse >= (target_pulse * 3u / 4u)));
}

bool Launcher_TriggerFire(void)
{
    if (current_state != LAUNCHER_STATE_IDLE)
    {
        return false;
    }

    // Volan aktif degilse veya devrini alamadiysa atisi guvenlik icin reddet
    if (!Launcher_IsFlywheelActive())
    {
        return false;
    }

    HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_SET);
    state_timer = HAL_GetTick();
    current_state = LAUNCHER_STATE_PUSHING;
    return true;
}

void Launcher_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. Volan Soft-Start Rampa Guncellemesi */
    if (flywheel_enabled)
    {
        uint32_t elapsed = now - ramp_start_ms;
        if (elapsed >= FLYWHEEL_SOFT_START_MS)
        {
            current_pulse = target_pulse;
        }
        else
        {
            current_pulse = ramp_start_pulse +
                ((target_pulse - ramp_start_pulse) * elapsed) / FLYWHEEL_SOFT_START_MS;
        }
        __HAL_TIM_SET_COMPARE(&htim_flywheel, TIM_CHANNEL_2, (uint16_t)current_pulse);
    }
    else
    {
        if (current_pulse > 0)
        {
            current_pulse = 0;
            __HAL_TIM_SET_COMPARE(&htim_flywheel, TIM_CHANNEL_2, 0);
        }
    }

    /* 2. Solenoid Non-blocking Durum Makinesi */
    switch (current_state)
    {
        case LAUNCHER_STATE_IDLE:
            break;

        case LAUNCHER_STATE_PUSHING:
            if (now - state_timer >= SOLENOID_PULSE_MS)
            {
                HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_RESET);
                state_timer = now;
                current_state = LAUNCHER_STATE_RELOADING;
            }
            break;

        case LAUNCHER_STATE_RELOADING:
            if (now - state_timer >= SOLENOID_RELOAD_MS)
            {
                current_state = LAUNCHER_STATE_IDLE;
            }
            break;

        default:
            Launcher_EmergencyStop();
            break;
    }
}

bool Launcher_IsFiring(void)
{
    return (current_state != LAUNCHER_STATE_IDLE);
}
