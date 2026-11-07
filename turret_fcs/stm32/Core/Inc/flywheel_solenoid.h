#ifndef __FLYWHEEL_SOLENOID_H
#define __FLYWHEEL_SOLENOID_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * ==============================================================================
 * ATIS SISTEMI SURUCUSU: Cift Flywheel (TB6612FNG) & Solenoid Itici
 * ==============================================================================
 * PB0: TB6612FNG PWMA & PWMB girisleri (TIM1_CH2N, 20 kHz ultrasonik sessiz PWM)
 * PB1: Solenoid valf MOSFET/Role tetik pini (GPIO Output Push-Pull)
 */

#define FLYWHEEL_GPIO_PORT   GPIOB
#define FLYWHEEL_PIN         GPIO_PIN_0   // TIM1_CH2N (AF1) - 20 kHz PWM

#define SOLENOID_GPIO_PORT   GPIOB
#define SOLENOID_PIN         GPIO_PIN_1   // Solenoid Valf Tetik Pini

// 20 kHz PWM Zamanlama (84 MHz APB2 timer saatinde ARR = 4199)
#define FLYWHEEL_PWM_ARR           4199u

// FF-180 (6V 23500 RPM) + Nerf Elite 2.0 Dart ile ~10 m/s icin nominal devir: 217 / 255 (~%85 Duty)
#define FLYWHEEL_DEFAULT_DUTY_PCT  217u
#define FLYWHEEL_DEFAULT_PULSE     ((uint32_t)(FLYWHEEL_PWM_ARR * FLYWHEEL_DEFAULT_DUTY_PCT) / 255u) // ~3574

// Soft-Start Rampa Suresi (ms) - 0'dan nominal devire yumusak artis (inrush akim korumasi)
#define FLYWHEEL_SOFT_START_MS     150u

// Zamanlama Ayarlari (Milisaniye)
#define SOLENOID_PULSE_MS    80u   // Solenoidin ileri itme enerjilenme suresi
#define SOLENOID_RELOAD_MS   150u  // Yayin mermiyi yuvaya cekme bekleme suresi

typedef enum {
    LAUNCHER_STATE_IDLE = 0,
    LAUNCHER_STATE_PUSHING,
    LAUNCHER_STATE_RELOADING
} LauncherState_t;

/* Fonksiyonlar */
void Launcher_Init(void);
void Launcher_Update(void);                 // Non-blocking durum makinesi ve soft-start rampasi

void Launcher_Flywheel_Enable(bool enable); // Motorlari soft-start ile ac / kapat
bool Launcher_IsFlywheelActive(void);       // Volanlar donuyor ve yeterli devre ulasti mi?
void Launcher_Flywheel_SetDuty(uint8_t duty_pct); // Nominal duty degistirme (0..100)
uint8_t Launcher_Flywheel_GetDuty(void);

/*
 * Atis tahrigini baslatir. Volanlar hazir degilse atisi REDDEDER (false).
 * Volan spin-up'i PC zaman cizelgesi (SPINUP fazi) yonetir.
 */
bool Launcher_TriggerFire(void);

bool Launcher_IsFiring(void);               // Su an atis icra ediliyor mu?
void Launcher_EmergencyStop(void);          // Volan (aninda %0 PWM) ve solenoidi keser

#endif /* __FLYWHEEL_SOLENOID_H */
