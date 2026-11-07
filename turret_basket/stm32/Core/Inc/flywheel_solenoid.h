/*
 * ==============================================================================
 * MODUL: BASKET FIRLATICI SURUCUSU (FF-180 + TB6612FNG & Solenoid Itici)
 * ==============================================================================
 * Cift FF-180 volan motorlarini (TB6612FNG surucu uzerinden 20 kHz PWM ile PB0)
 * ve dart itici solenoid valfini (PB1) islemciyi bloklamayan zamanlama ile surer.
 * ==============================================================================
 */
#ifndef __FLYWHEEL_SOLENOID_H
#define __FLYWHEEL_SOLENOID_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Pin Tanimlari */
#define FLYWHEEL_GPIO_PORT   GPIOB
#define FLYWHEEL_PIN         GPIO_PIN_0   /* TB6612FNG PWMA+PWMB 20 kHz PWM Pini (TIM1_CH2N) */

#define SOLENOID_GPIO_PORT   GPIOB
#define SOLENOID_PIN         GPIO_PIN_1   /* Solenoid Valf MOSFET/Opto Tetik Pini */

/* TB6612FNG 20 kHz PWM ARR Sabiti (84 MHz / 4200 = 20.000 Hz) */
#define FLYWHEEL_PWM_ARR     4199u

/* Soft-Start Rampa Suresi (ms) - 0'dan nominal devire yumusak artis (inrush akim korumasi) */
#define FLYWHEEL_SOFT_START_MS 150u

/* Zamanlama Ayarlari (Milisaniye) */
#define SOLENOID_PULSE_MS    80u   /* Solenoid ileri itme enerjilenme suresi */
#define SOLENOID_RELOAD_MS   150u  /* Yayin dart haznesini geri cekme suresi */

typedef enum {
    LAUNCHER_STATE_IDLE = 0,
    LAUNCHER_STATE_PUSHING,
    LAUNCHER_STATE_RELOADING
} LauncherState_t;

void Launcher_Init(void);
void Launcher_Update(void);              /* Non-blocking durum makinesi */

/* Dinamik Hiz Kontrolu: 0-255 arasi degeri 20 kHz PWM duty oranina donusturur */
void Launcher_Flywheel_SetSpeed(uint8_t speed_0_255);
uint8_t Launcher_Flywheel_GetSpeed(void);

void Launcher_Flywheel_Enable(bool enable); /* Nominal devirde (%75 / 190) ac/kapat */
bool Launcher_IsFlywheelActive(void);      /* Volanlar su an enerjili mi? */

/* Atis tahrigini baslatir. Volanlar hazir degilse atisi reddeder (false). */
bool Launcher_TriggerFire(void);

bool Launcher_IsFiring(void);            /* Su an atis icra ediliyor mu? */
void Launcher_EmergencyStop(void);       /* Volan ve solenoidi aninda durdurur (Fail-safe) */

#endif /* __FLYWHEEL_SOLENOID_H */
