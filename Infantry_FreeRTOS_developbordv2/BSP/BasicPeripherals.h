#ifndef __BASEICPERIPHERALS_H
#define __BASEICPERIPHERALS_H

#include "sys.h"

#define LED_GREEN  PFout(14)
#define LED_RED      PEout(11)
#define LASER      PGout(13)
#define LED1   PGout(1)
#define LED2   PGout(2)
#define LED3   PGout(3)
#define LED4   PGout(4)
#define LED5   PGout(5)
#define LED6   PGout(6)
#define LED7   PGout(7)
#define LED8   PGout(8)

#define BEEP_NOTE_1  3814
#define BEEP_NOTE_2  3401
#define BEEP_NOTE_3  3030
#define BEEP_NOTE_4  2865
#define BEEP_NOTE_5  2551
#define BEEP_NOTE_6  2273
#define BEEP_NOTE_7  2024

extern void led_green_on(void);
extern void led_green_off(void);
extern void led_green_toggle(void);

extern void led_red_on(void);
extern void led_red_off(void);
extern void led_red_toggle(void);

extern void flow_led_on(uint16_t num);
extern void flow_led_off(uint16_t num);
extern void flow_led_toggle(uint16_t num);

void BasicPreiph_Init(void);
void StrartingMusic(void);
void FrictionCaliMusic(void);
void FrictionCaliTriggerMusic(void);
void ImuCaliMusic(void);
void buzzer_off(void);
void buzzer_on(uint16_t psc, uint16_t pwm);

#endif

