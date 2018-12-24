#ifndef __BASEICPERIPHERALS_H
#define __BASEICPERIPHERALS_H

#include "sys.h"

#define LED_GREEN  PFout(14)
#define LED_RED      PEout(7)
#define LASER      PGout(13)

#define BEEP_NOTE_1  3814
#define BEEP_NOTE_2  3401
#define BEEP_NOTE_3  3030
#define BEEP_NOTE_4  2865
#define BEEP_NOTE_5  2551
#define BEEP_NOTE_6  2273
#define BEEP_NOTE_7  2024

void BasicPreiph_Init(void);
void StrartingMusic(void);
void FrictionCaliMusic(void);
void FrictionCaliTriggerMusic(void);
void ImuCaliMusic(void);

#endif

