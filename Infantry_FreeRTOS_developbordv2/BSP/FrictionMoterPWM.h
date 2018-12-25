#ifndef _FRICTIONMOTERPWM_H_
#define _FRICTIONMOTERPWM_H_

#define FRICTION_MOTER_L TIM2->CCR1
#define FRICTION_MOTER_R TIM2->CCR2

void FrictionMoterPWM_Init(void);

#endif

