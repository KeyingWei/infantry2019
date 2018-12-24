#ifndef _FRICTIONMOTERPWM_H_
#define _FRICTIONMOTERPWM_H_

#define FRICTION_MOTER_L TIM12->CCR1
#define FRICTION_MOTER_R TIM12->CCR2

void FrictionMoterPWM_Init(void);

#endif

