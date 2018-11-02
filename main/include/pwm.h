
#ifndef PWM_H
#define PWM_H

#define PWM_PIN GPIO_NUM_5

#define PWM_FULL 0.0f
#define PWM_OFF 100.0f
#define PWM_DIMM 60.0f

void pwm_init();
void pwm_set(float dutyCycle);

#endif