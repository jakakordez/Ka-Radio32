
#ifndef PWM_H
#define PWM_H

#define PWM_PIN GPIO_NUM_12

void pwm_init();
void pwm_set(float dutyCycle);
void pwm_process();

#endif