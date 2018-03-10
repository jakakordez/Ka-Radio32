
#ifndef ALARM_H
#define ALARM_H

#include "eeprom.h"

void startAlarm(struct device_settings *device);
void setAlarm(uint32_t time);
uint32_t getAlarm();
void processAlarm();
void disableAlarm();

#endif