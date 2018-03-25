
#include "alarm.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include "ntp.h"

#include "webclient.h"
#include "webserver.h"
#include "interface.h"
#include "eeprom.h"

#define TAG  "alarm"

static struct tm *dt;

enum ALARM_STATE{
	ALARM_DISABLED,
	ALARM_SET,
	ALARM_TRIGGERED
};

static uint32_t alarmTime;
static enum ALARM_STATE currentState;

void saveAlarmSettings(){
	struct device_settings *device;
	device = getDeviceSettings();
	if (device != NULL)	 {
		device->alarmTime = alarmTime;
		device->alarmState = currentState;
		saveDeviceSettings(device);
		ESP_LOGV(TAG,"Alarm saved");
		free(device);
	}
}

void startAlarm(struct device_settings *device){
	alarmTime = device->alarmTime;
	currentState = device->alarmState;
}

void setAlarm(uint32_t time){
	alarmTime = time;
	currentState = ALARM_SET;
	saveAlarmSettings();
}

void disableAlarm(){
	currentState = ALARM_DISABLED;
	saveAlarmSettings();
}

uint32_t getAlarm(){
	if(currentState == ALARM_DISABLED) return 24*60;
	else return alarmTime;
}

void processAlarm(time_t timestamp){
	dt=localtime(&timestamp);
	uint32_t currentTime = (dt->tm_hour*60)+dt->tm_min;
	if(currentTime == alarmTime){
		if(currentState == ALARM_SET){
			kprintf("Alarm triggered\n");
			currentState = ALARM_TRIGGERED;
			saveAlarmSettings();
			playStationInt(getCurrentStation());
			kprintf("Radio playing\n");
		}
	}
	else if(currentState == ALARM_TRIGGERED){
		currentState = ALARM_SET;
		saveAlarmSettings();
	}
}