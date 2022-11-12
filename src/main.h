#pragma once

#include <map>
#include "GPSStatus.h"
#include "NodeStatus.h"
#include "PowerStatus.h"
#include "graphics/Screen.h"
#include "mesh/generated/telemetry.pb.h"
#ifndef ARCH_PORTDUINO
#include <SparkFun_ATECCX08a_Arduino_Library.h>
#endif

extern uint8_t screen_found;
extern uint8_t screen_model;
extern uint8_t kb_model;

extern bool eink_found;
extern bool pmu_found;
extern bool isCharging;
extern bool isUSBPowered;

#ifndef ARCH_PORTDUINO
extern ATECCX08A atecc;
#endif

typedef struct _scanmap {
    uint8_t addr;
    uint8_t bus;
} scanmap;

extern scanmap i2cScanMap[128];

extern uint8_t nodeTelemetrySensorsMap[_TelemetrySensorType_MAX + 1];

extern int TCPPort; // set by Portduino

// Global Screen singleton.
extern graphics::Screen *screen;
// extern Observable<meshtastic::PowerStatus> newPowerStatus; //TODO: move this to main-esp32.cpp somehow or a helper class

// extern meshtastic::PowerStatus *powerStatus;
// extern meshtastic::GPSStatus *gpsStatus;
// extern meshtastic::NodeStatusHandler *nodeStatusHandler;

// Return a human readable string of the form "Meshtastic_ab13"
const char *getDeviceName();

extern uint32_t timeLastPowered;

extern uint32_t rebootAtMsec;
extern uint32_t shutdownAtMsec;

extern uint32_t serialSinceMsec;

// If a thread does something that might need for it to be rescheduled ASAP it can set this flag
// This will supress the current delay and instead try to run ASAP.
extern bool runASAP;

void nrf52Setup(), esp32Setup(), nrf52Loop(), esp32Loop(), clearBonds();
