#include "kbI2cBase.h"
#include "configuration.h"
#include "main.h"
#include <Wire.h>


KbI2cBase::KbI2cBase(const char *name) : concurrency::OSThread(name)
{
    this->_originName = name;
}

uint8_t read_from_14004(uint8_t reg, uint8_t *data, uint8_t length, TwoWire myWire)
{
  uint8_t readflag = 0;
  myWire.beginTransmission(CARDKB_ADDR);
  myWire.write(reg);
  myWire.endTransmission();    // stop transmitting
  delay(20);
  myWire.requestFrom(CARDKB_ADDR, (int)length);
  int i = 0;
  while ( myWire.available() ) // slave may send less than requested
  {
    data[i++] = myWire.read(); // receive a byte as a proper uint8_t
    readflag = 1;
  }
  return readflag;
}

void write_to_14004(uint8_t reg, uint8_t data, TwoWire myWire)
{
  myWire.beginTransmission(CARDKB_ADDR);
  myWire.write(reg);
  myWire.write(data);
  myWire.endTransmission();    // stop transmitting
}

int32_t KbI2cBase::runOnce()
{
    if (i2cScanMap[CARDKB_ADDR].addr != CARDKB_ADDR) {
        // Input device is not detected.
        return INT32_MAX;
    }

    TwoWire myWire = Wire;

    if (i2cScanMap[CARDKB_ADDR].bus == 1) {
#ifdef I2C_SDA1
        myWire = Wire1;
#endif
    }

    if (kb_model == 0x02) {
        // RAK14004
        uint8_t rDataBuf[8] = {0};
        uint8_t PrintDataBuf = 0;
        if (read_from_14004(0x01, rDataBuf, 0x04, myWire) == 1) {
            for (uint8_t aCount = 0; aCount < 0x04; aCount++) {
                for (uint8_t bCount = 0; bCount < 0x04; bCount++ ) {
                    if (((rDataBuf[aCount] >> bCount) & 0x01) == 0x01) {
                        PrintDataBuf = aCount * 0x04 + bCount + 1;
                    }
                }
            }
        }
        if (PrintDataBuf != 0) {
            DEBUG_MSG("RAK14004 key 0x%x pressed\n", PrintDataBuf);
            InputEvent e;
            e.inputEvent = MATRIXKEY;
            e.source = this->_originName;
            e.kbchar = PrintDataBuf;
            this->notifyObservers(&e);
        }
    } else {
        // m5 cardkb
        myWire.requestFrom(CARDKB_ADDR, 1);

        while (myWire.available()) {
            char c = myWire.read();
            InputEvent e;
            e.inputEvent = ModuleConfig_CannedMessageConfig_InputEventChar_NONE;
            e.source = this->_originName;
            switch (c) {
            case 0x1b: // ESC
                e.inputEvent = ModuleConfig_CannedMessageConfig_InputEventChar_CANCEL;
                break;
            case 0x08: // Back
                e.inputEvent = ModuleConfig_CannedMessageConfig_InputEventChar_BACK;
                e.kbchar = c;
                break;
            case 0xb5: // Up
                e.inputEvent = ModuleConfig_CannedMessageConfig_InputEventChar_UP;
                break;
            case 0xb6: // Down
                e.inputEvent = ModuleConfig_CannedMessageConfig_InputEventChar_DOWN;
                break;
            case 0xb4: // Left
                e.inputEvent = ModuleConfig_CannedMessageConfig_InputEventChar_LEFT;
                e.kbchar = c;
                break;
            case 0xb7: // Right
                e.inputEvent = ModuleConfig_CannedMessageConfig_InputEventChar_RIGHT;
                e.kbchar = c;
                break;
            case 0x0d: // Enter
                e.inputEvent = ModuleConfig_CannedMessageConfig_InputEventChar_SELECT;
                break;
            case 0x00: //nopress
                e.inputEvent = ModuleConfig_CannedMessageConfig_InputEventChar_NONE;
                break;
            default: // all other keys
                e.inputEvent = ANYKEY;
                e.kbchar = c;
                break;
            }

            if (e.inputEvent != ModuleConfig_CannedMessageConfig_InputEventChar_NONE) {
                this->notifyObservers(&e);
            }
        }
    }
    return 500;
}
