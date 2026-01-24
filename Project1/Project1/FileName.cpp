#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>


const int DXL_DIR_PIN = 2;
SoftwareSerial bt(4, 5);
Dynamixel2Arduino dxl(Serial, DXL_DIR_PIN);

#define MOTOR_ID 1
#define ADDR_OPERATING_MODE 11
#define ADDR_GOAL_CURRENT 102
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_CURRENT 126

uint16_t targetCur = 111;
uint16_t holdCur = 111;
int32_t posOpen = 2800;
int32_t posClose = 700;


void safeOpen() {
    dxl.write(MOTOR_ID, ADDR_GOAL_CURRENT, (uint8_t*)&holdCur, 2);
    dxl.write(MOTOR_ID, ADDR_GOAL_POSITION, (uint8_t*)&posOpen, 4);

    unsigned long start = millis();
    while (millis() - start < 3000) {
        int32_t curPos = 0;

        dxl.read(MOTOR_ID, 132, 4, (uint8_t*)&curPos, 10);

        if (abs(curPos - posOpen) < 30) break;
        delay(50);
    }
}

void safeClose() {
    dxl.write(MOTOR_ID, ADDR_GOAL_CURRENT, (uint8_t*)&targetCur, 2);
    dxl.write(MOTOR_ID, ADDR_GOAL_POSITION, (uint8_t*)&posClose, 4);

    unsigned long startTime = millis();
    int stallCount = 0;

    while (millis() - startTime < 60000) { 
        if (bt.available() && bt.read() == '1') {
            safeOpen();
            return;
        }

        int32_t pos = 0, vel = 0;
        int16_t cur = 0;

        dxl.read(MOTOR_ID, 132, 4, (uint8_t*)&pos, 10);
        dxl.read(MOTOR_ID, 128, 4, (uint8_t*)&vel, 10);
        dxl.read(MOTOR_ID, ADDR_PRESENT_CURRENT, 2, (uint8_t*)&cur, 10);

        bt.print(F("P:")); bt.print(pos);
        bt.print(F("|R:")); bt.print(cur);
        bt.print(F("|V:")); bt.println(abs(vel));

        if (abs(cur) >= 800 || (abs(vel) < 3 && abs(cur) >= (targetCur - 10))) {
            if (++stallCount > 1) { 
                dxl.write(MOTOR_ID, ADDR_GOAL_CURRENT, (uint8_t*)&holdCur, 2);
                dxl.write(MOTOR_ID, ADDR_GOAL_POSITION, (uint8_t*)&pos, 4);
                return;
            }
        }
        else {
            stallCount = 0;
        }

        if (abs(pos - posClose) < 30) {
            safeOpen();
            return;
        }
        delay(20); 
    }
}


void setup() {
    bt.begin(9600);
    dxl.begin(57600);
    dxl.setPortProtocolVersion(2.0);

    if (dxl.ping(MOTOR_ID)) {
        dxl.torqueOff(MOTOR_ID);
        uint8_t m = 5;
        dxl.write(MOTOR_ID, ADDR_OPERATING_MODE, &m, 1); // 1BITE
        dxl.torqueOn(MOTOR_ID);

        safeOpen();
    }
}

void loop() {
    if (bt.available()) {
        char c = bt.read();
        if (c == '1') safeOpen();
        else if (c == '2') safeClose();
    }
}