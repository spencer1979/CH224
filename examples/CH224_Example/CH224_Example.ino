#include <Arduino.h>
#include "CH224.h" // 假設你的 library 名稱為 CH224.h
#include <Wire.h>
#include <Arduino.h>
uint8_t voltageLevels[] = {5, 9, 15}; // 支援的電壓級別
uint8_t currentIndex = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Serial.println("CH224 Demo Start");

    // 初始化 CH224，假設 SDA=2, SCL=3
    CH224_init(2, 3);
    Serial.println("CH224 Initialized");
}

void loop() {
    uint8_t voltage = voltageLevels[currentIndex];
    Serial.printf("Requesting %dV output...\n", voltage);

    // 設定電壓，假設函式名稱為 CH224_SetVoltage
    if ( CH224_Fixed_Request(uint8_t vol) ) {
        Serial.println("Voltage set successfully.");
    } else {
        Serial.println("Failed to set voltage.");
    }

    currentIndex = (currentIndex + 1) % (sizeof(voltageLevels) / sizeof(voltageLevels[0]));
    delay(2000);
}
