#include <Arduino.h>
#include <Wire.h>
#include "CH224Q.h" // 引入 CH224Q.h

uint8_t voltageLevels[] = {5, 9, 15}; // 定義電壓級別
uint8_t currentIndex = 0;            // 當前電壓索引

void setup() {
    // 初始化內建 USB 串列，波特率為 115200
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // 等待串列初始化
    }
    Serial.println("ESP32-WROOM USB Serial Initialized");

    // 初始化 I2C，使用 GPIO 21 作為 SDA，GPIO 22 作為 SCL
    CH224Q_init(2, 3); // 初始化 CH224Q，指定 SDA 和 SCL 引腳
    Serial.println("CH224Q Initialized");

    // 初始化 CH224Q 數據結構
    CH224Q_Data_Init();
}

void loop() {
    // 設置電壓
    uint8_t voltage = voltageLevels[currentIndex];
    Serial.printf("Setting voltage to %dV...\n", voltage);
    Fixed_req(voltage); // 使用 Fixed_req 設置電壓

    // 切換到下一個電壓級別
    currentIndex = (currentIndex + 1) % 3; // 循環切換 5V -> 9V -> 15V

    // 延遲 2 秒
    delay(2000);
}