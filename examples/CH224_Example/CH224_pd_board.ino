#include <Arduino.h>
#include <Wire.h>
#include <CH224.h> // 引入 CH224Q.h

#define PD_EN_PIN 8       // PD_EN 引腳
#define USB_DETECT_PIN 9 // USB_DETECT 引腳
#define SDA_PIN 2         // I2C SDA 引腳
#define SCL_PIN 3         // I2C SCL 引腳
#define UP_BUTTON_PIN 21  // 升壓按鍵引腳
#define DOWN_BUTTON_PIN 20 // 降壓按鍵引腳

volatile bool usbDetectState = HIGH;    // Assume USB is not connected initially
volatile bool usbStateChanged = false; // Flag to indicate USB state change

int lastUsbDetectState = HIGH; // 上一次 USB_DETECT_PIN 的狀態
float currentVoltage = CH224_PPS_CFG_MIN ; // 初始電壓為 5V
float maxPPSVoltage =0; // PPS最大電壓為 
float minPPSVoltage =0; // PPS最小電壓為 
// 按鍵去抖動相關變數
unsigned long lastDebounceTimeUp = 0;
unsigned long lastDebounceTimeDown = 0;
const unsigned long debounceDelay = 50; // 去抖動延遲時間 (毫秒)

void IRAM_ATTR usbDetectISR() {
    usbDetectState = digitalRead(USB_DETECT_PIN); // Read the current state of USB_DETECT_PIN
    usbStateChanged = true;                      // Set the flag to indicate state change
}

void CH224PowerControl( bool powerOn) {
    
    if (powerOn==LOW) {
        digitalWrite(PD_EN_PIN, HIGH);
        // 初始化 CH224Q 數據結構並讀取狀態
        Serial.println("USB detected during setup. CH224Q power ON.");
        delay(500); // 等待 IC 開機完成
        CH224_DataInit(); // 初始化 CH224Q 數據結構
         CH224_Init(SDA_PIN, SCL_PIN);
        Serial.println("CH224Q Initialized");
        // USB 已插入，開啟 CH224Q 電源
        CH224_ReadStatus(); // 獲取 CH224Q 狀態
        Serial.println("CH224Q Read Status");
    } else {
       Wire.end(); // 結束 I2C 通信
        digitalWrite(PD_EN_PIN, LOW); // 關閉 CH224Q 電源
        CH224_DataInit(); // 初始化 CH224Q 數據結構
        Serial.println("CH224Q power OFF.");
    }

}


void setup() {
    // 初始化內建 USB 串列，波特率為 115200
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // 等待串列初始化
    }
    Serial.println("ESP32-C3 USB Serial Initialized");
   
    // 設定 PD_EN_PIN 和 USB_DETECT_PIN 的模式
    pinMode(PD_EN_PIN, OUTPUT);
    digitalWrite(PD_EN_PIN, LOW); // 初始狀態設為低電平（關閉 CH224Q 電源）
    pinMode(USB_DETECT_PIN, INPUT_PULLUP); // USB_DETECT_PIN 設為輸入模式，帶內部上拉

    // 設定按鍵引腳模式
    pinMode(UP_BUTTON_PIN, INPUT_PULLUP);   // UP 按鍵
    pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP); // DOWN 按鍵
    // 檢測 USB 是否已經插入
    int usbDetectState = digitalRead(USB_DETECT_PIN);
    CH224PowerControl(usbDetectState); // 控制 CH224Q 電源
    maxPPSVoltage = static_cast<float>(CH224_GetPPSAVSLimitVoltage(PD_PPS_SUPPLY, true)) / 1000.0f; // 獲取最大 PPS 電壓並轉為 float
    minPPSVoltage = static_cast<float>(CH224_GetPPSAVSLimitVoltage(PD_PPS_SUPPLY, false)) / 1000.0f; // 獲取最小 PPS 電壓並轉為 float  currentVoltage = minPPSVoltage; // 設定當前電壓為最小 PPS 電壓
    // Attach interrupt to USB_DETECT_PIN
     attachInterrupt(digitalPinToInterrupt(USB_DETECT_PIN), usbDetectISR, CHANGE);
    Serial.println("USB_DETECT_PIN interrupt attached");
}

void loop() {
    // Handle USB state change
    if (usbStateChanged) {
        usbStateChanged = false; // Reset the flag
        CH224PowerControl(usbDetectState); // 控制 CH224Q 電源
    }
   
   
    // 只有當 CH224Q 電源打開時才處理按鍵操作
    if (digitalRead(PD_EN_PIN) == HIGH) {
       
            static unsigned long lastRandomVoltageTime = 0;
            if (millis() - lastRandomVoltageTime >= 2000) {
                lastRandomVoltageTime = millis();
                // 產生 minPPSVoltage ~ maxPPSVoltage 之間的隨機電壓
                maxPPSVoltage = static_cast<float>(CH224_GetPPSAVSLimitVoltage(PD_PPS_SUPPLY, true)) / 1000.0f; // 獲取最大 PPS 電壓並轉為 float
                minPPSVoltage = static_cast<float>(CH224_GetPPSAVSLimitVoltage(PD_PPS_SUPPLY, false)) / 1000.0f; // 獲取最小 PPS 電壓並轉為 float  currentVoltage = minPPSVoltage; // 設定當前電壓為最小 PPS 電壓
                // Attach interrupt to USB_DETECT_PIN
                float randomVoltage = minPPSVoltage + static_cast<float>(rand()) / RAND_MAX * (maxPPSVoltage - minPPSVoltage);
                // 四捨五入到 0.1V (1位小數)
                randomVoltage = round(randomVoltage * 10.0f) / 10.0f;
                currentVoltage = randomVoltage;
                // 發送 PPS 請求（單位為伏特 V）
                CH224_PPS_Request(currentVoltage);
                Serial.print("Random PPS Voltage Requested: ");
                Serial.print(currentVoltage, 1);
                Serial.println(" V");
            }
        
           
        // 處理 UP 按鍵
        static int lastUpButtonState = HIGH;
        int upButtonState = digitalRead(UP_BUTTON_PIN);
        if (upButtonState == LOW && lastUpButtonState == HIGH && (millis() - lastDebounceTimeUp > debounceDelay)) {
            lastDebounceTimeUp = millis();
            // 讀取當前 PPS 電壓
            uint16_t ppsVoltageRaw = CH224_GetPPSVoltage();
            // 顯示 PPS 電壓
            Serial.print("Current PPS Voltage: ");
            Serial.print(ppsVoltageRaw, 1);
            Serial.println(" V");
        }
        lastUpButtonState = upButtonState;

        // 處理 DOWN 按鍵
        static int lastDownButtonState = HIGH;
        int downButtonState = digitalRead(DOWN_BUTTON_PIN);
        if (downButtonState == LOW && lastDownButtonState == HIGH && (millis() - lastDebounceTimeDown > debounceDelay)) {
            lastDebounceTimeDown = millis();

        
        }
        lastDownButtonState = downButtonState;
    }

    // 延遲 50 毫秒進行下一次檢測
    delay(50);
}