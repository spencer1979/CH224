/********************************************************************************
* File Name          : CH224Q.c
* Author             : WCH-ZJH
* Version            : V1.0.0
* Date               : 2025/01/02
* Description        : CH224Q test program with dynamic I2C pin configuration.
*********************************************************************************/

#include "CH224Q.h"
#include <Arduino.h>

_CH224Q_typedef CH224Q;

_FixedSupply Rx_FixedSupply;
_PPSupply Rx_PPSupply;
_AVSupply Rx_AVSupply;

_Message_Header Rx_Header;
_Message_ExtHeader Rx_Ext_Header;

uint16_t PD_Msg[10][4];

/**
 * @brief 初始化 CH224Q，允許指定 I2C 的 SDA 和 SCL 引腳
 * @param sda I2C 的 SDA 引腳
 * @param scl I2C 的 SCL 引腳
 */
void CH224Q_init(uint8_t sda, uint8_t scl) {

    Wire.begin(sda, scl); // ESP32/ESP8266 支持指定 SDA/SCL

    Serial.print("I2C Initialized with SDA: ");
    Serial.print(sda);
    Serial.print(", SCL: ");
    Serial.println(scl);
}

bool isI2CDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    return (error == 0); // 如果返回 0，表示設備存在
}

void I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data) {
    uint8_t result = Wire.beginTransmission(addr);
    if (result != 0) { // 檢查傳輸是否成功
        Serial.printf("I2C Write Error: Unable to communicate with device at 0x%02X\n", addr);
        return;
    }
    Wire.write(reg);
    result = Wire.endTransmission();
    if (result != 0) { // 檢查結束傳輸是否成功
        Serial.printf("I2C Write Error: Transmission failed for device at 0x%02X\n", addr);
    }
}

uint8_t I2C_Read_Byte(uint8_t addr, uint8_t reg) {
    uint8_t result = Wire.beginTransmission(addr);
    if (result != 0) { // 檢查傳輸是否成功
        Serial.printf("I2C Read Error: Unable to communicate with device at 0x%02X\n", addr);
        return 0; // 返回預設值
    }
    Wire.write(reg);
    result = Wire.endTransmission(false);
    if (result != 0) { // 檢查結束傳輸是否成功
        Serial.printf("I2C Read Error: Transmission failed for device at 0x%02X\n", addr);
        return 0; // 返回預設值
    }
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    } else {
        Serial.printf("I2C Read Error: No data available from device at 0x%02X\n", addr);
        return 0; // 返回預設值
    }
}

void I2C_SequentialRead(uint8_t addr, uint8_t reg, uint8_t len) {
    uint8_t result = Wire.beginTransmission(addr);
    if (result != 0) { // 檢查傳輸是否成功
        Serial.printf("I2C Sequential Read Error: Unable to communicate with device at 0x%02X\n", addr);
        return;
    }
    Wire.write(reg);
    result = Wire.endTransmission(false);
    if (result != 0) { // 檢查結束傳輸是否成功
        Serial.printf("I2C Sequential Read Error: Transmission failed for device at 0x%02X\n", addr);
        return;
    }
    Wire.requestFrom(addr, len);
    for (uint8_t i = 0; i < len; i++) {
        if (Wire.available()) {
            CH224Q.PD_data[i] = Wire.read();
        } else {
            Serial.printf("I2C Sequential Read Error: No data available at index %d from device at 0x%02X\n", i, addr);
            CH224Q.PD_data[i] = 0; // 填充預設值
        }
    }
}

void Get_CH224Qstatus() {
    Serial.println("\n/-----------------------------------/");
    Serial.println("            CH224Q Status           ");
    Serial.println("/-----------------------------------/");

    // 讀取 I2C 狀態寄存器
    CH224Q.I2C_status.Data = I2C_Read_Byte(0x23, 0x09);
    delay(2);
    Serial.printf("\nPWR_status(0:disable 1:enable):\n[ BC -> %d | QC2 -> %d | QC3 -> %d | PD -> %d | EPR -> %d ]\n\n",
                  CH224Q.I2C_status.BC, CH224Q.I2C_status.QC2, CH224Q.I2C_status.QC3, CH224Q.I2C_status.PD, CH224Q.I2C_status.EPR);
    Serial.printf("CH224Q.I2C_status:      0x09->%02x\n", CH224Q.I2C_status.Data);

    // 讀取電壓狀態寄存器
    CH224Q.vol_status = I2C_Read_Byte(0x23, 0x0A);
    delay(2);
    Serial.printf("CH224Q.vol_status:      0x0A->%02x\n", CH224Q.vol_status);

    // 讀取電流狀態寄存器
    CH224Q.current_status = I2C_Read_Byte(0x23, 0x50);
    delay(2);
    Serial.printf("CH224Q.current_status:  0x50->%02x\n", CH224Q.current_status);

    // 讀取 AVS 高位寄存器
    CH224Q.AVS_H = I2C_Read_Byte(0x23, 0x51);
    delay(2);
    Serial.printf("CH224Q.AVS_H:           0x51->%02x\n", CH224Q.AVS_H);

    // 讀取 AVS 低位寄存器
    CH224Q.AVS_L = I2C_Read_Byte(0x23, 0x52);
    delay(2);
    Serial.printf("CH224Q.AVS_L:           0x52->%02x\n", CH224Q.AVS_L);

    // 讀取 PPS 控制寄存器
    CH224Q.PPS_ctrl = I2C_Read_Byte(0x23, 0x53);
    delay(2);
    Serial.printf("CH224Q.PPS_ctrl:        0x53->%02x\n", CH224Q.PPS_ctrl);

    // 讀取 Source Capabilities 數據
    I2C_SequentialRead(0x23, 0x60, 48);

    if (CH224Q.I2C_status.PD == 1) { // 如果 PD 有效
        Serial.println("\n/-----------------------------------/");
        Serial.println("              Power Data            ");
        Serial.println("/-----------------------------------/");
        for (uint8_t i = 0; i < 48; i++) {
            Serial.printf("0x%02x->%02x\n", 0x60 + i, CH224Q.PD_data[i]);
        }

        // 分析 Source Capabilities
        SourceCap_Analyse();

        Serial.println("\n/-----------------------------------/");
        Serial.println("             Power Supply            ");
        Serial.println("/-----------------------------------/");
        for (uint8_t i = 0; i < 10; i++) {
            switch (PD_Msg[i][0]) {
                case 0:
                    break;
                case 1:
                    Serial.printf("[%02d] Fixed    %.2fV   %.2fA\n", i + 1, (float)PD_Msg[i][1] / 1000, (float)PD_Msg[i][2] / 1000);
                    break;
                case 2:
                    Serial.printf("[%02d]  PPS     %.2fV - %.2fV   %.2fA\n", i + 1, (float)PD_Msg[i][1] / 1000, (float)PD_Msg[i][2] / 1000, (float)PD_Msg[i][3] / 1000);
                    break;
                case 3:
                    Serial.printf("[%02d]  AVS     %.2fV - %.2fV   %-3dW\n", i + 1, (float)PD_Msg[i][1] / 1000, (float)PD_Msg[i][2] / 1000, PD_Msg[i][3]);
                    break;
                case 4:
                    Serial.printf("[%02d]  ----------Rev----------\n", i + 1);
                    break;
            }
        }
    }
}

void CH224Q_Data_Init() {
    CH224Q.AVS_H = 0;
    CH224Q.AVS_L = 0;
    for (uint8_t i = 0; i < 48; i++) {
        CH224Q.PD_data[i] = 0;
    }
    CH224Q.PPS_ctrl = 0;
    CH224Q.current_status = 0;
    CH224Q.I2C_status.Data = 0;
    CH224Q.vol_status = 0;
}

void AVS_req(float vol) {
    uint16_t data = (uint16_t)(vol * 1000.0) / 100;
    uint8_t AVS_DataH = (uint8_t)(((data & 0xFF00) >> 8) | 0x80);
    uint8_t AVS_DataL = (uint8_t)(data & 0x00FF);

    Serial.printf("dataH_write: %02x | dataL_write: %02x\n", AVS_DataH, AVS_DataL);

    I2C_Write_Byte(0x23, 0x52, AVS_DataL);
    delay(2);
    I2C_Write_Byte(0x23, 0x51, AVS_DataH);
    delay(2);

    if (I2C_Read_Byte(0x23, 0x0A) != 7) {
        delay(2);
        I2C_Write_Byte(0x23, 0x0A, 7);
        delay(2);
        if (I2C_Read_Byte(0x23, 0x0A) == 7) {
            Serial.println("Req_AVS");
        }
    }
}

void PPS_req(float vol) {
    uint8_t data = (uint8_t)(vol * 10.0);
    I2C_Write_Byte(0x23, 0x53, data);
    delay(2);
    if (I2C_Read_Byte(0x23, 0x0A) != 6) {
        delay(2);
        I2C_Write_Byte(0x23, 0x0A, 6);
    }
    Serial.printf("PPSdata_write: %02x     Req:  %.2f V\n", data, (float)(data / 10.0));
}

void Fixed_req(uint8_t vol) {
    uint8_t reg_value = 0;
    switch (vol) {
        case 5: reg_value = 0; break;
        case 9: reg_value = 1; break;
        case 12: reg_value = 2; break;
        case 15: reg_value = 3; break;
        case 20: reg_value = 4; break;
        case 28: reg_value = 5; break;
        default:
            Serial.println("Req_error...");
            return;
    }
    if (I2C_Read_Byte(0x23, 0x0A) != reg_value) {
        I2C_Write_Byte(0x23, 0x0A, reg_value);
    }
    Serial.printf("Req_%dV...\n", vol);
}

void SourceCap_Analyse() {
    memcpy(&Rx_Header.Data, &CH224Q.PD_data[0], 2);
    Serial.println("\n/-----------------------------------/");
    Serial.println("          Source Capabilities        ");
    Serial.println("/-----------------------------------/");

    if (Rx_Header.Message_Header.Ext == 1) {
        memcpy(&Rx_Ext_Header.Data, &CH224Q.PD_data[2], 2);
        for (uint8_t i = 0; i < Rx_Ext_Header.Message_Header.DataSize / 4; i++) {
            memcpy(&Rx_FixedSupply.Data, &CH224Q.PD_data[4 * (i + 1)], 4);
            if (Rx_FixedSupply.bit.FixedSupply == 3) {
                memcpy(&Rx_PPSupply.Data, &Rx_FixedSupply.Data, 4);
                if (Rx_PPSupply.bit.PPS == 1) {
                    memcpy(&Rx_AVSupply.Data, &Rx_PPSupply.Data, 4);
                    Serial.printf("PDO %d: EPR Adjustable Voltage Supply\n", i + 1);
                    Serial.printf("  Min Voltage: %d mV\n", Rx_AVSupply.bit.MinVoltage * 100);
                    Serial.printf("  Max Voltage: %d mV\n", Rx_AVSupply.bit.MaxVoltage * 100);
                    Serial.printf("  PDP: %d W\n", Rx_AVSupply.bit.PDP);
                    continue;
                }
                Serial.printf("PDO %d: Programmable Power Supply\n", i + 1);
                Serial.printf("  Min Voltage: %d mV\n", Rx_PPSupply.bit.MinVoltage * 100);
                Serial.printf("  Max Voltage: %d mV\n", Rx_PPSupply.bit.MaxVoltage * 100);
                Serial.printf("  Max Current: %d mA\n", Rx_PPSupply.bit.MaxCurrent * 50);
                continue;
            }
            Serial.printf("PDO %d: Fixed Supply\n", i + 1);
            Serial.printf("  Voltage: %d mV\n", Rx_FixedSupply.bit.Voltage * 50);
            Serial.printf("  Max Current: %d mA\n", Rx_FixedSupply.bit.MaxCurrent * 10);
        }
    } else {
        for (uint8_t i = 0; i < Rx_Header.Message_Header.NumDO; i++) {
            memcpy(&Rx_FixedSupply.Data, &CH224Q.PD_data[4 * i + 2], 4);
            if (Rx_FixedSupply.bit.FixedSupply == 3) {
                memcpy(&Rx_PPSupply.Data, &Rx_FixedSupply.Data, 4);
                Serial.printf("PDO %d: Programmable Power Supply\n", i + 1);
                Serial.printf("  Min Voltage: %d mV\n", Rx_PPSupply.bit.MinVoltage * 100);
                Serial.printf("  Max Voltage: %d mV\n", Rx_PPSupply.bit.MaxVoltage * 100);
                Serial.printf("  Max Current: %d mA\n", Rx_PPSupply.bit.MaxCurrent * 50);
                continue;
            }
            Serial.printf("PDO %d: Fixed Supply\n", i + 1);
            Serial.printf("  Voltage: %d mV\n", Rx_FixedSupply.bit.Voltage * 50);
            Serial.printf("  Max Current: %d mA\n", Rx_FixedSupply.bit.MaxCurrent * 10);
        }
    }
    Serial.println("/-----------------------------------/");
}
