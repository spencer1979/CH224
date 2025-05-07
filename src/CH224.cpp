/**
 * @file              CH224.c
 * @brief             CH224 test program with dynamic I2C pin configuration.
 * @author            WCH-ZJH
 * @modifier          Spencer Chen
 * @version           V1.0.0
 * @date              2025/01/02
 */

#include "CH224.h"
#include <Arduino.h>

// Definition moved to CH224.h

#define DEBUG_PRINT
#ifdef DEBUG_PRINT
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif

CH224_t CH224;

FixedSupply Rx_FixedSupply;
PPSupply Rx_PPSupply;
AVSupply Rx_AVSupply;

PD_Message_Header_t Rx_Header;
PD_Message_ExtHeader_t Rx_Ext_Header;
uint16_t PD_Msg[10][4];

/**
 * @brief Initializes the CH224 with specified I2C pins.
 * 
 * @param sda The SDA pin for I2C communication.
 * @param scl The SCL pin for I2C communication.
 */
void CH224_Init(uint8_t sda, uint8_t scl) {
    Wire.begin(sda, scl); // ESP32/ESP8266 supports specifying SDA/SCL pins

    DEBUG_PRINTF("I2C Initialized with SDA: ");
    DEBUG_PRINTF("%d, SCL: %d\n", sda, scl);
}

/**
 * @brief Resets the CH224 data structure to its default state.
 */
void CH224_DataInit() {
    CH224.AVS_H = 0;
    CH224.AVS_L = 0;
    for (uint8_t i = 0; i < 48; i++) {
        CH224.PD_data[i] = 0;
    }
    CH224.PPS_ctrl = 0;
    CH224.current_status = 0;
    CH224.I2C_status.Data = 0;
    CH224.vol_status = 0;
}

/**
 * @brief Writes a single byte to a specific register of an I2C device.
 * 
 * @param addr The 7-bit I2C address of the target device.
 * @param reg The register address to write to.
 * @param data The data byte to write.
 */
void I2C_WriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

/**
 * @brief Reads a single byte from a specific register of an I2C device.
 * 
 * @param addr The 7-bit I2C address of the target device.
 * @param reg The register address to read from.
 * @return uint8_t The byte read from the register, or 0xFF if an error occurs.
 */
uint8_t I2C_ReadByte(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF; // Return error value
}

/**
 * @brief Reads multiple bytes from consecutive registers of an I2C device.
 * 
 * @param addr The 7-bit I2C address of the target device.
 * @param reg The starting register address to read from.
 * @param len The number of bytes to read.
 * @param buffer Pointer to the buffer where the read bytes will be stored.
 */
void I2C_SequentialRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, len);
    for (uint8_t i = 0; i < len; i++) {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        }
    }
}

/**
 * @brief Reads the status registers of the CH224 and prints debug information.
 */
void CH224_ReadStatus() {
   
    CH224_DataInit(); // Initialize data structure
    //
    // Read I2C status register
    CH224.I2C_status.Data = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_STATUS);
    //force to set output 5V
    if( CH224.I2C_status.PD_ACT==1)
    {
        I2C_WriteByte( CH224_I2C_ADDRESS,CH224_REG_VOL_CFG,  CH224_VOL_5V );
        delay(2);
        DEBUG_PRINTF("Force switch to 5V 0x0A->%02x\n", I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG));
    }
    DEBUG_PRINTLN("\n/-----------------------------------/");
    DEBUG_PRINTLN("            CH224 Status           ");
    DEBUG_PRINTLN("/-----------------------------------/");
    delay(2);
    DEBUG_PRINTF("\nPWR_status(0:disable 1:enable):\n[ BC -> %d | QC2 -> %d | QC3 -> %d | PD -> %d | EPR -> %d ]\n\n",
                 CH224.I2C_status.BC_ACT, CH224.I2C_status.QC2_ACT, CH224.I2C_status.QC3_ACT, CH224.I2C_status.PD_ACT, CH224.I2C_status.EPR_EXIST);
    DEBUG_PRINTF("CH224.I2C_status:      0x09->%02x\n", CH224.I2C_status.Data);

    // Read voltage status register
    CH224.vol_status = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    delay(2);
    DEBUG_PRINTF("CH224.vol_status:      0x0A->%02x\n", CH224.vol_status);

    // Read current status register
    CH224.current_status = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_CURRENT_STATUS);
    delay(2);
    DEBUG_PRINTF("CH224.current_status:  0x50->%02x\n", CH224.current_status);

    // Read AVS high register
    CH224.AVS_H = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_H);
    delay(2);
    DEBUG_PRINTF("CH224.AVS_H:           0x51->%02x\n", CH224.AVS_H);

    // Read AVS low register
    CH224.AVS_L = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_L);
    delay(2);
    DEBUG_PRINTF("CH224.AVS_L:           0x52->%02x\n", CH224.AVS_L);

    // Read PPS control register
    CH224.PPS_ctrl = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG);
    delay(2);
    DEBUG_PRINTF("CH224.PPS_ctrl:        0x53->%02x\n", CH224.PPS_ctrl);

    // Read Source Capabilities data, using CH224.PD_data as the buffer
    I2C_SequentialRead(CH224_I2C_ADDRESS, CH224_REG_PD_SRCCAP_START, CH224_REG_PD_SRCCAP_LEN, CH224.PD_data);
    delay(2);
    if (CH224.I2C_status.PD_ACT == 1) { // If PD is active
        DEBUG_PRINTLN("\n/-----------------------------------/");
        DEBUG_PRINTLN("              Power Data            ");
        DEBUG_PRINTLN("/-----------------------------------/");
        for (uint8_t i = 0; i < 48; i++) {
            DEBUG_PRINTF("0x%02x->%02x\n", 0x60 + i, CH224.PD_data[i]);
        }

        CH224_SourceCap_Analyse(); // Analyze source capabilities

       
    }
}

/**
 * @brief Analyzes USB PD Source Capabilities and parses PDOs into Rx_PDOs.
 */
void CH224_SourceCap_Analyse() {
    memcpy( &Rx_Header.Data, &CH224.PD_data[0], 2 );
    if(Rx_Header.Ext==1)
    {
        memcpy( &Rx_Ext_Header.Data, &CH224.PD_data[2], 2 );
        for(uint8_t i=0;i<Rx_Ext_Header.DataSize/4;i++)
        {
            memcpy( &Rx_FixedSupply.Data, &CH224.PD_data[4*(i+1)], 4 );
            if(Rx_FixedSupply.FixedSupply==3)
            {
                memcpy( &Rx_PPSupply.Data, &Rx_FixedSupply.Data, 4 );
                if(Rx_PPSupply.PPS==1)
                {
                    memcpy( &Rx_AVSupply.Data, &Rx_PPSupply.Data, 4 );
                    PD_Msg[i][0]=3; // AVS
                    PD_Msg[i][1]=Rx_AVSupply.MinVoltage*100;
                    PD_Msg[i][2]=Rx_AVSupply.MaxVoltage*100;
                    PD_Msg[i][3]=Rx_AVSupply.PDP*1;
                    continue;
                }
                PD_Msg[i][0]=2;// PPS
                PD_Msg[i][1]=Rx_PPSupply.MinVoltage*100;
                PD_Msg[i][2]=Rx_PPSupply.MaxVoltage*100;
                PD_Msg[i][3]=Rx_PPSupply.MaxCurrent*50;
                continue;
            }
            PD_Msg[i][0]=1; // Fixed Supply
            PD_Msg[i][1]=Rx_FixedSupply.Voltage*50;
            PD_Msg[i][2]=Rx_FixedSupply.MaxCurrent*10;
            PD_Msg[i][3]=0;
            if( PD_Msg[i][1]==0&& PD_Msg[i][2]==0 &&PD_Msg[i][3]==0)
                      PD_Msg[i][0]=4; // 代表無效的 PDO
        }
    }
    else
    {
        for(uint8_t i=0;i<Rx_Header.NumDO;i++)
        {
            memcpy( &Rx_FixedSupply.Data, &CH224.PD_data[4*i+2], 4 );
            if(Rx_FixedSupply.FixedSupply==3)
            {
                memcpy( &Rx_PPSupply.Data, &Rx_FixedSupply.Data, 4 );
                PD_Msg[i][0]=2;// PPS
                PD_Msg[i][1]=Rx_PPSupply.MinVoltage*100;
                PD_Msg[i][2]=Rx_PPSupply.MaxVoltage*100;
                PD_Msg[i][3]=Rx_PPSupply.MaxCurrent*50;
                continue;
            }
            PD_Msg[i][0]=1;//   Fixed Supply
            PD_Msg[i][1]=Rx_FixedSupply.Voltage*50;
            PD_Msg[i][2]=Rx_FixedSupply.MaxCurrent*10;
            PD_Msg[i][3]=0;
        }
    }

    DEBUG_PRINTLN("\n/-----------------------------------/");
    DEBUG_PRINTLN("             Power Supply            ");
    DEBUG_PRINTLN("/-----------------------------------/");
    for (uint8_t i = 0; i < 10; i++) {
        switch (PD_Msg[i][0]) {
            case 0:
                break;
            case 1:
                DEBUG_PRINTF("[%02d] Fixed    %.2fV   %.2fA\n", i + 1, (float)PD_Msg[i][1] / 1000, (float)PD_Msg[i][2] / 1000);
                break;
            case 2:
                DEBUG_PRINTF("[%02d]  PPS     %.2fV - %.2fV   %.2fA\n", i + 1, (float)PD_Msg[i][1] / 1000, (float)PD_Msg[i][2] / 1000, (float)PD_Msg[i][3] / 1000);
                break;
            case 3:
                DEBUG_PRINTF("[%02d]  AVS     %.2fV - %.2fV   %-3dW\n", i + 1, (float)PD_Msg[i][1] / 1000, (float)PD_Msg[i][2] / 1000, PD_Msg[i][3]);
                break;
            case 4:
                DEBUG_PRINTF("[%02d]  ----------Rev----------\n", i + 1);
                break;
        }
    }

    // Print all PD_Msg[10][4] raw values
    DEBUG_PRINTLN("\n/-----------------------------------/");
    DEBUG_PRINTLN("           PD_Msg Raw Data           ");
    DEBUG_PRINTLN("/-----------------------------------/");
    for (uint8_t i = 0; i < 10; i++) {
        DEBUG_PRINTF("PD_Msg[%d]: ", i);
        for (uint8_t j = 0; j < 4; j++) {
            DEBUG_PRINTF("%5d ", PD_Msg[i][j]);
        }
        DEBUG_PRINTLN("");
    }
}
/**
 * @brief Checks if the fixed voltage mode is supported and matches the requested voltage.
 *
 * @param req_vol The requested voltage in millivolts (e.g., 5000 for 5V).
 * @return true if a matching fixed voltage is available, false otherwise.
 */
bool CH224_HasValidFixdVoltage(uint16_t req_vol)
{
    for (uint8_t i = 0; i < 10; i++) {
        // PD_Msg[i][0] == 1 means Fixed Supply
        if (PD_Msg[i][0] == 1) {
            // Check if voltage matches requested voltage (in mV)
            if (PD_Msg[i][1] == req_vol) {
                DEBUG_PRINTF("Found fixed voltage: %dmV at index %d\n", req_vol, i);
                return true;
            }
        }
    }
    DEBUG_PRINTF("Fixed voltage: %dmV not found\n", req_vol);
    return false;
}

/**
 * @brief Sends a request to set the fixed voltage mode with the specified voltage.
 *
 * @param vol The desired voltage in millivolts (e.g., 5000 for 5V).
 */
void CH224_Fixed_Request(uint16_t vol)
{
    if (!CH224_HasValidFixdVoltage(vol)) {
        DEBUG_PRINTF("Requested voltage %dmV not supported.\r\n", vol);
        return;
    }

    uint8_t read_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);

    switch (vol)
    {
        case 5000:
            DEBUG_PRINTF("Read before set (5V): 0x%02x\n", read_val);
            if (read_val != CH224_VOL_5V)
                delay(2);
            I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_VOL_5V);
            DEBUG_PRINTF("Req_ 5V... \r\n");
            break;
        case 9000:
            DEBUG_PRINTF("Read before set (9V): 0x%02x\n", read_val);
            if (read_val != CH224_VOL_9V)
                delay(2);
            I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_VOL_9V);
            DEBUG_PRINTF("Req_ 9V...  \r\n");
            break;
        case 12000:
            DEBUG_PRINTF("Read before set (12V): 0x%02x\n", read_val);
            if (read_val != CH224_VOL_12V)
                delay(2);
            I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_VOL_12V);
            DEBUG_PRINTF("Req_12V...  \r\n");
            break;
        case 15000:
            DEBUG_PRINTF("Read before set (15V): 0x%02x\n", read_val);
            if (read_val != CH224_VOL_15V)
                delay(2);
            I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_VOL_15V);
            DEBUG_PRINTF("Req_15V...  \r\n");
            break;
        case 20000:
            DEBUG_PRINTF("Read before set (20V): 0x%02x\n", read_val);
            if (read_val != CH224_VOL_20V)
                delay(2);
            I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_VOL_20V);
            DEBUG_PRINTF("Req_20V...  \r\n");
            break;
        case 28000 :
            DEBUG_PRINTF("Read before set (28V): 0x%02x\n", read_val);
            if (read_val != CH224_VOL_28V)
                delay(2);
            I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_VOL_28V);
            DEBUG_PRINTF("Req_28V...  \r\n");
            break;
        default:
            DEBUG_PRINTF("Req_error...  \r\n");
            break;
    }
    delay(2);
}


/**
 * @brief Sends a request to set the AVS (Adaptive Voltage Scaling) mode with the specified voltage.
 *
 * @param vol The desired voltage in volts (e.g., 5.0 for 5V).
 */
void CH224_AVS_Request(float vol)
{
    
}
