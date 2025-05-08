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
    CH224_DataInit(); // Initialize data structure
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
 * @brief Checks if the requested voltage is supported by the PD_Msg array.
 * 
 * @param req_mv The requested voltage in millivolts.5v=5000, 9v=9000, 12v=12000, 15v=15000, 20v=20000, 28v=28000
 * @param type The type of power supply (1=Fixed, 2=PPS, 3=AVS).
 * @return true if the requested voltage is supported, false otherwise.
 */
bool CH224_HasValidPdVoltage(uint8_t type,uint16_t req_mv)
{
    for (uint8_t i = 0; i < 10; i++) {
        // type: 1=Fixed, 2=PPS, 3=AVS
        if (PD_Msg[i][0] == PD_FIXED_SUPPLY) {
           
                if (PD_Msg[i][1] == req_mv) {
                    DEBUG_PRINTF("Found Fixed voltage: %dmV at index %d\n", req_mv, i);
                    return true;
                }   

            } 
        else if (PD_Msg[i][0]==PD_AVS_SUPPLY  || PD_Msg[i][0] == PD_PPS_SUPPLY) { // PPS or AVS
                
            if (PD_Msg[i][1] <= req_mv && req_mv <= PD_Msg[i][2]) {
                    DEBUG_PRINTF("Found %s voltage: %dmV at index %d\n", (type==2)?"PPS":"AVS", req_mv, i);
                    return true;
                }

            }
       
    }

    DEBUG_PRINTF("PD type %d voltage: %dmV not found\n", type, req_mv);
    return false;
}


/**
 * @brief Sends a request to set the fixed voltage mode with the specified voltage.
 *
 * @param vol The desired voltage in millivolts (e.g., 5000 for 5V).
 */
bool CH224_Fixed_Request(uint8_t vol)
{
    if (!CH224_HasValidPdVoltage(PD_FIXED_SUPPLY, vol * 1000)) {
        DEBUG_PRINTF("Requested voltage %dmV not supported.\r\n", vol * 1000);
        return false;
    }

    uint8_t read_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    uint8_t set_val = CH224_VOL_5V;
    bool valid = true;

    switch (vol)
    {
        case 5:
            set_val = CH224_VOL_5V;
            break;
        case 9:
            set_val = CH224_VOL_9V;
            break;
        case 12:
            set_val = CH224_VOL_12V;
            break;
        case 15:
            set_val = CH224_VOL_15V;
            break;
        case 20:
            set_val = CH224_VOL_20V;
            break;
        case 28:
            set_val = CH224_VOL_28V;
            break;
        default:
            valid = false;
            break;
    }

    if (!valid) {
        DEBUG_PRINTF("Req_error...  \r\n");
        return false;
    }

    DEBUG_PRINTF("Read before set (%dV): 0x%02x\n", vol, read_val);
    if (read_val != set_val) {
        I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, set_val);
        delay(2);
        DEBUG_PRINTF("Req_%dV...  \r\n", vol);
    }
    else {
        DEBUG_PRINTF("Already set to %dV...  \r\n", vol);
    }
    // 最後再做i2c讀 判斷是否是設定電壓
    uint8_t confirm_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    bool success = (confirm_val == set_val);
    DEBUG_PRINTF("Confirm set voltage: 0x%02x, %s\n", confirm_val, success ? "OK" : "FAIL");
    return success;
}


/**
 * 
 * @brief Sends a request to set the AVS (Adjustable Voltage Supply) mode with the specified voltage.
 *
 * @param vol The desired voltage in volts (e.g., 5.0 for 5V).
 * @return true if the request was successful, false otherwise.
 */
bool CH224_AVS_Request(float vol)
{
    // Check if the requested voltage is within the valid range for AVS
    if (CH224_HasAVS() == false)
    {
        DEBUG_PRINTLN("AVS not supported !");
        return false;
    }

    uint16_t req_mv = (uint16_t)(vol * 1000);
    if (!CH224_HasValidPdVoltage(PD_AVS_SUPPLY, req_mv))
    {
        DEBUG_PRINTF("AVS voltage %.2fV (=%dmV) not supported by PD_Msg[]\n", vol, req_mv);
        return false;
    }

    // Calculate AVS register value (AVS_H: high byte, AVS_L: low byte, value in 25mV steps)
    uint16_t avs_val = req_mv / 25;
    uint8_t avs_h = (avs_val >> 8) & 0xFF;
    uint8_t avs_l = avs_val & 0xFF;

    // Write AVS voltage registers
    I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_H, avs_h);
    delay(2);
    I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_L, avs_l);
    delay(2);

    // Switch to AVS mode
    I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_MODE_AVS);
    delay(2);

    // Read back AVS_H, AVS_L, and VOL_CFG for confirmation
    uint8_t read_avs_h = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_H);
    delay(2);
    uint8_t read_avs_l = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_L);
    delay(2);
    uint8_t read_mode = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);

    bool success = (read_avs_h == avs_h) && (read_avs_l == avs_l) && (read_mode == CH224_MODE_AVS);

    DEBUG_PRINTF("AVS request: %.2fV (AVS_H=0x%02x, AVS_L=0x%02x), readback H=0x%02x, L=0x%02x, MODE=0x%02x, %s\n",
        vol, avs_h, avs_l, read_avs_h, read_avs_l, read_mode, success ? "OK" : "FAIL");

    return success;
}

/**
 * @brief Sends a request to set the PPS (Programmable Power Supply) mode with the specified voltage.
 *
 * @param vol The desired voltage in volts (e.g., 5.0 for 5V).
 * @return true if the request was successful, false otherwise.
 */
bool CH224_PPS_Request(float vol)
{
    // Check if the requested voltage is within the valid range for PPS
    if (CH224_HasPPS() == false)
    {
        DEBUG_PRINTLN("PPS not supported !");
        return false;
    }

    uint16_t req_mv = (uint16_t)(vol * 1000);
    if (!CH224_HasValidPdVoltage(PD_PPS_SUPPLY, req_mv))
    {
        DEBUG_PRINTF("PPS voltage %.2fV (=%dmV) not supported by PD_Msg[]\n", vol, req_mv);
        return false;
    }

    // Read current VOL_CFG register to determine if already in PPS mode
    uint8_t vol_cfg_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    uint8_t set_val = (uint8_t)(vol * 10.0f);

    if (vol_cfg_val != CH224_MODE_PPS)
    {
        // First time applying for PPS: set PPS voltage first, then switch to PPS mode
        I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG, set_val);
        delay(2);
        I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_MODE_PPS);
        delay(2);
        DEBUG_PRINTF("First PPS request: Set PPS_CFG=0x%02x, then VOL_CFG=PPS mode\n", set_val);
    }
    else
    {
        // Already in PPS mode: just update PPS voltage
        I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG, set_val);
        delay(2);
        DEBUG_PRINTF("PPS mode already set: Update PPS_CFG=0x%02x\n", set_val);
    }

    // Read back and compare for confirmation
    uint8_t pps_cfg_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG);
    DEBUG_PRINTF("PPS_CFG reg readback: 0x%02x (%d)\n", pps_cfg_val, pps_cfg_val);

    bool success = (pps_cfg_val == set_val);
    DEBUG_PRINTF("PPS set %s\n", success ? "OK" : "FAIL");
    return success;
}
/**
 * @brief Gets the minimum or maximum voltage limit for PPS or AVS mode.
 *
 * @param type The type of power supply (1=Fixed, 2=PPS, 3=AVS).
 * @param get_max If true, get the maximum voltage; otherwise, get the minimum voltage.
 * @return uint16_t The voltage limit in millivolts.
 */
uint16_t CH224_GetPPSAVSLimitVoltage(uint8_t type, bool get_max)
{
    for (uint8_t i = 0; i < 10; i++)
    {
        if (PD_Msg[i][0] == type)
        {
            if (get_max) {
                DEBUG_PRINTF("Found %s max voltage: %dmV at index %d\n", (type == PD_PPS_SUPPLY) ? "PPS" : "AVS", PD_Msg[i][2], i);
                return PD_Msg[i][2];
            } else {
                DEBUG_PRINTF("Found %s min voltage: %dmV at index %d\n", (type == PD_PPS_SUPPLY) ? "PPS" : "AVS", PD_Msg[i][1], i);
                return PD_Msg[i][1];
            }
        }
    }
    return 0;
}
/**
 * @brief Gets the fixed voltage setting from the CH224 device.
 *
 * @return uint16_t The fixed voltage in millivolts.
 */
uint16_t CH224_GetFixedVoltage()
{   
    uint8_t fixed_vol = 99;
    uint16_t vol = 0;
    fixed_vol = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    switch (fixed_vol)
    {
        case CH224_VOL_5V:
            vol = 5000;
            break;
        case CH224_VOL_9V:
            vol = 9000;
            break;
        case CH224_VOL_12V:
            vol = 12000;
            break;
        case CH224_VOL_15V:
            vol = 15000;
            break;
        case CH224_VOL_20V:
            vol = 20000;
            break;
        case CH224_VOL_28V:
            vol = 28000;
            break;
        default:
            vol = 0; // Invalid voltage
            break;
    }
    DEBUG_PRINTF("Fixed voltage: %d mV\n", vol);
    return vol;
}

/**
 * @brief Gets the PPS voltage setting from the CH224 device.
 *
 * @return uint16_t The PPS voltage in millivolts.
 */
uint16_t CH224_GetPPSVoltage()
{
    uint8_t pps_vol = 0;
    uint16_t vol = 0;
    pps_vol = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG);
    vol = (uint16_t)(pps_vol * 100);
    DEBUG_PRINTF("PPS voltage: %d mV\n", vol);
    return vol;
}
/**
 * @brief Gets the AVS (Adaptive Voltage Scaling) voltage from the CH224 device.
 *
 * @return uint16_t The AVS voltage in millivolts.
 */
uint16_t CH224_GetAVSVoltage()
{
    uint16_t vol = 0;
    // Read AVS voltage from CH224 registers
    // AVS_H and AVS_L are combined to form the full voltage value
    // AVS_H is the high byte, and AVS_L is the low byte
    uint8_t avs_h = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_H);
    uint8_t avs_l = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_L);
    vol = (uint16_t)((  avs_h << 8) | avs_l);
    vol = (uint16_t)(vol * 25);
    DEBUG_PRINTF("AVS voltage: %d mV\n", vol);
    return vol;
}

bool CH224_HasPPS(void)
{
    for (uint8_t i = 0; i < 10; i++) {
        if (PD_Msg[i][0] == PD_PPS_SUPPLY) {
            DEBUG_PRINTLN("PPS supported");
            return true;
        }
    }
    DEBUG_PRINTLN("PPS not supported");
    return false;
}   

bool CH224_HasAVS(void)
{
    for (uint8_t i = 0; i < 10; i++) {
        if (PD_Msg[i][0] == PD_AVS_SUPPLY) {
            DEBUG_PRINTLN("AVS supported");
            return true;
        }
    }
    DEBUG_PRINTLN("AVS not supported");
    return false;
}

bool CH224_HasEPR(void)
{
    if (CH224.I2C_status.EPR_EXIST == 1) {
        DEBUG_PRINTLN("EPR supported");
        return true;
    }
    DEBUG_PRINTLN("EPR not supported");
    return false;
}



