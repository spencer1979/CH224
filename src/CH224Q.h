/*
 * CH224Q.h
 *
 *  Created on: 2025年1月13日
 *      Author: 33547
 */

#ifndef CH224Q_H_
#define CH224Q_H_

#include <Wire.h>
#include <stdint.h> // 使用標準整數類型

//----------------------------------------------------------------------------------------------//

// CH224A/Q PD_status_Reg
typedef union {
    struct {
        uint8_t BC : 1;
        uint8_t QC2 : 1;
        uint8_t QC3 : 1;
        uint8_t PD : 1;
        uint8_t EPR : 1;
        uint8_t Rev : 3;
    };
    uint8_t Data;
} _I2C_status_typedef;

typedef struct {
    _I2C_status_typedef I2C_status;
    uint8_t vol_status;
    uint8_t current_status;
    uint8_t AVS_H;
    uint8_t AVS_L;
    uint8_t PPS_ctrl;
    uint8_t PD_data[48];
} _CH224Q_typedef;

//Head
typedef union {
    struct {
        uint8_t MsgType : 5;
        uint8_t PDRole : 1;
        uint8_t SpecRev : 2;
        uint8_t PRRole : 1;
        uint8_t MsgID : 3;
        uint8_t NumDO : 3;
        uint8_t Ext : 1;
    } Message_Header;
    uint16_t Data;
} _Message_Header;

//Ext-Head
typedef union {
    struct {
        uint16_t DataSize : 9;
        uint16_t Rev : 1;
        uint16_t RequestChunk : 1;
        uint16_t ChunkNumber : 4;
        uint16_t Chunked : 1;
    } Message_Header;
    uint16_t Data;
} _Message_ExtHeader;

//Fixed Supply PDO - Source
typedef union {
    struct {
        uint32_t MaxCurrent : 10;
        uint32_t Voltage : 10;
        uint32_t PeakCurrent : 2;
        uint32_t Reserved : 1;
        uint32_t EPRModeCap : 1;
        uint32_t Unchunked : 1;
        uint32_t DualRoleData : 1;
        uint32_t USBComCap : 1;
        uint32_t UnconstrainedPWR : 1;
        uint32_t USBSuspendSupported : 1;
        uint32_t DualRolePWR : 1;
        uint32_t FixedSupply : 2;
    } bit;
    uint32_t Data;
} _FixedSupply;

//SPR Programmable Power Supply APDO - Source
typedef union {
    struct {
        uint32_t MaxCurrent : 7;
        uint32_t Reserved1 : 1;
        uint32_t MinVoltage : 8;
        uint32_t Reserved2 : 1;
        uint32_t MaxVoltage : 8;
        uint32_t Reserved3 : 2;
        uint32_t PPSPWRLimited : 1;
        uint32_t PPS : 2;
        uint32_t APDO : 2;
    } bit;
    uint32_t Data;
} _PPSupply;

//EPR Adjustable Voltage Supply APDO - Source
typedef union {
    struct {
        uint32_t PDP : 8;
        uint32_t MinVoltage : 8;
        uint32_t Reserved2 : 1;
        uint32_t MaxVoltage : 9;
        uint32_t PeakCurrent : 2;
        uint32_t EPRAVS : 2;
        uint32_t APDO : 2;
    } bit;
    uint32_t Data;
} _AVSupply;

//----------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------//

extern _CH224Q_typedef CH224Q;

extern _FixedSupply Rx_FixedSupply;
extern _PPSupply Rx_PPSupply;
extern _AVSupply Rx_AVSupply;

extern _Message_Header Rx_Header;
extern _Message_ExtHeader Rx_Ext_Header;

extern uint16_t PD_Msg[10][4];

//----------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------//

void CH224Q_init(uint8_t sda, uint8_t scl);
void I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t I2C_Read_Byte(uint8_t addr, uint8_t reg);
void I2C_SequentialRead(uint8_t addr, uint8_t reg, uint8_t len);
void Get_CH224Qstatus();
void CH224Q_Data_Init();
void AVS_req(float vol);
void PPS_req(float vol);
void Fixed_req(uint8_t vol);
void SourceCap_Analyse();

//----------------------------------------------------------------------------------------------//

#endif /* CH224Q_H_ */
