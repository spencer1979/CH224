/*
 * CH224.h
 *
 *  Created on: 2025年1月13日
 *      Author: 33547
 */

#ifndef CH224_H_
#define CH224_H_

#include <Wire.h>
#include <stdint.h> 
#define CH224_I2C_ADDRESS      0x23

// Protocol status register (0x09)
#define CH224_REG_STATUS      0x09

// Input value mapping: 0=5V, 1=9V, 2=12V, 3=15V, 4=20V, 5=28V, 6=PPS, 7=AVS
#define CH224_REG_VOL_CFG 0x0A

#define CH224_VOL_5V 0
#define CH224_VOL_9V 1
#define CH224_VOL_12V 2
#define CH224_VOL_15V 3
#define CH224_VOL_20V 4
#define CH224_VOL_28V 5
#define CH224_MODE_PPS 6 // PPS mode (only for CH224Q)
#define CH224_MODE_AVS 7 // AVS mode (only for CH224Q)

// Current status register (0x0B) Read-only
#define CH224_REG_CURRENT_STATUS  0x0B

// CH224 Current data (Maximum Current Step Register, read-only, unit: 50mA)
#define CH224_READ_CUR_STEP 50  // Read-only, unit: 50mA

// AVS voltage configuration register (0x51, 0x52) bit definition
#define CH224_REG_AVS_CFG_H    0x51  // AVS voltage configuration register high 8 bits
#define CH224_REG_AVS_CFG_L    0x52  // AVS voltage configuration register low 8 bits
#define CH224_READ_AVS_CFG_STEP 25  // Read-only, unit: 25mV

// PPS voltage configuration register (0x53) bit definition
#define CH224_REG_PPS_CFG      0x53  // PPS voltage configuration register (write-only, unit: 100mV)
#define CH224_READ_PPS_CFG_STEP 100   // PPS voltage step, unit: 100mV
#define CH224_PPS_CFG_MAX    2000  // PPS voltage configuration max value (20V, 2000mV)
#define CH224_PPS_CFG_MIN    500   // PPS voltage configuration min value (5V, 500mV)
#define CH224_PPS_CFG_STEP   100   // PPS voltage configuration step (100mV)


// CH224Q PD SRCCAP register address definition
#define CH224_REG_PD_SRCCAP_START 0x60  // PD SRCCAP data register start address
#define CH224_REG_PD_SRCCAP_END   0x8F  // PD SRCCAP data register end address
#define CH224_REG_PD_SRCCAP_LEN   (CH224_REG_PD_SRCCAP_END - CH224_REG_PD_SRCCAP_START + 1) // Data length

// Maximum number of Power Data Objects (PDOs) supported
#define MAX_PDO_COUNT 7

// CH224A/Q I2C status register (0x09)
/**
 * @brief I2C status register (0x09) bit definition.
 *
 * Bit 7: Reserved
 * Bit 6: AVS exist      (1 = AVS mode exists)
 * Bit 5: EPR exist      (1 = EPR mode exists, i.e., adapter max power > 100W)
 * Bit 4: EPR activation (1 = EPR handshake successful)
 * Bit 3: PD activation  (1 = PD handshake successful)
 * Bit 2: QC3 activation (1 = QC3 handshake successful)
 * Bit 1: QC2 activation (1 = QC2 handshake successful)
 * Bit 0: BC activation  (1 = BC handshake successful)
 *
 * All bits are read-only.
 */
typedef union {
    struct {
        uint8_t BC_ACT : 1;      // Bit 0: BC activation
        uint8_t QC2_ACT : 1;     // Bit 1: QC2 activation
        uint8_t QC3_ACT : 1;     // Bit 2: QC3 activation
        uint8_t PD_ACT : 1;      // Bit 3: PD activation
        uint8_t EPR_ACT : 1;     // Bit 4: EPR activation
        uint8_t EPR_EXIST : 1;    // Bit 5: EPR exist
        uint8_t AVR_EXIST : 1;    // Bit 6: AVS exist
        uint8_t REV : 1;         // Bit 7: Reserved
    } ;
    uint8_t Data;
} I2C_status_t;


typedef struct {
    I2C_status_t I2C_status;
    uint8_t vol_status;
    uint8_t current_status;
    uint8_t AVS_H;
    uint8_t AVS_L;
    uint8_t PPS_ctrl;
    uint8_t PD_data[48];
} CH224_t;

// Message Header
/**
 * @brief USB Power Delivery (PD) Message Header definition.
 *
 * bit15    bit14..bit12   bit11..bit9   bit8     bit7..bit6   bit5     bit4..bit0
 *   ↓           ↓             ↓          ↓          ↓          ↓           ↓
 *  Ext     NumDO(3)      MsgID(3)      PRRole     SpecRev(2)  PDRole   MsgType(5)
 *
 * - Ext (bit 15): Extended Message Indicator (1 bit)
 *      - 0: Not an extended message
 *      - 1: Extended message
 * - NumDO (bits 12-14): Number of Data Objects (3 bits)
 *      - Indicates the number of 32-bit Data Objects following the header (0-7).
 * - MsgID (bits 9-11): Message ID (3 bits)
 *      - Used for message identification and sequencing.
 * - PRRole (bit 8): Port Power Role (1 bit)
 *      - 0: Sink
 *      - 1: Source
 * - SpecRev (bits 6-7): Specification Revision (2 bits)
 *      - 00: USB PD Revision 1.0
 *      - 01: USB PD Revision 2.0
 *      - 10: USB PD Revision 3.0
 *      - 11: Reserved
 * - PDRole (bit 5): Power/Data Role (1 bit)
 *      - 0: UFP (Upstream Facing Port)
 *      - 1: DFP (Downstream Facing Port)
 * - MsgType (bits 0-4): Message Type (5 bits)
 *      - Specifies the type of message (e.g., Control, Data, Extended).
 *
 * The union allows access to the header as a whole (Data) or by individual fields (bits).
 */
typedef union {
    struct {
        uint16_t MsgType : 5;
        uint16_t PDRole : 1;
        uint16_t SpecRev : 2;
        uint16_t PRRole : 1;
        uint16_t MsgID : 3;
        uint16_t NumDO : 3;
        uint16_t Ext : 1;
    } ;
    uint16_t Data;
} PD_Message_Header_t;

// Extended Message Header
// USB PD Extended Message Header definition (see USB PD Spec 3.0 Table 6-29)
/**
 * @brief USB PD Extended Message Header
 *
 * 位元分佈（MSB→LSB）:
 *  15   14..11   10    9    8........0
 * +---+-------+----+----+-----------+
 * |Ch |Chunk# |Req |Rev | DataSize  |
 * +---+-------+----+----+-----------+
 *  |      |     |    |        |
 *  |      |     |    |        +-- [8:0]   資料長度 (9 bits)
 *  |      |     |    +----------- [9]     保留 (1 bit)
 *  |      |     +---------------- [10]    請求分塊 (1 bit)
 *  |      +---------------------- [14:11] 分塊編號 (4 bits)
 *  +----------------------------- [15]    分塊訊息 (1 bit)
 */
typedef union {
    struct {
        uint16_t DataSize : 9;
        uint16_t Rev : 1;
        uint16_t RequestChunk : 1;
        uint16_t ChunkNumber : 4;
        uint16_t Chunked : 1;
    } ;
    uint16_t Data;
} PD_Message_ExtHeader_t;

// Fixed Supply PDO - Source
/**
 * @brief USB Power Delivery (PD) Source Power Data Object (PDO) - Fixed Supply
 *
 * 位元分佈（MSB→LSB）:
 *  31  30 29 28 27 26 25 24 23 22 21 20 19........10 9.........0
 * +---+--+--+--+--+--+--+--+--+--+--+--+------------+----------+
 * |FS |DR|SU|UC|UC|DC|UC|EM| R|PC|      Voltage     |MaxCurrent|
 * +---+--+--+--+--+--+--+--+--+--+--+--+------------+----------+
 *  |   |  |  |  |  |  |  |  |  |  |  |         |           |
 *  |   |  |  |  |  |  |  |  |  |  |  |         |           +-- [9:0]    最大輸出電流 (10 bits, 單位 10mA)
 *  |   |  |  |  |  |  |  |  |  |  |  |         +-------------- [19:10]  輸出電壓 (10 bits, 單位 50mV)
 *  |   |  |  |  |  |  |  |  |  |  |  +------------------------ [21:20]  峰值電流能力 (2 bits)
 *  |   |  |  |  |  |  |  |  |  |  +--------------------------- [22]     保留 (1 bit)
 *  |   |  |  |  |  |  |  |  |  +------------------------------ [23]     是否支援 EPR 模式 (1 bit)
 *  |   |  |  |  |  |  |  |  +--------------------------------- [24]     是否支援 Unchunked Extended Message (1 bit)
 *  |   |  |  |  |  |  |  +------------------------------------ [25]     是否支援雙角色資料 (1 bit)
 *  |   |  |  |  |  |  +--------------------------------------- [26]     是否支援 USB 通訊 (1 bit)
 *  |   |  |  |  |  +------------------------------------------ [27]     是否為不受限電源 (1 bit)
 *  |   |  |  |  +--------------------------------------------- [28]     是否支援 USB Suspend (1 bit)
 *  |   |  |  +------------------------------------------------ [29]     是否支援雙角色電源 (1 bit)
 *  |   +------------------------------------------------------ [31:30]  固定電源類型 (2 bits)
 */
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
    };
    uint32_t Data;
} FixedSupply;

//SPR Programmable Power Supply APDO - Source
/**
 * @brief Represents a USB Power Delivery (USB PD) Programmable Power Supply (PPS) mode data structure.
 *
 * 位元分佈（MSB→LSB）:
 *  31 30 29 28 27 26 25 24 23........17 16........9 8.......1 0
 * +--+--+--+--+--+--+--+--+-------------+----------+--------+-+
 * |AP|AP|PP|PP|PL|R3|R3|MV|   MaxVolt   |MinVolt   |R1|MaxCur|
 * +--+--+--+--+--+--+--+--+-------------+----------+--------+-+
 *  |  |  |  |  |  |  |  |        |          |     |      |
 *  |  |  |  |  |  |  |  |        |          |     +------ [6:0]   最大電流 (MaxCurrent, 7 bits, 50mA units)
 *  |  |  |  |  |  |  |  |        |          +------------ [8:7]   保留 (Reserved1, 1 bit)
 *  |  |  |  |  |  |  |  |        +------------------------ [16:9] 最小電壓 (MinVoltage, 8 bits, 20mV units)
 *  |  |  |  |  |  |  |  +------------------------------- [17]    保留 (Reserved2, 1 bit)
 *  |  |  |  |  |  |  +---------------------------------- [25:18] 最大電壓 (MaxVoltage, 8 bits, 100mV units)
 *  |  |  |  |  |  +------------------------------------- [27:26] 保留 (Reserved3, 2 bits)
 *  |  |  |  |  +---------------------------------------- [28]    PPS 電源受限 (PPSPWRLimited, 1 bit)
 *  |  |  +---------------------------------------------- [30:29] PPS 模式 (PPS, 2 bits)
 *  +---------------------------------------------------- [32:31] APDO 類型 (APDO, 2 bits)
 *
 * Fields:
 * - MaxCurrent (7 bits): Maximum current in 50mA units that the PPS source can supply.
 * - Reserved1 (1 bit): Reserved, must be set to zero.
 * - MinVoltage (8 bits): Minimum voltage in 20mV units that the PPS source can supply.
 * - Reserved2 (1 bit): Reserved, must be set to zero.
 * - MaxVoltage (8 bits): Maximum voltage in 100mV units that the PPS source can supply.
 * - Reserved3 (2 bits): Reserved, must be set to zero.
 * - PPSPWRLimited (1 bit): Indicates if the PPS power is limited (1 = limited, 0 = not limited).
 * - PPS (2 bits): PPS mode indicator.
 * - APDO (2 bits): Augmented Power Data Object (APDO) type.
 *
 * @note This structure is specific to USB PD PPS mode and is used for negotiating programmable power supply parameters.
 */
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
    } ;
    uint32_t Data;
} PPSupply;

//EPR Adjustable Voltage Supply APDO - Source
/**
 * @brief USB Power Delivery (USB PD) 可調電壓供應 (EPR AVS) APDO 定義
 *
 * 位元分佈（MSB→LSB）:
 *  31........24 23........16 15........7 6.......4 3.......2 1.......0
 * +------------+------------+------------+--------+--------+--------+
 * |    PDP     | MinVoltage | MaxVoltage | PeakC  | EPRAVS |  APDO  |
 * +------------+------------+------------+--------+--------+--------+
 *  |                |             |         |         |         |
 *  |                |             |         |         |         +-- [1:0]   APDO 類型 (2 bits)
 *  |                |             |         |         +------------ [3:2]   EPR AVS 模式 (2 bits)
 *  |                |             |         +----------------------- [5:4]   峰值電流能力 (2 bits)
 *  |                |             +--------------------------------- [14:6]  最大電壓 (9 bits, 單位 100mV)
 *  |                +----------------------------------------------- [23:15] 最小電壓 (9 bits, 單位 100mV)
 *  +--------------------------------------------------------------- [31:24] PDP (8 bits, 單位 1W)
 *
 * 字段說明:
 * - PDP (8 bits): 可提供的最大功率，單位為 1W。
 * - MinVoltage (9 bits): 可提供的最小電壓，單位為 100mV。
 * - MaxVoltage (9 bits): 可提供的最大電壓，單位為 100mV。
 * - PeakCurrent (2 bits): 峰值電流能力。
 * - EPRAVS (2 bits): 可調電壓供應模式。
 * - APDO (2 bits): 擴展電源數據對象 (APDO) 類型。
 */
typedef union {
    struct {
        uint32_t PDP : 8;
        uint32_t MinVoltage : 8;
        uint32_t Reserved2 : 1;
        uint32_t MaxVoltage : 9;
        uint32_t PeakCurrent : 2;
        uint32_t EPRAVS : 2;
        uint32_t APDO : 2;
    } ;
    uint32_t Data;
} AVSupply;

//----------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------//

extern CH224_t CH224;

extern FixedSupply Rx_FixedSupply;
extern PPSupply Rx_PPSupply;
extern AVSupply Rx_AVSupply;

extern PD_Message_Header_t Rx_Header;
extern PD_Message_ExtHeader_t Rx_Ext_Header;
//PD_Msg[10][4] 代表最多 10 個 PDO，每個 PDO 佔用 4 個 uint16_t
//PD_Msg[0][0] 代表 PDO 類型，1 代表固定電壓，2 代表 PPS，3 代表 AVS，4 代表無效的 PDO
//PD_Msg[0][1] 代表 PDO 電壓(fixd mode),最小電壓(PPS ,AVS mode ) 
//PD_Msg[0][2] 代表 PDO 最大電壓(PPS ,AVS mode) ,最大電流(fixd mode)
//PD_Msg[0][3] 代表 PDO 最大功率(AVS mode) ,最大電流(PPS mode) ,0 (fixd mode)
extern uint16_t PD_Msg[10][4];

//----------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------//

void CH224_Init(uint8_t sda, uint8_t scl);
static void I2C_WriteByte(uint8_t addr, uint8_t reg, uint8_t data);
static uint8_t I2C_ReadByte(uint8_t addr, uint8_t reg);
static void I2C_SequentialRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer);
void CH224_ReadStatus(void);
void CH224_DataInit(void);
void CH224_AVS_Request(float vol);
void CH224_PPS_Request(float vol);
void CH224_Fixed_Request(uint16_t vol );
void CH224_SourceCap_Analyse();
bool CH224_HasValidFixedVoltage( uint16_t   req_vol);
#endif /* CH224_H_ */