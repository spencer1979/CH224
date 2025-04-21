# CH224 Arduino Library

A library for interfacing with the CH224 PD controller via I2C on Arduino-compatible boards.

## Features
- Read and write CH224 registers via I2C.
- Parse PDO (Power Data Object) data.
- Request specific voltage levels (5V, 9V, 12V, 15V, 20V, 28V, PPS, AVS).
- Monitor hardware parameters (current, AVS voltage, PPS voltage).
- Check support for EPR, QC 3.0, QC 2.0, and BC modes.

## Register Descriptions
### Key Registers
- **0x09: Status Register**
  - Provides information about supported USB-PD features, such as BC, QC2.0, QC3.0, PD, EPR, and AVS.
  - Example:
    - `BIT0`: BC Mode Active
    - `BIT1`: QC2 Mode Active
    - `BIT2`: QC3 Mode Active
    - `BIT3`: PD Mode Active
    - `BIT4`: EPR Mode Active
    - `BIT5`: EPR Available
    - `BIT6`: AVS Available

- **0x0A: Voltage Control Register**
  - Used to request specific voltage levels:
    - `0x00`: 5V
    - `0x01`: 9V
    - `0x02`: 12V
    - `0x03`: 15V
    - `0x04`: 20V
    - `0x05`: 28V
    - `0x06`: PPS Mode
    - `0x07`: AVS Mode

- **0x50: Current Data Register**
  - Indicates the maximum current value for the current PD profile.
  - Unit: 50mA.

- **0x51, 0x52: AVS Voltage Configuration Registers**
  - Configure the AVS requested voltage.
  - Unit: 25mV.
  - Write the high 8 bits to `0x51` and the low 8 bits to `0x52`.

- **0x53: PPS Voltage Configuration Register**
  - Configure the PPS requested voltage.
  - Unit: 100mV.

- **0x60~0x8F: PD Power Data Registers**
  - Read-only registers for retrieving USB-PD Source Capabilities (SRCCAP) data.
  - In EPR mode, these registers provide complete EPR_SRCCAP data.

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/spencer1979/CH224_arduino.git
   ```
2. **Add to Arduino IDE**:
   - Copy the `CH224_Arduino_library` folder to your Arduino libraries directory (e.g., `Documents/Arduino/libraries`).

## Hardware Connection
- Connect the CH224 module to your Arduino board using the I2C interface:
  - **SDA**: Connect to Arduino's SDA pin (e.g., A4 on Arduino Uno).
  - **SCL**: Connect to Arduino's SCL pin (e.g., A5 on Arduino Uno).
  - **VCC**: Connect to 3.3V or 5V power supply.
  - **GND**: Connect to ground.

## Usage Example
Here is a simple example to get started with the CH224 library:

```cpp
#include <Wire.h>
#include <CH224.h>

CH224 ch224; // Create CH224 object

void setup() {
    Serial.begin(9600); // Initialize serial communication
    ch224.begin(A4, A5); // Initialize I2C with SDA and SCL pins
    Serial.println("CH224 PD Controller Test");
}

void loop() {
    // Check supported modes
    Serial.print("PD Supported: ");
    Serial.println(ch224.isPDSupported() ? "Yes" : "No");
    Serial.print("EPR Supported: ");
    Serial.println(ch224.isEPRSupported() ? "Yes" : "No");

    // Request 20V
    Serial.println("Requesting 20V...");
    ch224.requestVoltage(4); // 4 corresponds to 20V mode
    delay(1000);

    // Read current
    Serial.print("Current: ");
    Serial.print(ch224.getCurrent());
    Serial.println(" mA");

    delay(5000); // Wait 5 seconds before repeating
}
```

## Debugging
To enable debugging, uncomment the `#define CH224_DEBUG 1` line in the `CH224.h` file. This will output detailed register read/write operations and other debug information to the serial monitor.

## License
This library is licensed under the MIT License. See `LICENSE` for details.