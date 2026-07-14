#include <Arduino.h>

constexpr uint8_t RS485_DIR_PIN = 2;

constexpr uint8_t DRIVER_1 = 1;
constexpr uint8_t DRIVER_2 = 2;

constexpr uint16_t FIXED_RPM = 500;

HardwareSerial& rs485 = Serial1;


// ============================================================
// Modbus CRC
// ============================================================

uint16_t modbusCRC(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}


// ============================================================
// Send one Modbus register command
// ============================================================

bool writeRegisterOnce(
    uint8_t address,
    uint16_t registerAddress,
    uint16_t value)
{
    uint8_t frame[8];

    frame[0] = address;
    frame[1] = 0x06;
    frame[2] = highByte(registerAddress);
    frame[3] = lowByte(registerAddress);
    frame[4] = highByte(value);
    frame[5] = lowByte(value);

    uint16_t crc = modbusCRC(frame, 6);

    frame[6] = lowByte(crc);
    frame[7] = highByte(crc);

    delay(10);

    while (rs485.available())
    {
        rs485.read();
    }

    // MAX485 transmit mode
    digitalWrite(RS485_DIR_PIN, HIGH);
    delayMicroseconds(100);

    rs485.write(frame, sizeof(frame));
    rs485.flush();

    // MAX485 receive mode
    digitalWrite(RS485_DIR_PIN, LOW);

    uint8_t response[8];
    size_t received = 0;
    uint32_t startTime = millis();

    while (millis() - startTime < 400)
    {
        while (rs485.available() && received < sizeof(response))
        {
            response[received++] = rs485.read();
        }

        if (received == sizeof(response))
        {
            break;
        }
    }

    if (received != sizeof(response))
    {
        return false;
    }

    uint16_t receivedCRC =
        static_cast<uint16_t>(response[6]) |
        (static_cast<uint16_t>(response[7]) << 8);

    if (receivedCRC != modbusCRC(response, 6))
    {
        return false;
    }

    // Function 0x06 should echo the request
    for (uint8_t i = 0; i < 6; i++)
    {
        if (response[i] != frame[i])
        {
            return false;
        }
    }

    return true;
}


bool writeRegister(
    uint8_t address,
    uint16_t registerAddress,
    uint16_t value)
{
    constexpr uint8_t MAX_ATTEMPTS = 3;

    for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)
    {
        if (writeRegisterOnce(address, registerAddress, value))
        {
            Serial.print("Driver ");
            Serial.print(address);
            Serial.println(": OK");

            return true;
        }

        Serial.print("Driver ");
        Serial.print(address);
        Serial.print(": retry ");
        Serial.println(attempt);

        delay(30);
    }

    Serial.print("Driver ");
    Serial.print(address);
    Serial.println(": no response");

    return false;
}


// ============================================================
// Motor commands
// ============================================================

void setSpeed(uint8_t address, uint16_t rpm)
{
    // Register 0x0056 = speed command
    writeRegister(address, 0x0056, rpm);
}


void startForward(uint8_t address)
{
    // Register 0x0066:
    // 1 = forward
    writeRegister(address, 0x0066, 1);
}


void stopMotor(uint8_t address)
{
    // Register 0x0066:
    // 0 = natural stop
    writeRegister(address, 0x0066, 0);
}


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    pinMode(RS485_DIR_PIN, OUTPUT);
    digitalWrite(RS485_DIR_PIN, LOW);

    // Teensy Serial1:
    // TX1 = pin 1
    // RX1 = pin 0
    rs485.begin(9600, SERIAL_8N1);

    delay(2000);

    Serial.println("Initializing drivers");

    // Put both drivers into RS485/internal mode
    writeRegister(DRIVER_1, 0x0136, 1);
    delay(30);

    writeRegister(DRIVER_2, 0x0136, 1);
    delay(30);

    // Stop both before starting
    stopMotor(DRIVER_1);
    delay(30);

    stopMotor(DRIVER_2);
    delay(30);

    // Set fixed speed once
    setSpeed(DRIVER_1, FIXED_RPM);
    delay(30);

    setSpeed(DRIVER_2, FIXED_RPM);
    delay(30);

    Serial.println("Starting Driver 1");
    startForward(DRIVER_1);

    delay(3000);

    Serial.println("Starting Driver 2");
    startForward(DRIVER_2);

    // Both run at the same fixed speed for 5 seconds
    delay(5000);

    Serial.println("Stopping both");

    stopMotor(DRIVER_1);
    delay(30);

    stopMotor(DRIVER_2);
}


// ============================================================
// Loop
// ============================================================

void loop()
{
}