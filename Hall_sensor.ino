#include <Arduino.h>

// ============================================================
// Hardware configuration
// ============================================================

// Teensy 4.1:
// Pin 0 = RX1 from MAX485 RO through voltage divider
// Pin 1 = TX1 to MAX485 DI
// Pin 2 = MAX485 DE and /RE connected together
constexpr uint8_t RS485_DIR_PIN = 2;

HardwareSerial& rs485 = Serial1;

constexpr uint8_t DRIVER_1_ADDRESS = 1;
constexpr uint8_t DRIVER_2_ADDRESS = 2;

constexpr uint16_t FIXED_RPM = 4000;

// Four-pole motor = two pole pairs
constexpr uint16_t MOTOR_POLE_PAIRS = 2;

// ============================================================
// BLD-305S registers
// ============================================================

constexpr uint16_t REG_SPEED_COMMAND = 0x0056;
constexpr uint16_t REG_RUN_COMMAND   = 0x0066;
constexpr uint16_t REG_FAULT         = 0x0076;
constexpr uint16_t REG_ADDRESS       = 0x00A6;
constexpr uint16_t REG_POLE_PAIRS    = 0x0116;
constexpr uint16_t REG_CONTROL_MODE  = 0x0136;

constexpr uint16_t REG_ACTUAL_SPEED  = 0x005F;

constexpr uint16_t COMMAND_STOP    = 0;
constexpr uint16_t COMMAND_FORWARD = 1;
constexpr uint16_t COMMAND_REVERSE = 2;
constexpr uint16_t COMMAND_BRAKE   = 3;

struct Motor
{
    uint8_t address;
    const char* name;
};

Motor motors[] =
{
    {DRIVER_1_ADDRESS, "Motor 1"},
    {DRIVER_2_ADDRESS, "Motor 2"}
};

constexpr size_t MOTOR_COUNT =
    sizeof(motors) / sizeof(motors[0]);

// ============================================================
// Modbus CRC-16
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
// RS485 helpers
// ============================================================

void clearReceiveBuffer()
{
    while (rs485.available())
    {
        rs485.read();
    }
}

void beginTransmit()
{
    digitalWrite(RS485_DIR_PIN, HIGH);
    delayMicroseconds(100);
}

void beginReceive()
{
    digitalWrite(RS485_DIR_PIN, LOW);
}

// ============================================================
// Write one register, function 0x06
// ============================================================

bool writeRegisterOnce(
    uint8_t deviceAddress,
    uint16_t registerAddress,
    uint16_t value)
{
    uint8_t request[8];

    request[0] = deviceAddress;
    request[1] = 0x06;
    request[2] = highByte(registerAddress);
    request[3] = lowByte(registerAddress);
    request[4] = highByte(value);
    request[5] = lowByte(value);

    const uint16_t crc = modbusCRC(request, 6);

    request[6] = lowByte(crc);
    request[7] = highByte(crc);

    delay(10);
    clearReceiveBuffer();

    beginTransmit();

    rs485.write(request, sizeof(request));
    rs485.flush();

    beginReceive();

    uint8_t response[8];
    size_t received = 0;
    const uint32_t startTime = millis();

    while (millis() - startTime < 400)
    {
        while (rs485.available() &&
               received < sizeof(response))
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

    const uint16_t receivedCRC =
        static_cast<uint16_t>(response[6]) |
        (static_cast<uint16_t>(response[7]) << 8);

    if (receivedCRC != modbusCRC(response, 6))
    {
        return false;
    }

    // Function 0x06 should echo the original request.
    for (uint8_t i = 0; i < 6; i++)
    {
        if (response[i] != request[i])
        {
            return false;
        }
    }

    return true;
}

bool writeRegister(
    uint8_t deviceAddress,
    uint16_t registerAddress,
    uint16_t value)
{
    constexpr uint8_t MAX_ATTEMPTS = 3;

    for (uint8_t attempt = 1;
         attempt <= MAX_ATTEMPTS;
         attempt++)
    {
        if (writeRegisterOnce(
                deviceAddress,
                registerAddress,
                value))
        {
            return true;
        }

        delay(30);
    }

    Serial.print("Driver ");
    Serial.print(deviceAddress);
    Serial.print(": write failed at register 0x");
    Serial.println(registerAddress, HEX);

    return false;
}

// ============================================================
// Read one register, function 0x03
// ============================================================

bool readRegisterOnce(
    uint8_t deviceAddress,
    uint16_t registerAddress,
    uint16_t& value)
{
    uint8_t request[8];

    request[0] = deviceAddress;
    request[1] = 0x03;
    request[2] = highByte(registerAddress);
    request[3] = lowByte(registerAddress);
    request[4] = 0x00;
    request[5] = 0x01;

    const uint16_t crc = modbusCRC(request, 6);

    request[6] = lowByte(crc);
    request[7] = highByte(crc);

    delay(10);
    clearReceiveBuffer();

    beginTransmit();

    rs485.write(request, sizeof(request));
    rs485.flush();

    beginReceive();

    uint8_t response[7];
    size_t received = 0;
    const uint32_t startTime = millis();

    while (millis() - startTime < 400)
    {
        while (rs485.available() &&
               received < sizeof(response))
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

    const uint16_t receivedCRC =
        static_cast<uint16_t>(response[5]) |
        (static_cast<uint16_t>(response[6]) << 8);

    if (receivedCRC != modbusCRC(response, 5))
    {
        return false;
    }

    if (response[0] != deviceAddress ||
        response[1] != 0x03 ||
        response[2] != 0x02)
    {
        return false;
    }

    value =
        (static_cast<uint16_t>(response[3]) << 8) |
        response[4];

    return true;
}

bool readRegister(
    uint8_t deviceAddress,
    uint16_t registerAddress,
    uint16_t& value)
{
    constexpr uint8_t MAX_ATTEMPTS = 3;

    for (uint8_t attempt = 1;
         attempt <= MAX_ATTEMPTS;
         attempt++)
    {
        if (readRegisterOnce(
                deviceAddress,
                registerAddress,
                value))
        {
            return true;
        }

        delay(30);
    }

    return false;
}

// ============================================================
// Motor commands
// ============================================================

bool stopMotor(uint8_t address)
{
    return writeRegister(
        address,
        REG_RUN_COMMAND,
        COMMAND_STOP);
}

bool startForward(uint8_t address)
{
    return writeRegister(
        address,
        REG_RUN_COMMAND,
        COMMAND_FORWARD);
}

bool setMotorSpeed(uint8_t address, uint16_t rpm)
{
    if (rpm > 3000)
    {
        rpm = 3000;
    }

    return writeRegister(
        address,
        REG_SPEED_COMMAND,
        rpm);
}

bool clearMotorFault(uint8_t address)
{
    return writeRegister(
        address,
        REG_FAULT,
        0);
}

// ============================================================
// Status display
// ============================================================

const char* runningStatusText(uint16_t status)
{
    switch (status)
    {
        case COMMAND_STOP:
            return "Stopped";

        case COMMAND_FORWARD:
            return "Forward";

        case COMMAND_REVERSE:
            return "Reverse";

        case COMMAND_BRAKE:
            return "Brake";

        default:
            return "Unknown";
    }
}

const char* faultText(uint16_t fault)
{
    switch (fault)
    {
        case 0:
            return "None";

        case 1:
            return "Overcurrent";

        case 2:
            return "Overtemperature";

        case 3:
            return "Overvoltage";

        case 4:
            return "Undervoltage";

        case 5:
            return "Hall sensor abnormality";

        case 6:
            return "Overspeed";

        case 8:
            return "Motor stalled";

        case 9:
            return "Peak current";

        default:
            return "Unknown fault";
    }
}

// ============================================================
// Initialize one driver
// ============================================================

bool initializeMotor(const Motor& motor)
{
    Serial.println();
    Serial.print("Initializing ");
    Serial.println(motor.name);

    uint16_t reportedAddress = 0;

    if (!readRegister(
            motor.address,
            REG_ADDRESS,
            reportedAddress))
    {
        Serial.println("  No RS485 response.");
        return false;
    }

    Serial.print("  Address confirmed: ");
    Serial.println(reportedAddress);

    // Internal/RS485 control mode.
    if (!writeRegister(
            motor.address,
            REG_CONTROL_MODE,
            1))
    {
        Serial.println("  Control-mode command failed.");
        return false;
    }

    delay(30);

    // Correct setting for a four-pole motor.
    if (!writeRegister(
            motor.address,
            REG_POLE_PAIRS,
            MOTOR_POLE_PAIRS))
    {
        Serial.println("  Pole-pair command failed.");
        return false;
    }

    delay(30);

    clearMotorFault(motor.address);
    delay(100);

    stopMotor(motor.address);
    delay(100);

    if (!setMotorSpeed(
            motor.address,
            FIXED_RPM))
    {
        Serial.println("  Speed command failed.");
        return false;
    }

    delay(50);

    if (!startForward(motor.address))
    {
        Serial.println("  Start command failed.");
        return false;
    }

    Serial.println("  Start command sent.");
    return true;
}

// ============================================================
// Read feedback
// ============================================================

void printMotorFeedback(const Motor& motor)
{
    uint16_t actualSpeed = 0;
    uint16_t runStatus = 0;
    uint16_t faultCode = 0;

    Serial.print(motor.name);
    Serial.print(" | ");

    if (readRegister(
            motor.address,
            REG_ACTUAL_SPEED,
            actualSpeed))
    {
        Serial.print("Actual speed: ");
        Serial.print(actualSpeed);
        Serial.print(" RPM");
    }
    else
    {
        Serial.print("Speed: no response");
    }

    delay(20);

    if (readRegister(
            motor.address,
            REG_RUN_COMMAND,
            runStatus))
    {
        Serial.print(" | Status: ");
        Serial.print(runningStatusText(runStatus));
    }
    else
    {
        Serial.print(" | Status: no response");
    }

    delay(20);

    if (readRegister(
            motor.address,
            REG_FAULT,
            faultCode))
    {
        Serial.print(" | Fault: ");
        Serial.print(faultCode);
        Serial.print(" (");
        Serial.print(faultText(faultCode));
        Serial.print(")");
    }
    else
    {
        Serial.print(" | Fault: no response");
    }

    Serial.println();
}

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    pinMode(RS485_DIR_PIN, OUTPUT);
    digitalWrite(RS485_DIR_PIN, LOW);

    rs485.begin(9600, SERIAL_8N1);

    delay(2000);

    Serial.println();
    Serial.println("Starting two BLD-305S motors.");

    // Initialize independently so Motor 2 can run even if Motor 1 fails.
    initializeMotor(motors[0]);
    delay(100);

    initializeMotor(motors[1]);
    delay(100);

    Serial.println();
    Serial.println("Initialization finished.");
}

// ============================================================
// Main loop
// ============================================================

void loop()
{
    static uint32_t previousFeedbackTime = 0;

    if (millis() - previousFeedbackTime >= 500)
    {
        previousFeedbackTime = millis();

        Serial.println("----------------------------------------");

        printMotorFeedback(motors[0]);
        delay(30);

        printMotorFeedback(motors[1]);
    }
}