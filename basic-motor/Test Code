constexpr uint8_t EN_PIN  = 2;
constexpr uint8_t BK_PIN  = 3;
constexpr uint8_t DIR_PIN = 4;

void setup()
{
    pinMode(EN_PIN, OUTPUT);
    pinMode(BK_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    // Safe starting condition
    digitalWrite(EN_PIN, HIGH);   // Motor disabled
    digitalWrite(BK_PIN, HIGH);   // Brake released
    digitalWrite(DIR_PIN, HIGH);  // Forward direction

    delay(3000);

    // Start motor
    digitalWrite(BK_PIN, HIGH);   // Make sure brake is released
    digitalWrite(EN_PIN, LOW);    // LOW enables motor
}

void loop()
{
}