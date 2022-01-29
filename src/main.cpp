#include <ArduinoJson.h>

#include <vector>

#include "Servo.hpp"

// Servo Input
const uint8_t pinEnable = 13;
const uint8_t pinStepA = 14;
const uint8_t pinDirA = 15;
const uint8_t pinStepB = 16;
const uint8_t pinDirB = 17;

// Servo Output
const uint8_t pinN1A = 7;
const uint8_t pinN2A = 8;
const uint8_t pinPWMA = 11;
const uint8_t pinN1B = 9;
const uint8_t pinN2B = 10;
const uint8_t pinPWMB = 12;

// Quadrature Encoder pins (0, 1, 2, 3, 4, 5, 7, 30, 31, 33)
const uint8_t pinEnc1A = 0;
const uint8_t pinEnc1B = 1;
const uint8_t pinEnc2A = 3;
const uint8_t pinEnc2B = 4;

//------------------------------------------------------------------------------
// Servos
//------------------------------------------------------------------------------

std::vector<Servo> servos = {
    Servo(1, pinN1A, pinN2A, pinPWMA, pinEnc1A, pinEnc1B),
    Servo(2, pinN1B, pinN2B, pinPWMB, pinEnc2A, pinEnc2B),
};

//------------------------------------------------------------------------------
// Control Loop
//------------------------------------------------------------------------------

auto prevMicros = micros();

void ctrlLoop()
{
    // Calculate cycle duration
    auto curMicros = micros();
    auto deltaMicros = curMicros - prevMicros;
    prevMicros = curMicros;

    // Call loop for each servo
    for (auto& servo : servos) {
        servo.loop(deltaMicros);
    }
}

//------------------------------------------------------------------------------
// Auxiliary Loop
//------------------------------------------------------------------------------

StaticJsonDocument<512> doc;

void auxLoop(void)
{
    // Log Servos
    for (auto& servo : servos) {
        servo.log();
    }

    if (Serial.available() > 0) {
        DeserializationError err = deserializeJson(doc, Serial);
        if (err) {
            Serial.print(F("deserializeJson() failed with code "));
            Serial.println(err.f_str());
            return;
        }

        size_t servoNums = doc["servo"] | 3;
        double Kp = doc["Kp"] | -1.0;
        double Ki = doc["Ki"] | -1.0;
        double Kd = doc["Kd"] | -1.0;
        uint16_t pwm = doc["pwm"] | 0;
        int32_t steps = doc["steps"] | 0;
        double ctrl = doc["ctrl"] | 2.0;
        double sineA = doc["sineA"] | 0.0;
        double sineF = doc["sineF"] | 0.0;

        // Loop through servos
        for (auto& servo : servos) {
            if ((servo.num & servoNums) == 0) {
                continue;
            }

            Serial.print("> Servo ");
            Serial.println(servo.num);

            if (Kp >= 0) {
                servo.setKp(Kp);
                Serial.print("  Set Kp = ");
                Serial.println(Kp);
            }

            if (Ki >= 0) {
                servo.setKi(Ki);
                Serial.print("  Set Ki = ");
                Serial.println(Ki);
            }

            if (Kd >= 0) {
                servo.setKd(Kd);
                Serial.print("  Set Kd = ");
                Serial.println(Kd);
            }

            if (pwm > 0) {
                servo.setPWMFrequency(pwm);
                Serial.print("  Set PWM frequency = ");
                Serial.println(pwm);
            }

            if (steps != 0) {
                servo.step(steps);
                Serial.print("  Set steps = ");
                Serial.println(steps);
            }

            if ((-1 <= ctrl) && (ctrl <= 1)) {
                servo.setModeCtrl(ctrl);
                Serial.print("  Set ctrl = ");
                Serial.println(ctrl);
            }

            if ((sineA != 0) || (sineF != 0)) {
                servo.setModeSine(sineA, sineF);
                Serial.print("  Set sine amplitude = ");
                Serial.println(sineA);
                Serial.print("  Set sine frequency = ");
                Serial.println(sineF);
            }

            // If log specified set flag
            if (doc.containsKey("log")) {
                servo.enableLog = doc["log"];
                if (servo.enableLog) {
                    Serial.println("  Enabled logging");
                } else {
                    Serial.println("  Disabled logging");
                }
            }
        }

        Serial.println("ok");
    }
}

void setup()
{
    // Wait for serial connecition to be ready
    while (!Serial)
        continue;

    Serial.println("dc-servo v0.1");

    // Initialize servos
    for (auto& servo : servos) {
        servo.setup();
    }
}

elapsedMicros cycleCounter;

void loop()
{
    if (cycleCounter >= ctrlTimeMicros) {
        cycleCounter -= ctrlTimeMicros;
        ctrlLoop();
    } else {
        auxLoop();
    }
}
