#ifndef DC_SERVO_H
#define DC_SERVO_H

#include <Arduino.h>

#include <QuadEncoder.h>

//------------------------------------------------------------------------------
// Control Cycles
//------------------------------------------------------------------------------

const uint32_t cyclesPerSecond = 100;
const uint32_t ctrlTimeMicros = 1000000 / cyclesPerSecond; // microseconds

//------------------------------------------------------------------------------
// PWM configuration
//------------------------------------------------------------------------------

const uint8_t pwmResolution(12);
const uint16_t pwmMaxValue((1 << pwmResolution));

//------------------------------------------------------------------------------
// Steps
//------------------------------------------------------------------------------

const double ticksPerStep = 2.0;
const double numGearTeeth = 10.0;
const double chainPitch = 0.25;
const uint32_t ppr = 44; // encoder pulses per revolution
const double gearRatio = 280.0;
const double inchPerStep = numGearTeeth * chainPitch / (ppr * gearRatio / ticksPerStep); // inch
const double stepsPerMM = 1 / (inchPerStep * 25.4);

//------------------------------------------------------------------------------
// PID
//------------------------------------------------------------------------------

struct PID {
    double kp, ki, kd;
    double propTerm, intTerm, derivTerm;

    void calc_ctrl(double pos, double vel)
    {
    }
};

//------------------------------------------------------------------------------
// Servo
//------------------------------------------------------------------------------

struct Servo {
    const uint8_t num;
    const uint8_t pinN1, pinN2, pinPWM;
    QuadEncoder enc;

    uint16_t pwmFrequency = 9500;
    double time = 0;
    double ctrl = 0;
    int32_t pos = 0;
    double vel = 0;
    volatile double setpoint = 0;

    PID pid;

    enum Mode {
        ModeAuto,
        ModeCtrl,
        ModeSine,
    };

    Mode mode = ModeAuto;
    double sineFreq = 0;
    double sineAmplitude = 0;

    Servo(uint8_t num, uint8_t pinN1, uint8_t pinN2, uint8_t pinPWM,
        uint8_t pinEncA, uint8_t pinEncB)
        : num(num)
        , pinN1(pinN1)
        , pinN2(pinN2)
        , pinPWM(pinPWM)
        , enc(num, pinEncA, pinEncB, 1)
    {
    }

    void setup()
    {
        // Set servo control pin modes
        pinMode(pinN1, OUTPUT);
        pinMode(pinN2, OUTPUT);
        pinMode(pinPWM, OUTPUT);

        // Set PWM frequency and resolution
        analogWriteResolution(pwmResolution);
        analogWriteFrequency(pinPWM, pwmFrequency);

        // Initialize encoder
        enc.setInitConfig();
        enc.init();

        // Print servo initialization message
        Serial.print("Servo ");
        Serial.print(num);
        Serial.println(" Initialized");
    }

    void loop(uint32_t deltaMicros)
    {
        // Read encoder value
        pos = enc.read();

        // Save delta time
        logDelta = deltaMicros;

        // Switch based on selected mode
        switch (mode) {
        case ModeAuto:
            break;
        case ModeCtrl:
            break;
        case ModeSine:
            time += (deltaMicros / 1e6);
            if (time > sineFreq) {
                time -= sineFreq;
            }
            ctrl = sineAmplitude * sin(2 * PI * sineFreq * time);
            break;
        }

        // Limit check control signal
        if (ctrl > 1.0) {
            ctrl = 1.0;
        } else if (ctrl < -1.0) {
            ctrl = -1.0;
        }

        // Output control to motor driver
        if (ctrl > 0) {
            digitalWrite(pinN1, HIGH);
            digitalWrite(pinN2, LOW);
            analogWrite(pinPWM, (pwmMaxValue * ctrl));
        } else if (ctrl < 0) {
            digitalWrite(pinN1, LOW);
            digitalWrite(pinN2, HIGH);
            analogWrite(pinPWM, (pwmMaxValue * -ctrl));
        } else {
            digitalWrite(pinN1, HIGH);
            digitalWrite(pinN2, HIGH);
            analogWrite(pinPWM, 0);
        }
    }

    void setModeAuto()
    {
        mode = ModeAuto;
    }

    void setModeCtrl(double constCtrl)
    {
        mode = ModeCtrl;
        ctrl = constCtrl;
    }

    void setModeSine(double amplitude, double frequency)
    {
        mode = ModeSine;
        sineAmplitude = amplitude;
        sineFreq = frequency;
        time = 0;
    }

    void setPWMFrequency(uint32_t pwmFrequency)
    {
        analogWriteFrequency(pinPWM, pwmFrequency);
    }

    void setKp(double kp)
    {
        pid.kp = kp;
    }

    void setKi(double ki)
    {
        pid.ki = ki;
    }

    void setKd(double kd)
    {
        pid.kd = kd;
    }

    void step(int32_t num)
    {
        setpoint += ticksPerStep * num;
    }

    // -------------------------------------------------------------------------
    // Logging
    // -------------------------------------------------------------------------

    struct Record {
        uint32_t num, time;
        float ctrl, pos, vel;
    };

    bool enableLog = false;
    uint32_t logDelta = 0;

    void log()
    {
        if ((logDelta > 0) && enableLog) {
            Record rec = { num, logDelta, (float)ctrl, (float)pos, (float)vel };
            Serial.write((uint8_t*)(&rec), sizeof(Record));
            logDelta = 0;
        }
    }
};

#endif