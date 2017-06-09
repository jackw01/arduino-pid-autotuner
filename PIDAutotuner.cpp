//
// Copyright (c) 2016-2017 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details
//

#include "PIDAutotuner.h"

PIDAutotuner::PIDAutotuner() {

}

// Set target input for tuning
void PIDAutotuner::setTargetInputValue(double target) {

    targetInputValue = target;
}

// Set loop interval
void PIDAutotuner::setLoopInterval(long interval) {

    loopInterval = interval;
}


// Set tuning cycles
void PIDAutotuner::setTuningCycles(int tuneCycles) {

    cycles = tuneCycles;
}

// Start loop
void PIDAutotuner::startTuningLoop() {

    i = 0;
    output = true;
    t1 = t2 = micros();
    microseconds = tHigh = tLow = 0;
    max = -1000000;
    min = 1000000;
    pAverage = iAverage = dAverage = 0;

    sei();
}

// Run one cycle of the loop
bool PIDAutotuner::tunePID(double input) {

    // Useful information on the algorithm used (Ziegler-Nichols method/Relay method)
    // http://www.processcontrolstuff.net/wp-content/uploads/2015/02/relay_autot-2.pdf
    // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    // https://www.cds.caltech.edu/~murray/courses/cds101/fa04/caltech/am04_ch8-3nov04.pdf

    // Calculate time delta
    long prevMicroseconds = microseconds;
    microseconds = micros();
    double deltaT = microseconds - prevMicroseconds;

    // Calculate max and min
    max = max(max, input);
    min = min(min, input);

    // Output is on and input signal has risen to target
    if (output && input > targetInputValue) {

        output = false;
        t1 = micros();
        tHigh = t1 - t2;
        max = targetInputValue;
    }

    // Output is off and input signal has dropped to target
    if (!output && input < targetInputValue) {

        output = true;
        t2 = micros();
        tLow = t2 - t1;

        double ku = (4.0 * 0.5) / (M_PI * (max - min) * 2.0);
        double tu = tLow + tHigh;

        // How gains are calculated
        // PID control algorithm needs Kp, Ki, and Kd
        // Ziegler-Nichols tuning method gives Kp, Ti, and Td
        //
        // Kp = 0.6Ku = Kc
        // Ti = 0.5Tu = Kc/Ki
        // Td = 0.125Tu = Kd/Kc
        //
        // Solving these equations for Kp, Ki, and Kd gives this:
        //
        // Kp = 0.6Ku
        // Ki = Kp / (0.5Tu) =
        // Kd = 0.125 * Kp * Tu

        // Constants
        // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

        // Normal PID
        //double pConstant = 0.6, iConstant = 0.5, dConstant = 0.125;

        // Less overshoot
        double pConstant = 0.33, iConstant = 0.5, dConstant = 0.33;

        // No overshoot
        //double pConstant = 0.2, iConstant = 0.5, dConstant = 0.33;

        kp = pConstant * ku;
        ki = (kp / (iConstant * tu)) * loopInterval;
        kd = (dConstant * kp * tu) / loopInterval;

        if (i > 1) {
            pAverage += kp;
            iAverage += ki;
            dAverage += kd;
        }

        min = targetInputValue;

        i ++;
    }

    if (i >= cycles) {

        output = false;

        kp = pAverage / (cycles - 2);
        ki = iAverage / (cycles - 2);
        kd = dAverage / (cycles - 2);
    }

    return output;
}

// Get PID constants after tuning
double PIDAutotuner::getKp() { return kp; };
double PIDAutotuner::getKi() { return ki; };
double PIDAutotuner::getKd() { return kd; };

// Is the tuning loop finished?
bool PIDAutotuner::isFinished() {

    return (i >= cycles);
}

// Automatically tune PID
/*
void PIDAutotuner::tunePID2(double targetInputValue, int cycles, long loopInterval) {

    // Useful information on the algorithm used (Ziegler-Nichols method/Relay method)
    // http://www.processcontrolstuff.net/wp-content/uploads/2015/02/relay_autot-2.pdf
    // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    // https://www.cds.caltech.edu/~murray/courses/cds101/fa04/caltech/am04_ch8-3nov04.pdf

    double input = 0;
    int i = 0;
    bool outputOn = true;

    long microseconds;
    long t1 = micros(), t2 = t1;
    long tHigh = 0, tLow = 0;

    double max = -1000000, min = 1000000;
    double pAverage = 0, iAverage = 0, dAverage = 0;

    sei();
    //driveMotorController.setSpeed(1.0, TB6612::tb6612ChannelA);

    while (true) {

        // Calculate time delta
        long prevMicroseconds = microseconds;
        microseconds = micros();
        double deltaT = microseconds - prevMicroseconds;

        // Get input
        //input = inputSource.getValue();

        max = max(max, input);
        min = min(min, input);

        if (outputOn && input > targetInputValue) {

            outputOn = false;
            //driveMotorController.setSpeed(0.0, TB6612::tb6612ChannelA);

            t1 = micros();
            tHigh = t1 - t2;

            max = targetInputValue;
        }

        if (!outputOn && input < targetInputValue) {

            outputOn = true;
            //driveMotorController.setSpeed(1.0, TB6612::tb6612ChannelA);

            t2 = micros();
            tLow = t2 - t1;

            double ku = (4.0 * 0.5) / (M_PI * (max - min) * 2.0);
            double tu = tLow + tHigh;

            // How gains are calculated
            // PID control algorithm needs Kp, Ki, and Kd
            // Ziegler-Nichols tuning method gives Kp, Ti, and Td
            //
            // Kp = 0.6Ku = Kc
            // Ti = 0.5Tu = Kc/Ki
            // Td = 0.125Tu = Kd/Kc
            //
            // Solving these equations for Kp, Ki, and Kd gives this:
            //
            // Kp = 0.6Ku
            // Ki = Kp / (0.5Tu) =
            // Kd = 0.125 * Kp * Tu

            // Constants
            // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

            // Normal PID
            //double pConstant = 0.6, iConstant = 0.5, dConstant = 0.125;

            // Less overshoot
            double pConstant = 0.33, iConstant = 0.5, dConstant = 0.33;

            // No overshoot
            //double pConstant = 0.2, iConstant = 0.5, dConstant = 0.33;

            kp = pConstant * ku;
            ki = (kp / (iConstant * tu)) * loopInterval;
            kd = (dConstant * kp * tu) / loopInterval;

            if (i > 1) {
                pAverage += kp;
                iAverage += ki;
                dAverage += kd;
            }

            min = targetInputValue;

            i ++;
        }

        if (i >= cycles) {

            //driveMotorController.setSpeed(0.0, TB6612::tb6612ChannelA);
            outputOn = false;

            kp = pAverage / (cycles - 2);
            ki = iAverage / (cycles - 2);
            kd = dAverage / (cycles - 2);

            return;
        }

        while ((micros() - microseconds) < loopInterval) {
        }
    }
}*/
