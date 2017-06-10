//
// PID automated tuning (Ziegler-Nichols/relay method) for Arduino and compatible boards
// Copyright (c) 2017 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details
//

#ifndef PIDAUTOTUNER_H
#define PIDAUTOTUNER_H

#include <Arduino.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class PIDAutotuner {

    public:

        // Constants for Ziegler-Nichols tuning mode
        static const int znModeBasicPID = 0;
        static const int znModeLessOvershoot = 1;
        static const int znModeNoOvershoot = 2;

        PIDAutotuner();

        // Configure parameters for PID tuning
        // See README for more details - https://github.com/jackw01/arduino-pid-autotuner/blob/master/README.md
        // targetInputValue: the target value to tune to
        // loopInterval: PID loop interval in microseconds - must match whatever the PID loop being tuned runs at
        // outputRange: min and max values of the output that can be used to control the system (0, 255 for analogWrite)
        // znMode: Ziegler-Nichols tuning mode (znModeBasicPID, znModeLessOvershoot, znModeNoOvershoot)
        // tuningCycles: number of cycles that the tuning runs for (optional, default is 10)
        void setTargetInputValue(double target);
        void setLoopInterval(long interval);
        void setOutputRange(double min, double max);
        void setZNMode(byte zn);
        void setTuningCycles(int tuneCycles);

        // Must be called immediately before the tuning loop starts
        void startTuningLoop();

        // Automatically tune PID
        // This function must be run in a loop at the same speed as the PID loop being tuned
        // See README for more details - https://github.com/jackw01/arduino-pid-autotuner/blob/master/README.md
        double tunePID(double input);

        // Get results of most recent tuning
        double getKp();
        double getKi();
        double getKd();

        bool isFinished(); // Is the tuning finished?

    private:

        double targetInputValue = 0;
        double loopInterval = 0;
        double minOutput, maxOutput;
        byte znMode = znModeBasicPID;
        int cycles = 10;

        // See startTuningLoop()
        int i;
        bool output;
        double outputValue;
        long microseconds, t1, t2, tHigh, tLow;
        double max, min;
        double pAverage, iAverage, dAverage;

        double kp, ki, kd;
};

#endif
