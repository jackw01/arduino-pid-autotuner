//
// PID automated tuning (Ziegler-Nichols/relay method) for Arduino and compatible boards
// Copyright (c) 2017 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details
//

#include <Arduino.h>

class PIDAutotuner {

    public:

        PIDAutotuner();

        // Configure parameters for PID tuning
        // See README for more details - https://github.com/jackw01/arduino-pid-autotuner/blob/master/README.md
        // targetInputValue: the target value to tune to
        // loopInterval: PID loop interval in microseconds - must match whatever the PID loop being tuned runs at
        // tuningCycles: number of cycles that the tuning runs for (optional, default is 10)
        void setTargetInputValue(double target);
        void setLoopInterval(long interval);
        void setTuningCycles(int tuneCycles);

        // Must be called immediately before the tuning loop starts
        void startTuningLoop();

        // Automatically tune PID
        // This function must be run in a loop at the same speed as the PID loop being tuned
        // See README for more details - https://github.com/jackw01/arduino-pid-autotuner/blob/master/README.md
        bool tunePID(double input);

        // Get results of most recent tuning
        double getKp();
        double getKi();
        double getKd();

        bool isFinished(); // Is the tuning finished?

    private:

        double targetInputValue = 0;
        double loopInterval = 0;
        int cycles = 10;

        int i;
        bool output;
        long microseconds, t1, t2, tHigh, tLow;
        double max, min;
        double pAverage, iAverage, dAverage;

        double kp, ki, kd;
};
