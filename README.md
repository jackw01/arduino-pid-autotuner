# arduino-pid-autotuner
Automated PID tuning using Ziegler-Nichols/relay method on Arduino and compatible boards

## How does it work?
`PIDAutotuner.h` and `PIDAutotuner.cpp` are fully commented to explain how the algorithm works.

## What PID controller does this work with?
This algorithm should work with all PID controllers and PID control libraries if it is properly configured.

## Example code
```c
#include <PIDAutotuner.h>

void setup() {

    PIDAutotuner tuner = PIDAutotuner();

    // Set the target value to tune to
    // This will depend on what you are tuning. This should be set to a value within
    // the usual range of the setpoint. For low-inertia systems, values at the lower
    // end of this range usually give better results. For anything else, start with a
    // value at the middle of the range.
    tuner.setTargetInputValue(targetInputValue);

    // Set the loop interval in microseconds
    // This must be the same as the interval the PID control loop will run at
    tuner.setLoopInterval(loopInterval);

    // Set the output range
    // These are the maximum and minimum possible output values of whatever you are
    // using to control the system (analogWrite is 0-255)
    tuner.setOutputRange(0, 255);

    // Set the Ziegler-Nichols tuning mode
    // Set it to either PIDAutotuner::znModeBasicPID, PIDAutotuner::znModeLessOvershoot,
    // or PIDAutotuner::znModeNoOvershoot. Test with znModeBasicPID first, but if there
    // is too much overshoot you can try the others.
    tuner.setZNMode(PIDAutotuner::znModeBasicPID);

    // This must be called immediately before the tuning loop
    tuner.startTuningLoop();

    // Run a loop until tuner.isFinished() returns true
    long microseconds;
    while (!tuner.isFinished()) {

        // This loop must run at the same speed as the PID control loop being tuned
        long prevMicroseconds = microseconds;
        microseconds = micros();

        // Get input value here (temperature, encoder position, velocity, etc)
        double input = doSomethingToGetInput();

        // Call tunePID() with the input value
        double output = tuner.tunePID(input);

        // Set the output - tunePid() will return values within the range configured
        // by setOutputRange(). Don't change the value or the tuning results will be
        // incorrect.
        doSomethingToSetOutput(output);

        // This loop must run at the same speed as the PID control loop being tuned
        while (micros() - microseconds < loopInterval) delayMicroseconds(1);
    }

    // Turn the output off here.
    doSomethingToSetOutput(0);

    // Get PID gains - set your PID controller's gains to these
    double kp = tuner.getKp();
    double ki = tuner.getKi();
    double kd = tuner.getKd();
}

void loop() {

    // ...
}
```
