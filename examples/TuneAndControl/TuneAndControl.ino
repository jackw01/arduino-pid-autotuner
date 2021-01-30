#include <pidautotuner.h>

double targetValue = 200.0;
int loopInterval = 10000;

PIDAutotuner tuner = PIDAutotuner();

void setup() {

  // Set the target value to tune to
  // This will depend on what you are tuning. This should be set to a value within
  // the usual range of the setpoint. For low-inertia systems, values at the lower
  // end of this range usually give better results. For anything else, start with a
  // value at the middle of the range.
  tuner.setTargetValue(targetValue);

  // Set the loop interval in microseconds
  // This must be the same as the interval the PID control loop will run at
  tuner.setLoopInterval(loopInterval);

  // Set the output range
  // These are the minimum and maximum possible output values of whatever you are
  // using to control the system (Arduino analogWrite, for example, is 0-255)
  tuner.setOutputRange(0, 255);

  // Set the Ziegler-Nichols tuning mode
  // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
  // or PIDAutotuner::ZNModeNoOvershoot. Defaults to ZNModeNoOvershoot as it is the
  // safest option.
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

  // This must be called immediately before the tuning loop
  // Must be called with the current time in microseconds
  tuner.startTuningLoop(micros());

  // Run a loop until tuner.isFinished() returns true
  long microseconds;
  while (!tuner.isFinished()) {

    // This loop must run at the same speed as the PID control loop being tuned
    long prevMicroseconds = microseconds;
    microseconds = micros();

    // Get input value here (temperature, encoder position, velocity, etc)
    double input = doSomethingToGetInput();

    // Call tunePID() with the input value and current time in microseconds
    double output = tuner.tune(input, microseconds);

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
  long microseconds = micros();
  double input = doSomethingToGetInput();
  double output = tuner.run(input, microseconds);
  doSomethingToSetOutput(output);
  // This loop must run at the same speed as the PID control loop being tuned
  while (micros() - microseconds < loopInterval) delayMicroseconds(1);
}

double doSomethingToGetInput(void) {
  return 1.0;
}
void doSomethingToSetOutput(double input) {

}
