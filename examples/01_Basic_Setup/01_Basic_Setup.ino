#include <AutoTunePID.h>

// Simulated hardware pins
const int inputPin = A0;
const int outputPin = 3;

// PID Variables
double input = 0.0;
double output = 0.0;
double setpoint = 100.0;

// Initialize the controller with initial conservative parameter guesses
// Kp = 2.0, Ki = 0.5, Kd = 0.1, Action = REVERSE (heating)
AutoTunePID myPID(&input, &output, &setpoint, 2.0, 0.5, 0.1,
                  AutoTunePID::REVERSE);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // Limit output to standard Arduino 8-bit PWM (0 - 255)
  myPID.SetOutputLimits(0, 255);

  // Set PID loop run rate to 100 milliseconds
  myPID.SetSampleTime(100);

  // Start the controller in AUTOMATIC mode
  myPID.SetMode(AutoTunePID::AUTOMATIC);

  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);

  Serial.println("AutoTunePID Basic Setup Example Initialized");
}

void loop() {
  // Read measurement input (simulate scaling 0-5V sensor to 0-200.0 physical
  // scale)
  input = analogRead(inputPin) * (5.0 / 1023.0) * 40.0;

  // Compute PID loop update
  myPID.Compute();

  // Output PWM signal to actuator
  analogWrite(outputPin, (int)output);

  // Report state data to Serial Plotter/Monitor every 500 milliseconds
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Input: ");
    Serial.print(input);
    Serial.print(" | Output: ");
    Serial.println(output);
  }
}
