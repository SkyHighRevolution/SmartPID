#include <AutoTunePID.h>

// PID Control Variables
double input = 0.0;
double output = 0.0;
double setpoint = 150.0;

// Initialize the controller (Forward acting / Direct, e.g. motor speed)
// Gains initialized to zero because we will run the Auto-Tuner to compute them
AutoTunePID myPID(&input, &output, &setpoint, 0.0, 0.0, 0.0, AutoTunePID::FORWARD);

// Simulated Motor Process Parameters
double simState = 0.0;
const double motorGain = 1.25;      // Steady-state gain (K)
const double motorTimeConst = 1.8;   // Time constant in seconds (Tau)
unsigned long lastSimTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Set limits to standard Arduino 8-bit PWM (0 - 255)
  myPID.SetOutputLimits(0, 255);
  
  // Set sample time (50 milliseconds) for high-speed response
  myPID.SetSampleTime(50);
  
  Serial.println("=== PID Auto-Tuning Motor Speed Simulation ===");
  Serial.println("Starting Åström-Hägglund relay feedback tuning phase...");
  
  // Start Tuning: 
  // - Relay amplitude: 60 (oscillates +/- 60 units around the bias level)
  // - Hysteresis: 1.5 (suppresses fake toggles due to noise)
  // - Timeout: 45000 ms (abort tuning if it exceeds 45s)
  // - Cycles to analyze: 5 full cycles
  myPID.StartTuning(60.0, 1.5, 45000, 5);
  
  lastSimTime = millis();
}

void loop() {
  unsigned long now = millis();
  
  // Simulate the first-order process physics at a constant 10ms rate
  if (now - lastSimTime >= 10) {
    double dt = (now - lastSimTime) / 1000.0;
    lastSimTime = now;
    
    // First-order motor speed dynamics: dy/dt = (K * u - y) / Tau
    double dState = (motorGain * output - simState) / motorTimeConst;
    simState += dState * dt;
    
    // Read simulated input with minor measurement noise
    input = simState + ((random(-100, 100) / 100.0) * 0.15);
  }
  
  // Process the PID / Tuning logic
  bool updated = myPID.Compute();
  
  if (updated) {
    if (myPID.GetMode() == AutoTunePID::TUNING) {
      // Print tuning oscillations to Serial (perfect for Arduino Serial Plotter)
      Serial.print("TUNING_PHASE ");
      Serial.print("Setpoint:"); Serial.print(setpoint);
      Serial.print(",");
      Serial.print("Input:"); Serial.print(input);
      Serial.print(",");
      Serial.print("Output:"); Serial.println(output);
      
    } else if (myPID.IsTuningComplete()) {
      static bool printedGains = false;
      if (!printedGains) {
        printedGains = true;
        Serial.println("\n--- Auto-Tuning Completed Successfully! ---");
        Serial.print("Tuning Rule Applied: Ziegler-Nichols PID\n");
        Serial.print("Calculated Gains -> Kp: "); Serial.print(myPID.GetKp(), 4);
        Serial.print(" | Ki: "); Serial.print(myPID.GetKi(), 4);
        Serial.print(" | Kd: "); Serial.println(myPID.GetKd(), 4);
        Serial.println("Transitioning to active PID control loop...\n");
        
        // Change setpoint to verify performance of calculated parameters
        setpoint = 180.0;
      }
      
      // Print active control loop values (perfect for step response visualizer)
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print("ACTIVE_CONTROL ");
        Serial.print("Setpoint:"); Serial.print(setpoint);
        Serial.print(",");
        Serial.print("Input:"); Serial.print(input);
        Serial.print(",");
        Serial.print("Output:"); Serial.println(output);
      }
    }
  }
}
