#include "HX711.h"

#define interruptPin 2 // Pin connected to the RPM signal
#define LOADCELL_DOUT_PIN 4
#define LOADCELL_SCK_PIN 3
#define ACS712_PIN A0

// Tuning multiplier for the loadcell
#define MULTIPLIER 2280.f
// ACS712 calibration values
#define ACS712_OFFSET 513 // Adjust based on your sensor (e.g., 512 for 5V/10-bit ADC)
#define ACS712_SENSITIVITY 0.185 // Sensitivity in V/A (e.g., 185mV/A for 5A module)

// Create an HX711 object
HX711 scale;

volatile unsigned long isrMicros;  // Timestamp of the last interrupt
volatile unsigned long isrCount;   // Number of interrupts detected
volatile bool newIsrMicros = false; // Flag for new data from ISR

unsigned long revMicros;      // Last recorded timestamp
unsigned long prevRevMicros;  // Previous timestamp
unsigned long revDuration;    // Time between revolutions
unsigned long revCount;       // Copy of isrCount for main loop

unsigned long prevDisplayMillis; // Timestamp for display updates
unsigned long displayInterval = 1000; // Update interval (1 second)

unsigned long lastRevUpdateMillis; // Timestamp of the last revDuration update
unsigned long resetInterval = 5000; // Time threshold to reset everything (5 seconds)

// Debounce variables
unsigned long lastInterruptMicros = 0; // Last interrupt time (in micros)
unsigned long debounceDelay = 10;      // Debounce delay (in microseconds)

// Timing for sensor readings
unsigned long lastSensorReadingMillis = 0;
unsigned long sensorReadingInterval = 500; // 500ms interval for sensor readings

int mode = 0; // 0 = RPM, 1 = Torque & Current

void setup() {
  Serial.begin(115200);
  Serial.println("RPM, Torque, and Current Measurement");
  
  pinMode(interruptPin, INPUT_PULLUP); // Interrupt pin with pull-up resistor
  isrCount = 0;
  attachInterrupt(digitalPinToInterrupt(interruptPin), revDetectorISR, RISING);
  lastRevUpdateMillis = millis(); // Initialize lastRevUpdateMillis

  // Initialize the load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Adjust this value based on your calibration
  scale.set_scale(MULTIPLIER); 
  scale.tare(); // Tare the scale at startup
}

void loop() {
  getIsrData();
  
  // Check if revDuration has not changed in 5 seconds
  if (millis() - lastRevUpdateMillis >= resetInterval && revCount > 0) { // Prevent reset if no pulses detected
    resetData(); // Reset all variables
  }

  // Read the sensors at intervals (non-blocking)
  if (millis() - lastSensorReadingMillis >= sensorReadingInterval) {
    lastSensorReadingMillis = millis();
    
    // Change mode based on user input
    if (Serial.available()) {
      char input = Serial.read(); // Read serial input
      if (input == 'r') {
        mode = 0;  // Set mode to RPM
        Serial.println("Mode: RPM");
      } else if (input == 't') {
        mode = 1;  // Set mode to Torque and Current
        Serial.println("Mode: Torque & Current");
      }
    }

    // Measure based on current mode
    if (mode == 0) {
      measureRPM();  // Measure RPM only
    } else if (mode == 1) {
      measureTorqueAndCurrent();  // Measure Torque and Current
    }
  }
}

// Retrieve ISR data safely
void getIsrData() {
  if (newIsrMicros) {
    prevRevMicros = revMicros; // Save the previous timestamp
    noInterrupts(); // Disable interrupts while accessing shared data
    revMicros = isrMicros;    // Update the current timestamp
    revCount = isrCount;      // Copy the count
    newIsrMicros = false;     // Reset the flag
    interrupts(); // Re-enable interrupts
    revDuration = revMicros - prevRevMicros; // Calculate revolution duration

    // Update lastRevUpdateMillis since revDuration was updated
    lastRevUpdateMillis = millis();
  }
}

// Measure RPM and display
void measureRPM() {
  if (revDuration > 0) {
    float rpm = 60000000.0 / revDuration; // Convert duration to RPM
    Serial.print("RPM: ");
    Serial.println(rpm);
  } else {
    Serial.println("Waiting for signal...");
  }
}

// Measure Torque and Current
void measureTorqueAndCurrent() {
  // Read the weight from the load cell 
  float weight = scale.get_units(10); // Average of 10 readings
  // Print the calculated torque to the Serial Monitor
  Serial.print("Torque: ");
  Serial.print(weight);
  Serial.println(" Ncm"); // Change to other units as needed

  // Read the current from the ACS712
  int rawADC = analogRead(ACS712_PIN); // Raw ADC value
  float voltage = (rawADC / 1024.0) * 5.0; // Convert ADC value to voltage (assuming 5V ADC reference)
  float current = (voltage - (ACS712_OFFSET / 1024.0) * 5.0) / ACS712_SENSITIVITY; // Calculate current in Amperes

  // Print the calculated current to the Serial Monitor
  Serial.print("Current: ");
  Serial.print(current, 3); // Print with 3 decimal places
  Serial.println(" A");
}

// Interrupt Service Routine with Debounce Logic
void revDetectorISR() {
  unsigned long currentMicros = micros(); // Get current time (in microseconds)

  // Only process the interrupt if enough time has passed since the last one
  if (currentMicros - lastInterruptMicros >= debounceDelay) {
    isrMicros = micros(); // Record the time of the interrupt
    isrCount++;           // Increment the count
    newIsrMicros = true;  // Set the flag to indicate new data

    lastInterruptMicros = currentMicros; // Update the last interrupt time
  }
}

// Reset all variables and start over
void resetData() {
  Serial.println("No change in revDuration for 5 seconds. Resetting...");
  
  // Reset variables
  revMicros = 0;
  prevRevMicros = 0;
  revDuration = 0;
  revCount = 0;
  lastRevUpdateMillis = millis(); // Reset the timer for the next update
  
  // Optionally, you could also reset ISR counters, depending on your needs
  isrMicros = 0;
  isrCount = 0;
  newIsrMicros = false;
}
