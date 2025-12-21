/**
 * WireWinder ESP32 Firmware
 * Wire winding controller with servo control and PWM input
 * 
 * GPIO Configuration:
 * - GPIO 4: Limit switch input (with 2.3k Ohm pull-down resistor)
 * - GPIO 16: PWM input for external control
 * - GPIO 18: Servo output for winding mechanism
 * - GPIO 19: Servo output for spool fixation
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// Pin definitions
#define LIMIT_SWITCH_PIN 4
#define PWM_INPUT_PIN 16
#define WINDING_SERVO_PIN 18
#define FIXATION_SERVO_PIN 19

// Servo objects
Servo windingServo;
Servo fixationServo;

// PWM reading variables
volatile unsigned long pwmRisingTime = 0;
volatile unsigned long pwmPulseWidth = 0;
volatile bool pwmNewData = false;

// Control variables
int windingSpeed = 0;        // 0-180 degrees for servo control
bool limitSwitchPressed = false;
bool spoolFixed = false;

/**
 * Interrupt handler for PWM input
 */
void IRAM_ATTR pwmInterrupt() {
  if (digitalRead(PWM_INPUT_PIN) == HIGH) {
    // Rising edge - record time
    pwmRisingTime = micros();
  } else {
    // Falling edge - calculate pulse width
    if (pwmRisingTime > 0) {
      pwmPulseWidth = micros() - pwmRisingTime;
      pwmNewData = true;
      pwmRisingTime = 0;
    }
  }
}

/**
 * Map PWM pulse width to servo angle
 * Standard PWM: 1000-2000 microseconds -> 0-180 degrees
 */
int mapPWMToServoAngle(unsigned long pulseWidth) {
  // Constrain pulse width to valid range
  if (pulseWidth < 1000) pulseWidth = 1000;
  if (pulseWidth > 2000) pulseWidth = 2000;
  
  // Map to servo angle (0-180)
  return map(pulseWidth, 1000, 2000, 0, 180);
}

/**
 * Initialize all hardware
 */
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("WireWinder ESP32 Firmware Starting...");
  
  // Initialize limit switch as digital input
  pinMode(LIMIT_SWITCH_PIN, INPUT);
  Serial.println("Limit switch initialized on GPIO 4");
  
  // Initialize PWM input pin
  pinMode(PWM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmInterrupt, CHANGE);
  Serial.println("PWM input initialized on GPIO 16");
  
  // Initialize winding servo
  windingServo.attach(WINDING_SERVO_PIN);
  windingServo.write(90); // Start at middle position
  Serial.println("Winding servo initialized on GPIO 18");
  
  // Initialize fixation servo
  fixationServo.attach(FIXATION_SERVO_PIN);
  fixationServo.write(0); // Start unlocked position
  Serial.println("Fixation servo initialized on GPIO 19");
  
  Serial.println("System initialized successfully!");
}

/**
 * Main control loop
 */
void loop() {
  // Read limit switch state
  limitSwitchPressed = digitalRead(LIMIT_SWITCH_PIN);
  
  // Process PWM input if new data available
  if (pwmNewData) {
    pwmNewData = false;
    
    // Convert PWM pulse width to servo angle
    windingSpeed = mapPWMToServoAngle(pwmPulseWidth);
    
    Serial.print("PWM Pulse: ");
    Serial.print(pwmPulseWidth);
    Serial.print(" us -> Winding Speed: ");
    Serial.println(windingSpeed);
  }
  
  // Control logic based on limit switch
  if (limitSwitchPressed) {
    // Limit switch activated - stop winding
    Serial.println("Limit switch activated - stopping winding");
    windingServo.write(90); // Neutral position
    
    // Ensure spool is fixed when limit is reached
    if (!spoolFixed) {
      fixationServo.write(180); // Lock position
      spoolFixed = true;
      Serial.println("Spool fixation engaged");
    }
  } else {
    // Normal operation - control based on PWM input
    windingServo.write(windingSpeed);
    
    // Release spool fixation during normal operation
    if (spoolFixed) {
      fixationServo.write(0); // Unlock position
      spoolFixed = false;
      Serial.println("Spool fixation released");
    }
  }
  
  // Small delay to prevent excessive loop speed
  delay(10);
}
