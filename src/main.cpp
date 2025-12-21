#include <Arduino.h>
#include <ESP32Servo.h>

// Pin assignments
static const int PIN_LIMIT = 4;     // Endstop: to +3.3V via switch, external pulldown ~2.3 kOhm to GND
static const int PIN_INPUT1 = 16;   // PWM input 1 (winding motor control)
static const int PIN_INPUT2 = 17;   // PWM input 2 (gate open/close)
static const int PIN_SERVO1 = 19;   // Servo output 1 (winding motor)
static const int PIN_SERVO2 = 18;   // Servo output 2 (gate)

// Servo 1 (winding motor) pulse bounds in microseconds
static const uint16_t SERVO1_MIN_US = 500;
static const uint16_t SERVO1_MAX_US = 2500;
static const uint16_t SERVO1_NEUTRAL_US = 1500;

// Servo 2 (gate) pulse bounds in microseconds
static const uint16_t SERVO2_MIN_US = 1100;
static const uint16_t SERVO2_MAX_US = 1900;
static const uint16_t SERVO2_CLOSED_US = 1200;   // Gate closed position
static const uint16_t SERVO2_OPEN_US = 2000;     // Gate open position

// Control parameters
static const uint16_t INPUT_DEADZONE = 30;        // Deadzone around neutral (±30 µs)
static const uint16_t INPUT_NEUTRAL = 1500;       // Neutral position
static const uint32_t PULSE_TIMEOUT_MS = 60000;    // 48 seconds without pulses = error
static const uint32_t SERVO1_UPDATE_INTERVAL = 50; // Update servo1 every 500ms ±30ms
static const int PULSES_PER_REVOLUTION = 4;       // 4 endstop pulses = 1 revolution
static const int MAX_REVOLUTIONS = 3;            // Max revolutions in each direction
static const uint16_t SERVO1_MAX_STEP = 50;     // Max change (µs) per update interval

// System states
enum SystemState {
	STATE_IDLE,           // Waiting, motors stopped
	STATE_WINDING,        // Winding wire
	STATE_UNWINDING,      // Unwinding wire
	STATE_ERROR           // Error condition, motors stopped
};

Servo servo1;
Servo servo2;

// Global state variables
SystemState currentState = STATE_IDLE;
// pulseCount: total endstop pulses counted, range [0 .. MAX_REVOLUTIONS * PULSES_PER_REVOLUTION]
volatile unsigned long pulseCount = 0;
unsigned long lastLimitPulseTime = 0;   // Time of last endstop pulse
unsigned long lastServo1UpdateTime = 0; // Time of last servo1 update
uint16_t lastInput1Value = INPUT_NEUTRAL; // Last valid input1 value
uint16_t lastInput2Value = INPUT_NEUTRAL; // Last valid input2 value
volatile unsigned long endstopPulseCount = 0;    // Total endstop rising-edge pulses seen (updated in ISR)
volatile bool endstopPulseFlag = false; // Set by ISR when pulse seen
uint16_t currentServo1Pulse = SERVO1_NEUTRAL_US; // Last written pulse for servo1
uint16_t currentServo2Pulse = SERVO2_CLOSED_US; // Last written pulse for servo2
bool servo1PhaseSubtract = true;       // Start with subtract phase
bool allowWinding = true;
bool allowUnwinding = true;
// Remember which direction caused the last ERROR so recovery can be restricted
SystemState errorFromState = STATE_IDLE;

static inline uint16_t clampServo1(unsigned long us) {
	if (us < SERVO1_MIN_US) return SERVO1_MIN_US;
	if (us > SERVO1_MAX_US) return SERVO1_MAX_US;
	return (uint16_t)us;
}




static inline bool isInDeadzone(unsigned long pulse) {
	return (pulse >= (INPUT_NEUTRAL - INPUT_DEADZONE)) && 
	       (pulse <= (INPUT_NEUTRAL + INPUT_DEADZONE));
}

// Compute servo1 pulse: limit to ±100 µs from neutral on the side of motion
static inline uint16_t computeServo1FromInput(uint16_t input) {
	// Return the clamped input as the target; stepping logic will limit per-interval change
	if (isInDeadzone(input)) return SERVO1_NEUTRAL_US;
	return clampServo1((unsigned long)input);
}

void setup() {
	Serial.begin(115200);
	delay(100);
	Serial.println("WireWinder (ESP32Servo): starting");

	pinMode(PIN_LIMIT, INPUT_PULLDOWN);
	pinMode(PIN_INPUT1, INPUT);
	pinMode(PIN_INPUT2, INPUT);

	// Attach interrupt on rising edge for endstop to avoid missing pulses
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT), []() {
		// ISR: increment total endstop event count and set flag; update pulseCount based on direction
		endstopPulseCount++;
		endstopPulseFlag = true;
		const unsigned long MAX_PULSES = (unsigned long)MAX_REVOLUTIONS * (unsigned long)PULSES_PER_REVOLUTION;
		if (currentState == STATE_WINDING) {
			if (pulseCount < MAX_PULSES) pulseCount++;
		} else if (currentState == STATE_UNWINDING) {
			if (pulseCount > 0) pulseCount--;
		}
	}, RISING);

	// Attach servos with min/max pulse bounds
	servo1.attach(PIN_SERVO1, SERVO1_MIN_US, SERVO1_MAX_US);
	servo2.attach(PIN_SERVO2, SERVO2_MIN_US, SERVO2_MAX_US);

	// Neutral on startup
	servo1.writeMicroseconds(SERVO1_NEUTRAL_US);
	currentServo1Pulse = SERVO1_NEUTRAL_US;
	servo2.writeMicroseconds(SERVO2_CLOSED_US);
	currentServo2Pulse = SERVO2_CLOSED_US;
	
	lastServo1UpdateTime = millis();
}

void loop() {
	const unsigned long now = millis();
	
	// Read both input pulse widths
	const unsigned long pulse1 = pulseIn(PIN_INPUT1, HIGH, 25000UL);
	const unsigned long pulse2 = pulseIn(PIN_INPUT2, HIGH, 25000UL);

	// Endstop: HIGH when pressed/connected to +3.3V
	const bool limitActive = digitalRead(PIN_LIMIT) == HIGH;

	// Process any endstop pulses flagged by the ISR
	if (endstopPulseFlag) {
		endstopPulseFlag = false;
		lastLimitPulseTime = now;
		// Print summary for diagnostics (pulseCount updated in ISR)
		unsigned long revs = pulseCount / PULSES_PER_REVOLUTION;
		Serial.printf("Endstop event: %lu pulses -> %lu/%d revolutions\n", pulseCount, revs, MAX_REVOLUTIONS);
	}

	// Process INPUT1 for winding motor control
	if (pulse1 > 0) {
		lastInput1Value = pulse1;
	}

	// Determine state based on INPUT1
	SystemState newState = STATE_IDLE;
	if (!isInDeadzone(lastInput1Value)) {
		if (lastInput1Value > INPUT_NEUTRAL) {
			newState = STATE_WINDING;
		} else {
			newState = STATE_UNWINDING;
		}
	}

	// State transitions
	if (newState != currentState) {
		// If we're in ERROR state, don't accept transitions until input returns to neutral
		if (currentState == STATE_ERROR) {
			if (!isInDeadzone(lastInput1Value)) {
				// ignore newState while still in error (input not neutral)
				// keep currentState == STATE_ERROR
			} else {
				// input is neutral, clear ERROR to IDLE but keep the original error direction blocked
				Serial.println("Recovered: input neutral, clearing ERROR to IDLE (errored direction remains blocked)");
				if (errorFromState == STATE_WINDING) {
					allowWinding = false;
					allowUnwinding = true;
				} else if (errorFromState == STATE_UNWINDING) {
					allowUnwinding = false;
					allowWinding = true;
				} else {
					allowWinding = true;
					allowUnwinding = true;
				}
				currentState = STATE_IDLE;
			}
		} else {
			if (newState == STATE_IDLE) {
				Serial.println("State: IDLE");
				currentServo1Pulse = SERVO1_NEUTRAL_US;
				currentState = STATE_IDLE;
			} else if (newState == STATE_WINDING) {
				if (!allowWinding) {
					Serial.println("Transition to WINDING blocked (direction not allowed)");
					// ignore transition
				} else {
					Serial.println("State: WINDING");
					if (allowWinding) pulseCount = 0;
					lastLimitPulseTime = now;
					servo1PhaseSubtract = true;
					currentState = STATE_WINDING;
				}
			} else if (newState == STATE_UNWINDING) {
				if (!allowUnwinding) {
					Serial.println("Transition to UNWINDING blocked (direction not allowed)");
					// ignore transition
				} else {
					Serial.println("State: UNWINDING");
					if (allowUnwinding) pulseCount = 0;
					lastLimitPulseTime = now;
					servo1PhaseSubtract = true;
					currentState = STATE_UNWINDING;
				}
			}
		}
	}

	// Check for errors
	if (currentState == STATE_WINDING || currentState == STATE_UNWINDING) {
			// Check revolution limit using pulseCount
			unsigned long revs = pulseCount / PULSES_PER_REVOLUTION;
			if (revs >= (unsigned)MAX_REVOLUTIONS) {
				Serial.println("Max revolutions reached");
				// block further motion in the same direction, allow only reverse
				if (currentState == STATE_WINDING) {
					allowWinding = false;
					allowUnwinding = true;
				} else if (currentState == STATE_UNWINDING) {
					allowUnwinding = false;
					allowWinding = true;
				}
				// stop movement now (logical neutral; control loop will write)
				currentServo1Pulse = SERVO1_NEUTRAL_US;
				currentState = STATE_IDLE;
			} else {
				// under limit, allow both directions
				allowWinding = true;
				allowUnwinding = true;
			}
		
		// Check timeout (no pulses for 48 seconds)
		if (now - lastLimitPulseTime > PULSE_TIMEOUT_MS) {
			Serial.println("ERROR: No endstop pulses for 48 seconds");
			// remember which direction caused the error, block that direction on recovery
			errorFromState = currentState;
			if (errorFromState == STATE_WINDING) {
				allowWinding = false;
				allowUnwinding = true;
			} else if (errorFromState == STATE_UNWINDING) {
				allowUnwinding = false;
				allowWinding = true;
			}
			currentState = STATE_ERROR;
			// mark logical neutral; control loop will handle writes
			currentServo1Pulse = SERVO1_NEUTRAL_US;
		}
	}

	// Control SERVO1 (winding motor) - alternating subtract/add around the input
	if (currentState == STATE_WINDING || currentState == STATE_UNWINDING) {
		// If we're within 1 second of the pulse timeout, hold a fixed PWM
		unsigned long warnThreshold = (PULSE_TIMEOUT_MS > 1000) ? (PULSE_TIMEOUT_MS - 1000) : 0;
		if (now - lastLimitPulseTime >= warnThreshold && warnThreshold > 0) {
			// One second before entering ERROR: set fixed output 100 µs from neutral
			int32_t fixedPulse;
			if (currentState == STATE_WINDING) {
				fixedPulse = (int32_t)SERVO1_NEUTRAL_US + 100;
			} else {
				fixedPulse = (int32_t)SERVO1_NEUTRAL_US - 100;
			}
			if (fixedPulse < (int32_t)SERVO1_MIN_US) fixedPulse = SERVO1_MIN_US;
			if (fixedPulse > (int32_t)SERVO1_MAX_US) fixedPulse = SERVO1_MAX_US;
			uint16_t next = (uint16_t)fixedPulse;
			servo1.writeMicroseconds(next);
			currentServo1Pulse = next;
			// update the timestamp so regular stepping doesn't immediately run
			lastServo1UpdateTime = now;
			// skip further per-interval stepping while in warning window
		} else {
		// Calculate remaining full revolutions
		const unsigned long absRevolutions = pulseCount / PULSES_PER_REVOLUTION;
		const long remainingRevs = (long)MAX_REVOLUTIONS - (long)absRevolutions;
		// If only one revolution or less remains, stop changing servo output (keep currentServo1Pulse)
		if (remainingRevs <= 1) {
			// do nothing: hold currentServo1Pulse
		} else {
			if (now - lastServo1UpdateTime >= SERVO1_UPDATE_INTERVAL) {
				// Base target is the raw input clamped to servo limits
				uint16_t base = computeServo1FromInput(lastInput1Value);
				// If direction is blocked by limits, treat target as neutral so same-direction motion stops
				if (base > SERVO1_NEUTRAL_US && !allowWinding) base = SERVO1_NEUTRAL_US;
				if (base < SERVO1_NEUTRAL_US && !allowUnwinding) base = SERVO1_NEUTRAL_US;
				uint16_t next = currentServo1Pulse;
				if (base == SERVO1_NEUTRAL_US) {
					// No motion requested
					next = SERVO1_NEUTRAL_US;
				} else {
					int32_t candidate;
					if (servo1PhaseSubtract) {
						candidate = (int32_t)base - (int32_t)SERVO1_MAX_STEP;
					} else {
						candidate = (int32_t)base + (int32_t)SERVO1_MAX_STEP;
					}
					// Ensure candidate does not cross neutral and direction stays the same
					if (base > SERVO1_NEUTRAL_US) {
						// Winding side: candidate must remain > SERVO1_NEUTRAL_US
						if (candidate <= (int32_t)SERVO1_NEUTRAL_US) {
							candidate = (int32_t)SERVO1_NEUTRAL_US + 1;
						}
					} else {
						// Unwinding side: candidate must remain < SERVO1_NEUTRAL_US
						if (candidate >= (int32_t)SERVO1_NEUTRAL_US) {
							candidate = (int32_t)SERVO1_NEUTRAL_US - 1;
						}
					}
					// Clamp candidate to servo physical limits
					if (candidate < (int32_t)SERVO1_MIN_US) candidate = SERVO1_MIN_US;
					if (candidate > (int32_t)SERVO1_MAX_US) candidate = SERVO1_MAX_US;
					next = (uint16_t)candidate;
					// Toggle phase for next interval
					servo1PhaseSubtract = !servo1PhaseSubtract;
				}
				servo1.writeMicroseconds(next);
				currentServo1Pulse = next;
				lastServo1UpdateTime = now;
			}
		}
		}
	} else if (currentState == STATE_ERROR) {
		// Hold physical servo at neutral while in error and wait for input to return to neutral
		servo1.writeMicroseconds(SERVO1_NEUTRAL_US);
		currentServo1Pulse = SERVO1_NEUTRAL_US;
		// If input1 is back to neutral, clear ERROR to IDLE but keep the original direction blocked
		if (pulse1 > 0 && isInDeadzone(pulse1)) {
			Serial.println("Recovered: input neutral, clearing ERROR to IDLE (original direction still blocked)");
			// Keep only the opposite direction allowed
			if (errorFromState == STATE_WINDING) {
				allowWinding = false;
				allowUnwinding = true;
			} else if (errorFromState == STATE_UNWINDING) {
				allowUnwinding = false;
				allowWinding = true;
			} else {
				// no specific error direction recorded; allow both
				allowWinding = true;
				allowUnwinding = true;
			}
			currentState = STATE_IDLE;
			// keep errorFromState so the transition logic can enforce opposite-direction-only recovery
		}
	}

	// Track INPUT2 and control SERVO2 only when deviating from neutral (deadzone)
	if (pulse2 > 0) {
		lastInput2Value = pulse2;
	}

	// Control SERVO2 (gate) - change only when INPUT2 crosses deadzone thresholds
	{
		const uint16_t upper = INPUT_NEUTRAL + INPUT_DEADZONE;
		const uint16_t lower = INPUT_NEUTRAL - INPUT_DEADZONE;
		uint16_t target2 = currentServo2Pulse; // default: no change
		if (lastInput2Value > upper) {
			// Above neutral => closed
			target2 = SERVO2_CLOSED_US;
		} else if (lastInput2Value < lower) {
			// Below neutral => open
			target2 = SERVO2_OPEN_US;
		} else {
			// within deadzone: do not change state
		}
		if (target2 != currentServo2Pulse) {
			servo2.writeMicroseconds(target2);
			currentServo2Pulse = target2;
		}
	}

	// Diagnostics
	static unsigned long lastPrint = 0;
	if (now - lastPrint > 500) {
		// Compute full revolutions (based on pulseCount)
		unsigned long revolutions = pulseCount / PULSES_PER_REVOLUTION;
		// Compute remaining time until timeout (in seconds) only when winding/unwinding
		unsigned long remainingSec = 0;
		if (currentState == STATE_WINDING || currentState == STATE_UNWINDING) {
			long remainingMs = 0;
			if (now >= lastLimitPulseTime) {
				long elapsed = (long)(now - lastLimitPulseTime);
				long rem = (long)PULSE_TIMEOUT_MS - elapsed;
				if (rem < 0) rem = 0;
				remainingMs = rem;
			}
			remainingSec = (unsigned long)((remainingMs + 500) / 1000); // round to nearest second
		}
		Serial.printf("State:%d Input1:%uus Input2:%luus Rev:%lurevs (%lupulses) Endstop:%lu S1:%uus S2:%uus Remain:%lus\n",
				  currentState, lastInput1Value, pulse2, revolutions, pulseCount, endstopPulseCount, currentServo1Pulse, currentServo2Pulse, remainingSec);
		lastPrint = now;
	}

	
}




