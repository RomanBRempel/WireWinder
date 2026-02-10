#include <Arduino.h>
#include <ESP32Servo.h>

// Servo objects
Servo servo1;
Servo servo2;

// Pin assignments
static const int PIN_LIMIT = 4;     // Endstop: to +3.3V via switch, external pulldown ~2.3 kOhm to GND
static const int PIN_INPUT1 = 16;   // PWM input 1 (winding motor control)
static const int PIN_INPUT2 = 17;   // PWM input 2 (gate open/close)
static const int PIN_SERVO1 = 19;   // Servo output 1 (winding motor)
static const int PIN_SERVO2 = 18;   // Servo output 2 (gate)
// UART for external communication (Serial1)
static const int PIN_UART_RX = 21;  // Serial1 RX
static const int PIN_UART_TX = 22;  // Serial1 TX

// Servo 1 (winding motor) pulse bounds in microseconds
static const uint16_t SERVO1_MIN_US = 500;
static const uint16_t SERVO1_MAX_US = 2500;
static const uint16_t SERVO1_NEUTRAL_US = 1500;

// Servo 2 (gate) pulse bounds in microseconds
static const uint16_t SERVO2_MIN_US = 1100;
static const uint16_t SERVO2_MAX_US = 1900;
static const uint16_t SERVO2_NEUTRAL_US = 1540;
static const uint16_t SERVO2_CLOSED_US = 1200;   // Gate closed position
static const uint16_t SERVO2_OPEN_US = 2000;     // Gate open position

// Control parameters
static const uint16_t INPUT_DEADZONE = 30;        // Deadzone around neutral (±30 µs)
static const uint16_t INPUT_NEUTRAL = 1500;       // Neutral position
// Timeouts for endstop pulse absence (per direction)
static const uint32_t PULSE_TIMEOUT_WIND_MS = 98000;   // 90 seconds without pulses = error (winding)
static const uint32_t PULSE_TIMEOUT_UNWIND_MS = 80000; // 80 seconds without pulses = error (unwinding)
static const uint32_t SERVO1_UPDATE_INTERVAL = 30; // Update servo1 every 500ms ±30ms
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

// Global state variables
SystemState currentState = STATE_IDLE;
// pulseCount: total endstop pulses counted, range [0 .. MAX_REVOLUTIONS * PULSES_PER_REVOLUTION]
volatile unsigned long pulseCount = 0;
unsigned long lastLimitPulseTime = 0;   // Time of last endstop pulse
unsigned long lastServo1UpdateTime = 0; // Time of last servo1 update
volatile unsigned long endstopPulseCount = 0;    // Total endstop rising-edge pulses seen (updated in ISR)
volatile bool endstopPulseFlag = false; // Set by ISR when pulse seen
// Accumulated time (ms) counted toward the no-pulse timeout when user stops the process
static unsigned long accumulatedWindMs = 0;
static unsigned long accumulatedUnwindMs = 0;
// Time when current WINDING/UNWINDING active segment started (timer-based logic)
static unsigned long lastStateActiveTime = 0;
// At startup we may assume spool is fully wound; this flag prevents resetting
// `pulseCount` to zero on the first motion so unwinding will decrement from max.
static bool startupSpoolAssumedFull = false;
// Ensure startup-only timer initialization runs once
static bool initialTimersInitialized = false;
bool servo1PhaseSubtract = true;       // Start with subtract phase
bool allowWinding = true;
bool allowUnwinding = true;
// Remember which direction caused the last ERROR so recovery can be restricted
SystemState errorFromState = STATE_IDLE;
// Remember last active direction before going to IDLE so accumulated timers
// survive a stop-and-start and can be transferred when direction changes.
static SystemState lastActiveDirection = STATE_IDLE;

// Runtime tracked values for servos and last inputs
static uint16_t currentServo1Pulse = SERVO1_NEUTRAL_US;
static uint16_t currentServo2Pulse = SERVO2_CLOSED_US;
static unsigned long lastInput1Value = INPUT_NEUTRAL;
static unsigned long lastInput2Value = INPUT_NEUTRAL;

// Debug printing helpers
#include <stdarg.h>
static unsigned long startupBlockUntil = 0; // kept for compatibility (0 = no block)
static void dbgPrintf(const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	char buf[256];
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	Serial.print(buf);
}

static void dbgPrintln(const char *s) {
	Serial.println(s);
}

// Arming / command-waiting: require channel1 to send a max PWM then neutral
static bool waitingForArm = true;
static bool seenArmMax = false;
static const uint16_t ARM_MAX_THRESHOLD = 1900; // treat >= this as 'maximum' command
static const uint16_t ARM_MIN_ACCEPT = 500;
static const uint16_t ARM_MAX_ACCEPT = 2500;

// LED indicator (built-in)
static const int LED_PIN = 2; // onboard LED on most ESP32 dev boards
enum LedMode { LED_OFF=0, LED_SLOW, LED_FAST, LED_ON };
static LedMode ledMode = LED_OFF;
static bool ledState = false;
static unsigned long lastLedToggle = 0;

// Non-blocking startup servo timing (0 = none)
static unsigned long startupServo1Until = 0;
// Diagnostics timer while waiting for arm
static unsigned long lastArmDiag = 0;

static inline uint16_t clampServo1(unsigned long us) {
	if (us < SERVO1_MIN_US) return SERVO1_MIN_US;
	if (us > SERVO1_MAX_US) return SERVO1_MAX_US;
	return (uint16_t)us;
}




static inline bool isInDeadzone(unsigned long pulse) {
	return (pulse >= (INPUT_NEUTRAL - INPUT_DEADZONE)) && 
	       (pulse <= (INPUT_NEUTRAL + INPUT_DEADZONE));
}

// Generic pulse accept/filter function for ranges
static inline bool pulseInAcceptRange(unsigned long pulse, unsigned long minVal, unsigned long maxVal) {
	return (pulse >= minVal) && (pulse <= maxVal);
}

// Compute servo1 pulse: limit to ±100 µs from neutral on the side of motion
static inline uint16_t computeServo1FromInput(uint16_t input) {
	// Return the clamped input as the target; stepping logic will limit per-interval change
	if (isInDeadzone(input)) return SERVO1_NEUTRAL_US;
	return clampServo1((unsigned long)input);
}

// Set servo1 to neutral with a small 'nudge' away then back.
// Sequence: neutral -> offset (30 µs toward previous value) -> neutral.
static void setServo1NeutralSequence() {
	const uint16_t neutral = SERVO1_NEUTRAL_US;
	// write neutral first
	servo1.writeMicroseconds(neutral);
	// determine direction based on previous/current pulse
	uint16_t prev = currentServo1Pulse;
	const uint16_t offset = 30;
	uint16_t shifted;
	if (prev > neutral) {
		shifted = clampServo1((unsigned long)neutral + offset);
	} else if (prev < neutral) {
		shifted = clampServo1((unsigned long)neutral - offset);
	} else {
		// if equal, nudge upward
		shifted = clampServo1((unsigned long)neutral + offset);
	}
	
	servo1.writeMicroseconds(shifted);
	
	servo1.writeMicroseconds(neutral);
	currentServo1Pulse = neutral;
}

void setup() {
	Serial.begin(115200);
	delay(100);
	// No startup delay for serial output
	startupBlockUntil = 0;
	dbgPrintln("WireWinder (ESP32Servo): starting");

	// Initialize hardware UART (Serial1) on pins RX=21, TX=22
	Serial1.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
	dbgPrintln("Serial1 initialized on RX=21 TX=22 (115200)");

	pinMode(PIN_LIMIT, INPUT_PULLDOWN);
	pinMode(PIN_INPUT1, INPUT);
	pinMode(PIN_INPUT2, INPUT);
	// LED pin
	pinMode(LED_PIN, OUTPUT);

	// Attach interrupt on rising edge for endstop to avoid missing pulses
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT), []() {
		// ISR: increment total endstop event count and set flag; update pulseCount based on direction
		// Only record that a pulse happened; do NOT modify `pulseCount` when
		// operating in timer-only mode (endstop ignored for logic).
		endstopPulseCount++;
		endstopPulseFlag = true;
	}, RISING);

	// Attach servos with min/max pulse bounds
	servo1.attach(PIN_SERVO1, SERVO1_MIN_US, SERVO1_MAX_US);
	servo2.attach(PIN_SERVO2, SERVO2_MIN_US, SERVO2_MAX_US);

	// Startup sequence: briefly drive servo1 to 1300 µs, then return to neutral (non-blocking)
	servo1.writeMicroseconds(1450);
	servo1.writeMicroseconds(SERVO1_NEUTRAL_US);

	servo2.writeMicroseconds(SERVO2_CLOSED_US);
	currentServo2Pulse = SERVO2_CLOSED_US;
    
	// Start with spool considered fully wound: set pulseCount to maximum
	pulseCount = (unsigned long)MAX_REVOLUTIONS * (unsigned long)PULSES_PER_REVOLUTION;
	// Block further winding at startup, allow only unwinding
	allowWinding = false;
	allowUnwinding = true;
	startupSpoolAssumedFull = true;
	dbgPrintf("Startup: spool assumed fully wound (%lu pulses)\n", pulseCount);

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
		// We intentionally IGNORE endstop pulses for control logic; keep the
		// diagnostic counter and clear the startup assumption so pulseCount
		// won't be forcibly left at startup max.
		unsigned long revs = pulseCount / PULSES_PER_REVOLUTION;
		dbgPrintf("Endstop event (ignored for control): %lu pulses -> %lu/%d revolutions\n", pulseCount, revs, MAX_REVOLUTIONS);
		startupSpoolAssumedFull = false;
	}

	// Handle non-blocking startup servo1 restore
	if (startupServo1Until != 0 && now >= startupServo1Until) {
		setServo1NeutralSequence();
		startupServo1Until = 0;
	}

	// LED mode selection
	LedMode targetLedMode = LED_OFF;
	if (waitingForArm) {
		targetLedMode = LED_SLOW;
	} else if (currentState == STATE_ERROR) {
		targetLedMode = LED_FAST;
	} else if (!waitingForArm && currentState == STATE_IDLE) {
		targetLedMode = LED_ON;
	} else {
		targetLedMode = LED_OFF;
	}
	// apply mode change
	if (targetLedMode != ledMode) {
		ledMode = targetLedMode;
		ledState = false;
		digitalWrite(LED_PIN, LOW);
		lastLedToggle = now;
	}
	// update LED according to mode (non-blocking)
	if (ledMode == LED_SLOW) {
		const unsigned long interval = 500UL;
		if (now - lastLedToggle >= interval) {
			ledState = !ledState;
			digitalWrite(LED_PIN, ledState ? HIGH : LOW);
			lastLedToggle = now;
		}
	} else if (ledMode == LED_FAST) {
		const unsigned long interval = 100UL;
		if (now - lastLedToggle >= interval) {
			ledState = !ledState;
			digitalWrite(LED_PIN, ledState ? HIGH : LOW);
			lastLedToggle = now;
		}
	} else if (ledMode == LED_ON) {
		if (!ledState) {
			ledState = true;
			digitalWrite(LED_PIN, HIGH);
		}
	} else { // LED_OFF
		if (ledState) {
			ledState = false;
			digitalWrite(LED_PIN, LOW);
		}
	}

	// Arming sequence: require a MAX on channel1 then a neutral to enable inputs
	if (waitingForArm) {
		// Only consider pulses in arm-accept range
		if (pulseInAcceptRange(pulse1, ARM_MIN_ACCEPT, ARM_MAX_ACCEPT)) {
			// detect max pulse (allow pulses >= ARM_MAX_THRESHOLD)
			if (pulse1 >= ARM_MAX_THRESHOLD) {
				seenArmMax = true;
				dbgPrintln("Arm: saw max pulse on channel1");
			}
			// if we've seen max and now neutral, finish arming
			if (seenArmMax && isInDeadzone(pulse1)) {
				waitingForArm = false;
				dbgPrintln("Armed: channel1 max->neutral sequence received");
			}
		} else {
			// out-of-range pulses are ignored for arming
		}
		// print diagnostics about inputs while waiting for arm (rate-limited)
		if (now - lastArmDiag >= 500UL) {
			dbgPrintf("ARM WAIT DIAG: pulse1:%uus pulse2:%uus seenMax:%d\n", pulse1, pulse2, seenArmMax ? 1 : 0);
			lastArmDiag = now;
		}
		// while waiting, hold servos safe and ignore other inputs
		setServo1NeutralSequence();
		servo2.writeMicroseconds(SERVO2_CLOSED_US);
		currentServo2Pulse = SERVO2_CLOSED_US;
		// skip normal control until armed
		return;
	}

	// Process INPUT1 for winding motor control (only when armed)
	// Accept only pulses in valid control range [800..2000]
	if (pulseInAcceptRange(pulse1, 800, 2000)) {
		lastInput1Value = pulse1;
	}

	// Determine state based on INPUT1
	SystemState newState = STATE_IDLE;
	if (!isInDeadzone(lastInput1Value)) {
		if (lastInput1Value > INPUT_NEUTRAL) {
			newState = STATE_UNWINDING;
		} else {
			newState = STATE_WINDING;
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
					dbgPrintln("Recovered: input neutral, clearing ERROR to IDLE (errored direction remains blocked)");
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
					dbgPrintln("State: IDLE");
				// remember which direction we just came from
				lastActiveDirection = currentState;
				// mark no active segment
				lastStateActiveTime = 0;
				setServo1NeutralSequence();
				currentState = STATE_IDLE;
			} else if (newState == STATE_WINDING) {
				if (!allowWinding) {
						dbgPrintln("Transition to WINDING blocked (direction not allowed)");
					// ignore transition
				} else {
					// If starting WINDING from IDLE (cold start), initialize timers:
					// wind elapsed = 0 (full remaining), unwind elapsed = full timeout (zero remaining)
					if (startupSpoolAssumedFull && !initialTimersInitialized) {
						accumulatedWindMs = 0;
						accumulatedUnwindMs = PULSE_TIMEOUT_UNWIND_MS;
						dbgPrintf("START: WIND accumWind:%lums accumUnw:%lums\n", accumulatedWindMs, accumulatedUnwindMs);
						initialTimersInitialized = true;
						startupSpoolAssumedFull = false;
					}
					dbgPrintln("State: WINDING");
					// If switching from the opposite direction, account for time spent
					// in that previous direction toward the new direction's timeout.
					if (currentState == STATE_UNWINDING) {
						// We were unwinding and now start winding: add the active segment
						// to the *unwind* accumulator (it represents time spent unwinding).
						unsigned long seg = 0;
						if (lastStateActiveTime != 0 && now >= lastStateActiveTime) seg = now - lastStateActiveTime;
						unsigned long beforeUnw = accumulatedUnwindMs;
						accumulatedUnwindMs += seg;
						if (accumulatedUnwindMs > PULSE_TIMEOUT_UNWIND_MS) accumulatedUnwindMs = PULSE_TIMEOUT_UNWIND_MS;
						dbgPrintf("TRANSFER: UNW->W: beforeUnw:%lums seg:%lums afterUnw:%lums\n", beforeUnw, seg, accumulatedUnwindMs);
						// Compute desired remaining winding time = priorUnwind + scaled diff
						unsigned long diff = (PULSE_TIMEOUT_WIND_MS > PULSE_TIMEOUT_UNWIND_MS) ?
							(PULSE_TIMEOUT_WIND_MS - PULSE_TIMEOUT_UNWIND_MS) :
							(PULSE_TIMEOUT_UNWIND_MS - PULSE_TIMEOUT_WIND_MS);
						unsigned long scaled = 0;
						if (accumulatedUnwindMs > 0 && PULSE_TIMEOUT_UNWIND_MS > 0) {
							unsigned long long prod = (unsigned long long)diff * (unsigned long long)accumulatedUnwindMs;
							scaled = (unsigned long)(prod / (unsigned long long)PULSE_TIMEOUT_UNWIND_MS);
						}
						unsigned long desiredRemain = accumulatedUnwindMs + scaled;
						// Set accumulatedWindMs so that remaining = desiredRemain
						if (desiredRemain >= PULSE_TIMEOUT_WIND_MS) accumulatedWindMs = 0;
						else accumulatedWindMs = PULSE_TIMEOUT_WIND_MS - desiredRemain;
						dbgPrintf("TRANSFER: UNW->W: desiredRemain:%lums set accumulatedWindMs:%lums\n", desiredRemain, accumulatedWindMs);
					} else if (lastActiveDirection == STATE_UNWINDING) {
						// Previously stopped while unwinding and now starting winding after IDLE.
						// Do not modify accumulators on stop->start; keep previously accumulated values.
						dbgPrintf("TRANSFER(stop)->W: previous was UNW, no accumulator change\n");
						lastActiveDirection = STATE_IDLE;
					}
					if (allowWinding && !startupSpoolAssumedFull) pulseCount = 0;
					lastStateActiveTime = now;
					servo1PhaseSubtract = true;
					currentState = STATE_WINDING;
				}
			} else if (newState == STATE_UNWINDING) {
				if (!allowUnwinding) {
						dbgPrintln("Transition to UNWINDING blocked (direction not allowed)");
					// ignore transition
				} else {
					// If starting UNWINDING from IDLE (cold start), initialize timers:
					// unwind elapsed = 0 (full remaining), wind elapsed = full timeout (zero remaining)
					if (startupSpoolAssumedFull && !initialTimersInitialized) {
						accumulatedUnwindMs = 0;
						accumulatedWindMs = PULSE_TIMEOUT_WIND_MS;
						dbgPrintf("START: UNW accumUnw:%lums accumWind:%lums\n", accumulatedUnwindMs, accumulatedWindMs);
						initialTimersInitialized = true;
						startupSpoolAssumedFull = false;
					}
					dbgPrintln("State: UNWINDING");
					// If switching from the opposite direction, account for time spent
					// in that previous direction toward the new direction's timeout.
					if (currentState == STATE_WINDING) {
						unsigned long seg = 0;
						if (lastStateActiveTime != 0 && now >= lastStateActiveTime) seg = now - lastStateActiveTime;
						accumulatedWindMs += seg;
						if (accumulatedWindMs > PULSE_TIMEOUT_WIND_MS) accumulatedWindMs = PULSE_TIMEOUT_WIND_MS;
						// Compute desired remaining unwind time = windAccum + diff scaled by proportion
						unsigned long diff = (PULSE_TIMEOUT_WIND_MS > PULSE_TIMEOUT_UNWIND_MS) ?
							(PULSE_TIMEOUT_WIND_MS - PULSE_TIMEOUT_UNWIND_MS) :
							(PULSE_TIMEOUT_UNWIND_MS - PULSE_TIMEOUT_WIND_MS);
						unsigned long scaled = 0;
						if (accumulatedWindMs > 0 && PULSE_TIMEOUT_WIND_MS > 0) {
							unsigned long long prod = (unsigned long long)diff * (unsigned long long)accumulatedWindMs;
							scaled = (unsigned long)(prod / (unsigned long long)PULSE_TIMEOUT_WIND_MS);
						}
						unsigned long desiredRemain = accumulatedWindMs + scaled;
						if (desiredRemain >= PULSE_TIMEOUT_UNWIND_MS) accumulatedUnwindMs = 0;
						else accumulatedUnwindMs = PULSE_TIMEOUT_UNWIND_MS - desiredRemain;
						dbgPrintf("TRANSFER: W->UNW: desiredRemain:%lums set accumulatedUnwindMs:%lums\n", desiredRemain, accumulatedUnwindMs);
					} else if (lastActiveDirection == STATE_WINDING) {
						// Previously stopped while winding and now starting unwinding after IDLE.
						// Do not modify accumulators on stop->start; keep previously accumulated values.
						dbgPrintf("TRANSFER(stop)->UNW: previous was WIND, no accumulator change\n");
						lastActiveDirection = STATE_IDLE;
					}
					if (allowUnwinding && !startupSpoolAssumedFull) pulseCount = 0;
					lastStateActiveTime = now;
					servo1PhaseSubtract = true;
					currentState = STATE_UNWINDING;
				}
			}
		}
	}

	// Update accumulators while active and check for errors
	if (currentState == STATE_WINDING || currentState == STATE_UNWINDING) {
		// compute elapsed since last active sample and apply to both accumulators
		if (lastStateActiveTime == 0) {
			// just entered active state; initialize timestamp
			lastStateActiveTime = now;
		} else if (now >= lastStateActiveTime) {
			unsigned long seg = now - lastStateActiveTime;
			if (seg > 0) {
				if (currentState == STATE_WINDING) {
					// increase wind elapsed (reduces wind remaining)
					accumulatedWindMs += seg;
					if (accumulatedWindMs > PULSE_TIMEOUT_WIND_MS) accumulatedWindMs = PULSE_TIMEOUT_WIND_MS;
					// decrease unwind elapsed proportionally so unwind remaining increases
					unsigned long reduce = (unsigned long)((unsigned long long)seg * (unsigned long long)PULSE_TIMEOUT_UNWIND_MS / (unsigned long long)PULSE_TIMEOUT_WIND_MS);
					if (reduce >= accumulatedUnwindMs) accumulatedUnwindMs = 0;
					else accumulatedUnwindMs -= reduce;
				} else {
					// UNWINDING: increase unwind elapsed, decrease wind elapsed proportionally
					accumulatedUnwindMs += seg;
					if (accumulatedUnwindMs > PULSE_TIMEOUT_UNWIND_MS) accumulatedUnwindMs = PULSE_TIMEOUT_UNWIND_MS;
					unsigned long reduce = (unsigned long)((unsigned long long)seg * (unsigned long long)PULSE_TIMEOUT_WIND_MS / (unsigned long long)PULSE_TIMEOUT_UNWIND_MS);
					if (reduce >= accumulatedWindMs) accumulatedWindMs = 0;
					else accumulatedWindMs -= reduce;
				}
				// advance sample time
				lastStateActiveTime = now;
			}
		}
			// Update allow flags based on remaining times (do not allow direction if remaining == 0)
			unsigned long remWind = (accumulatedWindMs >= PULSE_TIMEOUT_WIND_MS) ? 0 : (PULSE_TIMEOUT_WIND_MS - accumulatedWindMs);
			unsigned long remUnw = (accumulatedUnwindMs >= PULSE_TIMEOUT_UNWIND_MS) ? 0 : (PULSE_TIMEOUT_UNWIND_MS - accumulatedUnwindMs);
			allowWinding = (remWind > 0);
			allowUnwinding = (remUnw > 0);
		
		// Check timeout (no pulses for configured timeout per direction).
		uint32_t timeoutMs = (currentState == STATE_WINDING) ? PULSE_TIMEOUT_WIND_MS : PULSE_TIMEOUT_UNWIND_MS;
		unsigned long totalElapsed = (currentState == STATE_WINDING) ? accumulatedWindMs : accumulatedUnwindMs;
		if (totalElapsed > timeoutMs) {
			dbgPrintf("ERROR: No endstop pulses for %lu seconds\n", timeoutMs / 1000);
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
			// mark logical neutral; perform neutral sequence now
			setServo1NeutralSequence();
		}
	}

	// Control SERVO1 (winding motor) - alternating subtract/add around the input
	if (currentState == STATE_WINDING || currentState == STATE_UNWINDING) {
		// If remaining timeout reached zero, force neutral immediately
		uint32_t timeoutMs = (currentState == STATE_WINDING) ? PULSE_TIMEOUT_WIND_MS : PULSE_TIMEOUT_UNWIND_MS;
		// Compute total elapsed toward timeout (accumulated + current segment)
		unsigned long segElapsed = 0;
		if (lastStateActiveTime != 0 && now >= lastStateActiveTime) segElapsed = now - lastStateActiveTime;
		unsigned long totalElapsed = ((currentState == STATE_WINDING) ? accumulatedWindMs : accumulatedUnwindMs) + segElapsed;
		if (totalElapsed >= timeoutMs) {
			// timeout reached: force neutral immediately
			setServo1NeutralSequence();
			currentServo1Pulse = SERVO1_NEUTRAL_US;
			lastServo1UpdateTime = now;
		} else {
		// warning when within 1 second of timeout
		unsigned long warnThresholdMs = (timeoutMs > 1000) ? (timeoutMs - 1000) : 0;
		if (totalElapsed >= warnThresholdMs && warnThresholdMs > 0) {
			// One second before entering ERROR: set fixed output 100 µs from neutral
			int32_t fixedPulse;
			if (currentState == STATE_WINDING) {
				fixedPulse = (int32_t)SERVO1_NEUTRAL_US - 50;
			} else {
				fixedPulse = (int32_t)SERVO1_NEUTRAL_US + 50;
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
		// Calculate remaining full revolutions and per-direction proximity
		const unsigned long absRevolutions = pulseCount / PULSES_PER_REVOLUTION;
		const long remainingToMax = (long)MAX_REVOLUTIONS - (long)absRevolutions;
		const long remainingToZero = (long)absRevolutions;
		// Compute remaining time for both directions (include active segment)
		unsigned long segElapsedForHold = 0;
		if (lastStateActiveTime != 0 && now >= lastStateActiveTime) segElapsedForHold = now - lastStateActiveTime;
		unsigned long totalElapsedWindForHold = accumulatedWindMs + ((currentState == STATE_WINDING) ? segElapsedForHold : 0);
		unsigned long totalElapsedUnwForHold = accumulatedUnwindMs + ((currentState == STATE_UNWINDING) ? segElapsedForHold : 0);
		unsigned long remWindMsForHold = (totalElapsedWindForHold >= PULSE_TIMEOUT_WIND_MS) ? 0 : (PULSE_TIMEOUT_WIND_MS - totalElapsedWindForHold);
		unsigned long remUnwMsForHold = (totalElapsedUnwForHold >= PULSE_TIMEOUT_UNWIND_MS) ? 0 : (PULSE_TIMEOUT_UNWIND_MS - totalElapsedUnwForHold);
		// If endstop/rev counting is unavailable (timer-only), prefer timer-based blocking:
		bool holdServo = false;
		if (!startupSpoolAssumedFull) {
			if (currentState == STATE_WINDING) {
				if (remainingToMax <= 1 && remWindMsForHold == 0) holdServo = true;
			} else if (currentState == STATE_UNWINDING) {
				if (remainingToZero <= 1 && remUnwMsForHold == 0) holdServo = true;
			}
		} else {
			// If we started assumed full, allow WINDING to move, but still prevent UNWINDING
			if (currentState == STATE_UNWINDING) {
				if (remainingToZero <= 1 && remUnwMsForHold == 0) holdServo = true;
			}
		}
		if (holdServo) {
			// do nothing: hold currentServo1Pulse
		} else {
			if (now - lastServo1UpdateTime >= SERVO1_UPDATE_INTERVAL) {
				// Base target is the raw input clamped to servo limits
				uint16_t base = computeServo1FromInput(lastInput1Value);
				// If remaining timeout for the active direction reached zero, force neutral
				{
					unsigned long segForBase = 0;
					if (lastStateActiveTime != 0 && now >= lastStateActiveTime) segForBase = now - lastStateActiveTime;
					if (currentState == STATE_WINDING) {
						unsigned long totalElapsedWind = accumulatedWindMs + segForBase;
						if (totalElapsedWind >= PULSE_TIMEOUT_WIND_MS) base = SERVO1_NEUTRAL_US;
					} else if (currentState == STATE_UNWINDING) {
						unsigned long totalElapsedUnw = accumulatedUnwindMs + segForBase;
						if (totalElapsedUnw >= PULSE_TIMEOUT_UNWIND_MS) base = SERVO1_NEUTRAL_US;
					}
				}
					// If direction is blocked by limits, treat target as neutral so same-direction motion stops
					// NOTE: `base > NEUTRAL` corresponds to UNWINDING, `base < NEUTRAL` to WINDING.
					if (base > SERVO1_NEUTRAL_US && !allowUnwinding) base = SERVO1_NEUTRAL_US;
					if (base < SERVO1_NEUTRAL_US && !allowWinding) base = SERVO1_NEUTRAL_US;
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
				if (next == SERVO1_NEUTRAL_US) {
					setServo1NeutralSequence();
				}
				currentServo1Pulse = next;
				lastServo1UpdateTime = now;
				}
			}
			}
			}
		} else if (currentState == STATE_ERROR) {
		// Hold physical servo at neutral while in error and wait for input to return to neutral
		setServo1NeutralSequence();
		// If input1 is back to neutral, clear ERROR to IDLE but keep the original direction blocked
		if (pulseInAcceptRange(pulse1, 800, 2000) && isInDeadzone(pulse1)) {
						dbgPrintln("Recovered: input neutral, clearing ERROR to IDLE (original direction still blocked)");
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
	// Accept only pulses in valid control range [800..2000]
	if (pulseInAcceptRange(pulse2, 800, 2000)) {
		lastInput2Value = pulse2;
	}

	// Control SERVO2 (gate) - change only when INPUT2 crosses deadzone thresholds
	{
		const uint16_t upper = INPUT_NEUTRAL + INPUT_DEADZONE;
		const uint16_t lower = INPUT_NEUTRAL - INPUT_DEADZONE;
		uint16_t target2 = currentServo2Pulse; // default: no change
		if (lastInput2Value > upper) {
			// Above neutral => closed
			target2 = SERVO2_OPEN_US;
		} else if (lastInput2Value < lower) {
			// Below neutral => open
			target2 = SERVO2_CLOSED_US;
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
		// Compute remaining time until timeout (in seconds) for both winding and unwinding
		unsigned long remainingWindSec = 0;
		unsigned long remainingUnwindSec = 0;
		{
			unsigned long segWind = 0;
			unsigned long segUnw = 0;
			if (currentState == STATE_WINDING && lastStateActiveTime != 0 && now >= lastStateActiveTime) segWind = now - lastStateActiveTime;
			if (currentState == STATE_UNWINDING && lastStateActiveTime != 0 && now >= lastStateActiveTime) segUnw = now - lastStateActiveTime;
			unsigned long totalElapsedWind = accumulatedWindMs + segWind;
			unsigned long totalElapsedUnwind = accumulatedUnwindMs + segUnw;
			unsigned long remWindMs = (totalElapsedWind >= PULSE_TIMEOUT_WIND_MS) ? 0 : (PULSE_TIMEOUT_WIND_MS - totalElapsedWind);
			unsigned long remUnwMs = (totalElapsedUnwind >= PULSE_TIMEOUT_UNWIND_MS) ? 0 : (PULSE_TIMEOUT_UNWIND_MS - totalElapsedUnwind);
			remainingWindSec = (remWindMs + 500) / 1000;
			remainingUnwindSec = (remUnwMs + 500) / 1000;
		}
		dbgPrintf("State:%d Input1:%uus Input2:%luus Rev:%lurevs (%lupulses) Endstop:%lu S1:%uus S2:%uus RemW:%lus RemU:%lus\n",
				  currentState, lastInput1Value, pulse2, revolutions, pulseCount, endstopPulseCount, currentServo1Pulse, currentServo2Pulse, remainingWindSec, remainingUnwindSec);
		lastPrint = now;
	}
}




