#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <FastAccelStepper.h>

// WiFi credentials
const char* WIFI_SSID = "RBR_WiFi";
const char* WIFI_PASSWORD = "87770759";

// Web server
WebServer server(80);

// Preferences for storing settings in non-volatile memory
Preferences preferences;

// Web control variables
enum ControlMode { CONTROL_PWM, CONTROL_WEB };
ControlMode controlMode = CONTROL_PWM;
unsigned long webCommandInput1 = 1500; // Neutral position
unsigned long webCommandInput2 = 1500; // Neutral position
unsigned long lastWebCommandTime = 0;
const unsigned long WEB_COMMAND_TIMEOUT = 5000; // 5 seconds timeout
bool testMode = false; // Test mode: ignore revolution limits

// Servo object (only for gate control)
Servo servo2;

// TMC2209 driver in standalone mode (no UART)
// All configuration done via hardware pins (Vref, MS1/MS2, etc)

// Pin assignments
static const int PIN_INPUT1 = 16;   // PWM input 1 (winding motor control)
static const int PIN_INPUT2 = 17;   // PWM input 2 (gate open/close)
// TMC2209 stepper pins
static const int PIN_STEP = 25;     // STEP pin for TMC2209
static const int PIN_DIR = 26;      // DIR pin for TMC2209
static const int PIN_EN = 27;        // ENABLE pin for TMC2209 (active LOW)
// Servo for gate
static const int PIN_SERVO2 = 18;   // Servo output 2 (gate)
// UART for external communication (Serial1)
static const int PIN_UART_RX = 21;  // Serial1 RX
static const int PIN_UART_TX = 22;  // Serial1 TX

// Stepper motor parameters
static const uint32_t STEPPER_MAX_SPEED = 8000;    // Maximum speed in steps/sec
static const uint32_t STEPPER_ACCEL = 1000;        // Acceleration in steps/sec²
static const uint16_t STEPPER_MICROSTEPS = 4;     // Microstepping setting
static const uint16_t STEPPER_RMS_CURRENT = 800;   // RMS current in mA
static const int32_t STEPS_PER_REV = 200 * STEPPER_MICROSTEPS; // 200 full steps/rev * microsteps
static const float STEPPER_GEAR_RATIO = 27.1f; // Motor turns per output revolution
static const float STEPS_PER_OUTPUT_REV = (float)STEPS_PER_REV * STEPPER_GEAR_RATIO;

// User-configurable servo positions (loaded from non-volatile memory at startup)
uint16_t servo2ClosedPos = 1100;   // Gate closed position (default, will be overridden)
uint16_t servo2OpenPos = 1900;     // Gate open position (default, will be overridden)
// Minimum difference between open/closed positions to ensure meaningful servo movement
// 50µs provides reliable operation for most servo models while allowing flexibility
static const uint16_t MIN_SERVO_DIFF = 50; // Minimum difference between open/closed positions (µs)

// Control parameters
static const uint16_t INPUT_DEADZONE = 30;        // Deadzone around neutral (±30 µs)
static const uint16_t INPUT_NEUTRAL = 1500;       // Neutral position
// Web-only gate command pulses (independent of servo2 open/closed positions)
static const uint16_t WEB_GATE_OPEN_CMD = INPUT_NEUTRAL + INPUT_DEADZONE + 50;
static const uint16_t WEB_GATE_CLOSE_CMD = INPUT_NEUTRAL - INPUT_DEADZONE - 50;
// Timeouts per direction
static const uint32_t PULSE_TIMEOUT_WIND_MS = 98000;   // 98 seconds timeout (winding)
static const uint32_t PULSE_TIMEOUT_UNWIND_MS = 80000; // 80 seconds timeout (unwinding)
static const uint32_t STEPPER_UPDATE_INTERVAL = 50; // Update stepper speed every 50ms
// User-configurable max revolutions (loaded from non-volatile memory at startup)
float maxRevolutions = 3.0f;            // Max revolutions in each direction (default, will be overridden)

// System states
enum SystemState {
	STATE_IDLE,           // Waiting, motors stopped
	STATE_WINDING,        // Winding wire
	STATE_UNWINDING,      // Unwinding wire
	STATE_ERROR           // Error condition, motors stopped
};

// Global state variables
SystemState currentState = STATE_IDLE;
// Revolution counter: positive = wound, negative = unwound
float revolutionCounter = 0.0f;  // Current position in revolutions
unsigned long lastStepperUpdateTime = 0; // Time of last stepper update
// Accumulated time (ms) counted toward the no-pulse timeout when user stops the process
static unsigned long accumulatedWindMs = 0;
static unsigned long accumulatedUnwindMs = 0;
// Time when current WINDING/UNWINDING active segment started (timer-based logic)
static unsigned long lastStateActiveTime = 0;
// Ensure startup-only timer initialization runs once
static bool initialTimersInitialized = false;
bool allowWinding = true;
bool allowUnwinding = true;
// Remember which direction caused the last ERROR so recovery can be restricted
SystemState errorFromState = STATE_IDLE;
// Remember last active direction before going to IDLE so accumulated timers
// survive a stop-and-start and can be transferred when direction changes.
static SystemState lastActiveDirection = STATE_IDLE;

// Runtime tracked values for stepper and servo2 and last inputs
static int32_t currentStepperSpeed = 0;  // Current stepper speed in steps/sec
static bool stepperEnabled = false;       // Stepper enable state
static uint16_t currentServo2Pulse = 1200; // Initially closed
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

// FastAccelStepper engine and stepper instance
static FastAccelStepperEngine stepperEngine;
static FastAccelStepper *stepper = nullptr;

static inline bool isInDeadzone(unsigned long pulse) {
	return (pulse >= (INPUT_NEUTRAL - INPUT_DEADZONE)) &&
	       (pulse <= (INPUT_NEUTRAL + INPUT_DEADZONE));
}

// Generic pulse accept/filter function for ranges
static inline bool pulseInAcceptRange(unsigned long pulse, unsigned long minVal, unsigned long maxVal) {
	return (pulse >= minVal) && (pulse <= maxVal);
}

// Compute stepper speed from input PWM (returns speed in steps/sec)
static inline int32_t computeStepperSpeedFromInput(uint16_t input) {
	if (isInDeadzone(input)) return 0;

	// Always run at max speed when outside deadzone
	return (input > INPUT_NEUTRAL) ? (int32_t)STEPPER_MAX_SPEED : -(int32_t)STEPPER_MAX_SPEED;
}

// Enable/disable stepper motor outputs
static void setStepperEnable(bool enable) {
	if (!stepper) {
		stepperEnabled = false;
		currentStepperSpeed = 0;
		return;
	}

	if (enable) {
		stepper->enableOutputs();
		stepperEnabled = true;
		return;
	}

	stepper->stopMove();
	stepperEnabled = false;
	currentStepperSpeed = 0;
}

// Stop motion but keep holding torque
static void setStepperHold() {
	if (!stepper) {
		stepperEnabled = false;
		currentStepperSpeed = 0;
		return;
	}
	stepper->stopMove();
	stepper->enableOutputs();
	stepperEnabled = false;
	currentStepperSpeed = 0;
}

// Update stepper speed (called periodically)
static void updateStepperSpeed(int32_t targetSpeed) {
	if (!stepper) {
		return;
	}

	if (targetSpeed == 0) {
		setStepperHold();
		return;
	}

	if (!stepperEnabled) {
		setStepperEnable(true);
	}

	uint32_t absSpeed = (uint32_t)abs(targetSpeed);
	stepper->setSpeedInHz(absSpeed);
	if (targetSpeed > 0) {
		stepper->runForward();
	} else {
		stepper->runBackward();
	}

	currentStepperSpeed = targetSpeed;
}

// Web server handlers
void handleRoot() {
	String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
	<meta charset="UTF-8">
	<meta name="viewport" content="width=device-width, initial-scale=1.0">
	<title>WireWinder Status</title>
	<style>
		body { font-family: Arial, sans-serif; margin: 20px; background: #1a1a1a; color: #e0e0e0; }
		h1 { color: #4CAF50; }
		.status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; }
		.status-card { background: #2a2a2a; padding: 15px; border-radius: 8px; border-left: 4px solid #4CAF50; }
		.status-card h3 { margin-top: 0; color: #4CAF50; }
		.status-value { font-size: 1.5em; font-weight: bold; color: #fff; }
		.status-label { color: #aaa; font-size: 0.9em; margin-top: 5px; }
		.state-idle { border-left-color: #2196F3; }
		.state-winding { border-left-color: #FF9800; }
		.state-unwinding { border-left-color: #9C27B0; }
		.state-error { border-left-color: #f44336; }
		.arm-waiting { border-left-color: #FFC107; }
		.arm-ready { border-left-color: #4CAF50; }
		.connection-status { padding: 10px; border-radius: 5px; margin-bottom: 20px; text-align: center; }
		.connected { background: #4CAF50; color: white; }
		.loading { background: #FF9800; color: white; }
		.control-panel { background: #2a2a2a; padding: 20px; border-radius: 8px; margin-bottom: 20px; }
		.control-panel h2 { margin-top: 0; color: #4CAF50; }
		.button-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; margin-bottom: 15px; }
		.btn { padding: 15px 20px; border: none; border-radius: 5px; font-size: 16px; font-weight: bold; cursor: pointer; transition: all 0.3s; }
		.btn:hover { transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
		.btn:active { transform: translateY(0); }
		.btn-wind { background: #FF9800; color: white; }
		.btn-unwind { background: #9C27B0; color: white; }
		.btn-stop { background: #f44336; color: white; }
		.btn-open { background: #2196F3; color: white; }
		.btn-close { background: #607D8B; color: white; }
		.control-mode { text-align: center; padding: 10px; background: #333; border-radius: 5px; }
		.mode-web { color: #4CAF50; }
		.mode-pwm { color: #FF9800; }
	</style>
</head>
<body>
	<h1>🔧 WireWinder Control Panel</h1>
	<div id="connection-status" class="connection-status loading">Connecting...</div>

	<div class="control-panel">
		<h2>Manual Control</h2>
		<div class="control-mode" id="control-mode">Control Mode: <span class="mode-pwm">PWM</span></div>
		<div style="margin: 15px 0;">
			<button class="btn" onclick="sendCommand('arm')" style="width: 100%; background: #4CAF50;">✓ ARM System</button>
		</div>
		<h3 style="color: #aaa; margin-top: 15px;">Motor Control</h3>
		<div class="button-grid">
			<button class="btn btn-wind" onclick="sendCommand('wind')">⬆️ Wind</button>
			<button class="btn btn-stop" onclick="sendCommand('stop')">⏹️ Stop</button>
			<button class="btn btn-unwind" onclick="sendCommand('unwind')">⬇️ Unwind</button>
		</div>
		<h3 style="color: #aaa;">Test Mode (No Limits)</h3>
		<div class="button-grid">
			<button class="btn" onclick="sendCommand('test_wind')" style="background: #FFA726;">🔧 Test Wind</button>
			<button class="btn" onclick="sendCommand('test_unwind')" style="background: #BA68C8;">🔧 Test Unwind</button>
		</div>
	</div>

	<div class="status-grid">
		<div class="status-card" id="state-card">
			<h3>System State</h3>
			<div class="status-value" id="state">--</div>
			<div class="status-label">Current Operation</div>
		</div>
		<div class="status-card" id="arm-card">
			<h3>ARM Status</h3>
			<div class="status-value" id="armStatus">--</div>
			<div class="status-label">System Armed</div>
		</div>
		<div class="status-card">
			<h3>Input Channels</h3>
			<div class="status-value" id="input1">-- μs</div>
			<div class="status-label">Channel 1 (Motor - Filtered)</div>
			<div style="font-size: 0.9em; color: #888; margin-top: 5px;" id="rawInput1">Raw: -- μs</div>
			<div class="status-value" id="input2" style="margin-top: 10px;">-- μs</div>
			<div class="status-label">Channel 2 (Gate - Filtered)</div>
			<div style="font-size: 0.9em; color: #888; margin-top: 5px;" id="rawInput2">Raw: -- μs</div>
		</div>
		<div class="status-card">
			<h3>Stepper Motor</h3>
			<div class="status-value" id="stepSpeed">0 steps/s</div>
			<div class="status-label">Current Speed</div>
			<div class="status-value" id="stepDir">--</div>
			<div class="status-label">Direction</div>
		</div>
		<div class="status-card">
			<h3>Revolution Counter</h3>
			<div class="status-value" id="revolutions">0.00</div>
			<div class="status-label">Output Rev (Current)</div>
			<div class="status-value" id="motorRevolutions" style="margin-top: 10px;">0.00</div>
			<div class="status-label">Motor Rev (Current)</div>
			<div class="status-label" style="margin-top: 10px;">Output Max: <span id="maxRevs-display">3.0</span></div>
			<div class="status-label">Motor Max: <span id="maxRevs-motor">81.3</span></div>
			<div style="margin-top: 15px;">
				<label style="color: #aaa; font-size: 0.85em;">Max Output Revolutions:</label>
				<input type="number" id="maxRevs" value="3.0" min="0.1" max="100" step="0.1"
					style="width: 100%; padding: 5px; background: #333; color: #fff; border: 1px solid #555; border-radius: 3px;"
					onblur="saveMaxRevs()" onkeypress="if(event.key==='Enter') saveMaxRevs()">
			</div>
		</div>
		<div class="status-card">
			<h3>Gate Servo</h3>
			<div class="status-value" id="servo2">-- μs</div>
			<div class="status-label">Current Position</div>
			<div style="margin-top: 15px;">
				<button class="btn" id="gateToggle" onclick="toggleGate()" style="width: 100%; background: #2196F3;">🔓 Open Gate</button>
			</div>
			<div style="margin-top: 10px;">
				<label style="color: #aaa; font-size: 0.85em;">Open (μs):</label>
				<input type="number" id="servo2Open" value="2000" min="500" max="2500"
					style="width: 100%; padding: 5px; background: #333; color: #fff; border: 1px solid #555; border-radius: 3px;"
					onblur="saveServoSetting('servo2Open')" onkeypress="if(event.key==='Enter') saveServoSetting('servo2Open')">
			</div>
			<div style="margin-top: 5px;">
				<label style="color: #aaa; font-size: 0.85em;">Closed (μs):</label>
				<input type="number" id="servo2Closed" value="1200" min="500" max="2500"
					style="width: 100%; padding: 5px; background: #333; color: #fff; border: 1px solid #555; border-radius: 3px;"
					onblur="saveServoSetting('servo2Closed')" onkeypress="if(event.key==='Enter') saveServoSetting('servo2Closed')">
			</div>
		</div>
		<div class="status-card">
			<h3>Remaining Time (Winding)</h3>
			<div class="status-value" id="remWind">-- s</div>
			<div class="status-label">Time until limit</div>
		</div>
		<div class="status-card">
			<h3>Remaining Time (Unwinding)</h3>
			<div class="status-value" id="remUnwind">-- s</div>
			<div class="status-label">Time until limit</div>
		</div>
	</div>
	<script>
		const stateNames = ['IDLE', 'WINDING', 'UNWINDING', 'ERROR'];
		const stateClasses = ['state-idle', 'state-winding', 'state-unwinding', 'state-error'];
		let activeCommand = null; // Track active command for heartbeat
		let minServoDiff = 50; // Default, will be updated from backend

		function loadSettings() {
			fetch('/settings')
				.then(response => response.json())
				.then(data => {
					document.getElementById('servo2Open').value = data.servo2Open;
					document.getElementById('servo2Closed').value = data.servo2Closed;
					document.getElementById('maxRevs').value = data.maxRevolutions;
					document.getElementById('maxRevs-display').textContent = data.maxRevolutions;
					if (data.maxMotorRevolutions !== undefined) {
						document.getElementById('maxRevs-motor').textContent = data.maxMotorRevolutions;
					}
					if (data.minServoDiff !== undefined) {
						minServoDiff = data.minServoDiff;
					}
				})
				.catch(error => console.error('Failed to load settings:', error));
		}

		function saveMaxRevs() {
			const maxRevolutions = document.getElementById('maxRevs').value;

			fetch('/settings?maxRevolutions=' + maxRevolutions, {
				method: 'POST'
			})
				.then(response => response.json())
				.then(data => {
					if (data.success) {
						document.getElementById('maxRevs-display').textContent = maxRevolutions;
						if (data.maxMotorRevolutions !== undefined) {
							document.getElementById('maxRevs-motor').textContent = data.maxMotorRevolutions;
						}
					}
				})
				.catch(error => {
					console.error('Error saving max revolutions:', error);
				});
		}

		function saveServoSetting(fieldId) {
			const servo2Open = document.getElementById('servo2Open').value;
			const servo2Closed = document.getElementById('servo2Closed').value;

			fetch('/settings?servo2Open=' + servo2Open + '&servo2Closed=' + servo2Closed, {
				method: 'POST'
			})
				.then(response => response.json())
				.then(data => {
					if (!data.success) {
						alert('Failed to save servo settings');
					}
				})
				.catch(error => {
					console.error('Error saving servo settings:', error);
				});
		}

		function toggleGate() {
			const currentPos = parseInt(document.getElementById('servo2').textContent);
			const closedPos = parseInt(document.getElementById('servo2Closed').value);
			const openPos = parseInt(document.getElementById('servo2Open').value);

			// Determine if gate is currently closed (within minServoDiff of closed position)
			const isClosed = Math.abs(currentPos - closedPos) < minServoDiff;

			if (isClosed) {
				sendCommand('open');
			} else {
				sendCommand('close');
			}
		}

		function sendCommand(cmd) {
			activeCommand = cmd; // Set active command
			fetch('/command?cmd=' + cmd)
				.then(response => response.json())
				.then(data => {
					if (data.success) {
						console.log('Command success: ' + data.message);
						// Update control mode indicator
						if (data.mode === 'web') {
							document.getElementById('control-mode').innerHTML =
								'Control Mode: <span class="mode-web">WEB CONTROL ACTIVE</span>';
						} else {
							document.getElementById('control-mode').innerHTML =
								'Control Mode: <span class="mode-pwm">PWM</span>';
						}
					} else {
						alert('Command failed: ' + data.message);
						activeCommand = null;
					}
				})
				.catch(error => {
					alert('Error sending command: ' + error);
					activeCommand = null;
				});
		}

		// Send heartbeat to keep active command alive
		setInterval(() => {
			if (activeCommand && activeCommand !== 'stop') {
				fetch('/command?cmd=' + activeCommand).catch(() => {});
			}
		}, 2000); // Send every 2 seconds

		function updateStatus() {
			fetch('/status')
				.then(response => response.json())
				.then(data => {
					document.getElementById('connection-status').textContent = 'Connected';
					document.getElementById('connection-status').className = 'connection-status connected';

					// Update control mode
					if (data.controlMode === 'web') {
						document.getElementById('control-mode').innerHTML =
							'Control Mode: <span class="mode-web">WEB CONTROL ACTIVE</span>';
					} else {
						document.getElementById('control-mode').innerHTML =
							'Control Mode: <span class="mode-pwm">PWM</span>';
						activeCommand = null; // Clear active command when switched to PWM
					}

					document.getElementById('state').textContent = stateNames[data.state] || 'UNKNOWN';
					const stateCard = document.getElementById('state-card');
					stateCard.className = 'status-card ' + stateClasses[data.state];

					// Update ARM status
					const armCard = document.getElementById('arm-card');
					if (data.armed) {
						document.getElementById('armStatus').textContent = 'ARMED';
						armCard.className = 'status-card arm-ready';
					} else {
						document.getElementById('armStatus').textContent = 'WAITING';
						armCard.className = 'status-card arm-waiting';
					}

					document.getElementById('input1').textContent = data.input1 + ' μs';
					document.getElementById('input2').textContent = data.input2 + ' μs';
					document.getElementById('rawInput1').textContent = 'Raw: ' + data.rawInput1 + ' μs';
					document.getElementById('rawInput2').textContent = 'Raw: ' + data.rawInput2 + ' μs';
					document.getElementById('stepSpeed').textContent = data.stepSpeed + ' steps/s';
					document.getElementById('stepDir').textContent = data.stepSpeed > 0 ? 'UNWINDING' : (data.stepSpeed < 0 ? 'WINDING' : 'STOPPED');
					document.getElementById('revolutions').textContent = (data.revolutions || 0).toFixed(2);
					document.getElementById('motorRevolutions').textContent = (data.motorRevolutions || 0).toFixed(2);
					document.getElementById('maxRevs-display').textContent = (data.maxRevolutions || 3).toFixed(1);
					if (data.maxMotorRevolutions !== undefined) {
						document.getElementById('maxRevs-motor').textContent = data.maxMotorRevolutions.toFixed(1);
					}
					document.getElementById('servo2').textContent = data.servo2 + ' μs';

					// Update gate toggle button text based on current position
					const gateToggle = document.getElementById('gateToggle');
					const closedPos = parseInt(document.getElementById('servo2Closed').value);
					const isClosed = Math.abs(data.servo2 - closedPos) < minServoDiff;
					if (isClosed) {
						gateToggle.innerHTML = '🔓 Open Gate';
						gateToggle.style.background = '#2196F3';
					} else {
						gateToggle.innerHTML = '🔒 Close Gate';
						gateToggle.style.background = '#607D8B';
					}

					document.getElementById('remWind').textContent = data.remWind + ' s';
					document.getElementById('remUnwind').textContent = data.remUnwind + ' s';
				})
				.catch(error => {
					document.getElementById('connection-status').textContent = 'Connection Error';
					document.getElementById('connection-status').className = 'connection-status state-error';
				});
		}

		// Update every 500ms
		setInterval(updateStatus, 500);
		updateStatus();

		// Load settings on page load
		loadSettings();
	</script>
</body>
</html>
)rawliteral";
	server.send(200, "text/html", html);
}

void handleStatus() {
	// Calculate current values
	unsigned long now = millis();

	// Read current pulse values (for diagnostics)
	unsigned long rawPulse1, rawPulse2;
	if (controlMode == CONTROL_WEB) {
		rawPulse1 = webCommandInput1;
		rawPulse2 = webCommandInput2;
	} else {
		rawPulse1 = pulseIn(PIN_INPUT1, HIGH, 25000UL);
		rawPulse2 = pulseIn(PIN_INPUT2, HIGH, 25000UL);
	}

	unsigned long segWind = 0;
	unsigned long segUnw = 0;
	if (currentState == STATE_WINDING && lastStateActiveTime != 0 && now >= lastStateActiveTime)
		segWind = now - lastStateActiveTime;
	if (currentState == STATE_UNWINDING && lastStateActiveTime != 0 && now >= lastStateActiveTime)
		segUnw = now - lastStateActiveTime;

	unsigned long totalElapsedWind = accumulatedWindMs + segWind;
	unsigned long totalElapsedUnwind = accumulatedUnwindMs + segUnw;
	unsigned long remWindMs = (totalElapsedWind >= PULSE_TIMEOUT_WIND_MS) ? 0 : (PULSE_TIMEOUT_WIND_MS - totalElapsedWind);
	unsigned long remUnwMs = (totalElapsedUnwind >= PULSE_TIMEOUT_UNWIND_MS) ? 0 : (PULSE_TIMEOUT_UNWIND_MS - totalElapsedUnwind);
	unsigned long remainingWindSec = (remWindMs + 500) / 1000;
	unsigned long remainingUnwindSec = (remUnwMs + 500) / 1000;

	// Build JSON response
	String json = "{";
	json += "\"state\":" + String((int)currentState) + ",";
	json += "\"armed\":" + String(waitingForArm ? "false" : "true") + ",";
	json += "\"rawInput1\":" + String(rawPulse1) + ",";
	json += "\"rawInput2\":" + String(rawPulse2) + ",";
	json += "\"input1\":" + String(lastInput1Value) + ",";
	json += "\"input2\":" + String(lastInput2Value) + ",";
	json += "\"stepSpeed\":" + String(currentStepperSpeed) + ",";
	json += "\"revolutions\":" + String(revolutionCounter, 2) + ",";
	json += "\"motorRevolutions\":" + String(revolutionCounter * STEPPER_GEAR_RATIO, 2) + ",";
	json += "\"maxRevolutions\":" + String(maxRevolutions, 2) + ",";
	json += "\"maxMotorRevolutions\":" + String(maxRevolutions * STEPPER_GEAR_RATIO, 2) + ",";
	json += "\"servo2\":" + String(currentServo2Pulse) + ",";
	json += "\"remWind\":" + String(remainingWindSec) + ",";
	json += "\"remUnwind\":" + String(remainingUnwindSec) + ",";
	json += "\"controlMode\":\"" + String(controlMode == CONTROL_WEB ? "web" : "pwm") + "\"";
	json += "}";

	server.send(200, "application/json", json);
}

void handleCommand() {
	if (!server.hasArg("cmd")) {
		server.send(400, "text/plain", "Missing command");
		return;
	}

	String cmd = server.arg("cmd");
	bool success = true;
	String message = "OK";

	if (cmd == "arm") {
		// ARM system (emulate PWM arming sequence)
		waitingForArm = false;
		seenArmMax = true;
		dbgPrintln("System ARMED via web command");
		message = "System armed successfully";
	} else if (cmd == "wind") {
		// Start winding (input1 below neutral)
		controlMode = CONTROL_WEB;
		testMode = false;
		webCommandInput1 = 1000; // Winding value
		webCommandInput2 = INPUT_NEUTRAL;
		lastWebCommandTime = millis();
		message = "Winding started";
	} else if (cmd == "unwind") {
		// Start unwinding (input1 above neutral)
		controlMode = CONTROL_WEB;
		testMode = false;
		webCommandInput1 = 1800; // Unwinding value
		webCommandInput2 = INPUT_NEUTRAL;
		lastWebCommandTime = millis();
		message = "Unwinding started";
	} else if (cmd == "test_wind") {
		// Test winding (no revolution limits)
		controlMode = CONTROL_WEB;
		testMode = true;
		// Clear error state and timers so test mode always engages
		currentState = STATE_IDLE;
		errorFromState = STATE_IDLE;
		allowWinding = true;
		allowUnwinding = true;
		accumulatedWindMs = 0;
		accumulatedUnwindMs = 0;
		lastStateActiveTime = 0;
		initialTimersInitialized = false;
		webCommandInput1 = 1000; // Winding value
		webCommandInput2 = INPUT_NEUTRAL;
		lastWebCommandTime = millis();
		message = "Test winding started (no limits)";
	} else if (cmd == "test_unwind") {
		// Test unwinding (no revolution limits)
		controlMode = CONTROL_WEB;
		testMode = true;
		// Clear error state and timers so test mode always engages
		currentState = STATE_IDLE;
		errorFromState = STATE_IDLE;
		allowWinding = true;
		allowUnwinding = true;
		accumulatedWindMs = 0;
		accumulatedUnwindMs = 0;
		lastStateActiveTime = 0;
		initialTimersInitialized = false;
		webCommandInput1 = 1800; // Unwinding value
		webCommandInput2 = INPUT_NEUTRAL;
		lastWebCommandTime = millis();
		message = "Test unwinding started (no limits)";
	} else if (cmd == "stop") {
		// Stop motor
		controlMode = CONTROL_WEB;
		testMode = false;
		webCommandInput1 = INPUT_NEUTRAL;
		webCommandInput2 = INPUT_NEUTRAL;
		lastWebCommandTime = millis();
		message = "Motor stopped";
	} else if (cmd == "open") {
		// Open gate (input2 above neutral)
		controlMode = CONTROL_WEB;
		webCommandInput1 = INPUT_NEUTRAL; // Stop motor
		webCommandInput2 = WEB_GATE_OPEN_CMD;
		lastWebCommandTime = millis();
		message = "Gate opening";
	} else if (cmd == "close") {
		// Close gate (input2 below neutral)
		controlMode = CONTROL_WEB;
		webCommandInput1 = INPUT_NEUTRAL; // Stop motor
		webCommandInput2 = WEB_GATE_CLOSE_CMD;
		lastWebCommandTime = millis();
		message = "Gate closing";
	} else if (cmd == "pwm_mode") {
		// Switch back to PWM control
		controlMode = CONTROL_PWM;
		testMode = false;
		message = "Switched to PWM control mode";
	} else {
		success = false;
		message = "Unknown command: " + cmd;
	}

	String response = "{\"success\":" + String(success ? "true" : "false") +
	                  ",\"message\":\"" + message + "\"" +
	                  ",\"mode\":\"" + String(controlMode == CONTROL_WEB ? "web" : "pwm") + "\"}";
	server.send(success ? 200 : 400, "application/json", response);
}

void handleSettings() {
	if (server.method() == HTTP_GET) {
		// Return current settings
		String json = "{";
		json += "\"servo2Open\":" + String(servo2OpenPos) + ",";
		json += "\"servo2Closed\":" + String(servo2ClosedPos) + ",";
		json += "\"minServoDiff\":" + String(MIN_SERVO_DIFF) + ",";
		json += "\"maxRevolutions\":" + String(maxRevolutions, 2) + ",";
		json += "\"maxMotorRevolutions\":" + String(maxRevolutions * STEPPER_GEAR_RATIO, 2) + ",";
		json += "\"gearRatio\":" + String(STEPPER_GEAR_RATIO, 2);
		json += "}";
		server.send(200, "application/json", json);
	} else if (server.method() == HTTP_POST) {
		// Update settings
		bool changed = false;
		String message = "Settings updated";

		// Temporary variables to hold new values for validation
		uint16_t newServo2Open = servo2OpenPos;
		uint16_t newServo2Closed = servo2ClosedPos;
		bool servo2Changed = false;

		if (server.hasArg("servo2Open")) {
			uint16_t val = server.arg("servo2Open").toInt();
			if (val >= 500 && val <= 2500) {
				newServo2Open = val;
				servo2Changed = true;
			}
		}

		if (server.hasArg("servo2Closed")) {
			uint16_t val = server.arg("servo2Closed").toInt();
			if (val >= 500 && val <= 2500) {
				newServo2Closed = val;
				servo2Changed = true;
			}
		}

		// Validate that open and closed positions are sufficiently different
		if (servo2Changed) {
			uint16_t diff = (newServo2Open > newServo2Closed) 
				? (newServo2Open - newServo2Closed) 
				: (newServo2Closed - newServo2Open);
			
			if (diff >= MIN_SERVO_DIFF) {
				// Valid: positions are sufficiently different
				servo2OpenPos = newServo2Open;
				servo2ClosedPos = newServo2Closed;
				changed = true;
			} else {
				// Invalid: positions are too close
				message = "Error: Open and Closed positions must differ by at least " + String(MIN_SERVO_DIFF) + " μs";
				servo2Changed = false;
			}
		}

		if (server.hasArg("maxRevolutions")) {
			float val = server.arg("maxRevolutions").toFloat();
			if (val > 0 && val <= 100) {
				maxRevolutions = val;
				changed = true;
			}
		}

		// Save to non-volatile memory if settings changed
		if (changed) {
			preferences.begin("wirewinder", false);
			preferences.putUShort("servo2Open", servo2OpenPos);
			preferences.putUShort("servo2Closed", servo2ClosedPos);
			preferences.putFloat("maxRevs", maxRevolutions);
			preferences.end();
			dbgPrintln("Settings saved to non-volatile memory");
		}

		String response = "{\"success\":" + String(changed ? "true" : "false") +
		                  ",\"message\":\"" + message + "\"" +
		                  ",\"maxRevolutions\":" + String(maxRevolutions, 2) +
		                  ",\"maxMotorRevolutions\":" + String(maxRevolutions * STEPPER_GEAR_RATIO, 2) + "}";
		server.send(200, "application/json", response);
	}
}

void setup() {
	Serial.begin(115200);
	delay(100);
	// No startup delay for serial output
	startupBlockUntil = 0;
	dbgPrintln("WireWinder (TMC2209 Stepper): starting");

	// Load settings from non-volatile memory
	preferences.begin("wirewinder", true); // true = read-only
	servo2OpenPos = preferences.getUShort("servo2Open", 1900); // default 1900
	servo2ClosedPos = preferences.getUShort("servo2Closed", 1100); // default 1100
	maxRevolutions = preferences.getFloat("maxRevs", 3.0f); // default 3.0
	preferences.end();
	
	// Validate loaded servo positions - ensure they're sufficiently different
	uint16_t diff = (servo2OpenPos > servo2ClosedPos) 
		? (servo2OpenPos - servo2ClosedPos) 
		: (servo2ClosedPos - servo2OpenPos);
	if (diff < MIN_SERVO_DIFF) {
		dbgPrintf("WARNING: Loaded servo positions too close (%u µs apart), resetting to defaults\n", diff);
		servo2OpenPos = 1900;
		servo2ClosedPos = 1100;
		// Save corrected values
		preferences.begin("wirewinder", false);
		preferences.putUShort("servo2Open", servo2OpenPos);
		preferences.putUShort("servo2Closed", servo2ClosedPos);
		preferences.end();
	}
	
	dbgPrintf("Loaded settings: servo2Open=%u servo2Closed=%u maxRevs=%.1f\n",
	          servo2OpenPos, servo2ClosedPos, maxRevolutions);

	// Connect to WiFi
	dbgPrintf("Connecting to WiFi: %s\n", WIFI_SSID);
	WiFi.mode(WIFI_STA);
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

	unsigned long wifiStartTime = millis();
	while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 10000) {
		delay(500);
		Serial.print(".");
	}

	if (WiFi.status() == WL_CONNECTED) {
		dbgPrintln("\nWiFi connected!");
		dbgPrintf("IP address: %s\n", WiFi.localIP().toString().c_str());
		dbgPrintf("Open http://%s in your browser\n", WiFi.localIP().toString().c_str());
	} else {
		dbgPrintln("\nWiFi connection failed! Continuing without web interface.");
	}

	// Setup web server routes
	server.on("/", handleRoot);
	server.on("/status", handleStatus);
	server.on("/command", handleCommand);
	server.on("/settings", handleSettings);
	server.begin();
	dbgPrintln("Web server started");

	// Initialize hardware UART (Serial1) on pins RX=21, TX=22
	Serial1.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
	dbgPrintln("Serial1 initialized on RX=21 TX=22 (115200)");

	pinMode(PIN_INPUT1, INPUT);
	pinMode(PIN_INPUT2, INPUT);
	// LED pin
	pinMode(LED_PIN, OUTPUT);

	// Initialize FastAccelStepper
	stepperEngine.init();
	stepper = stepperEngine.stepperConnectToPin(PIN_STEP);
	if (stepper == nullptr) {
		dbgPrintln("ERROR: Failed to init FastAccelStepper");
	} else {
		stepper->setDirectionPin(PIN_DIR);
		stepper->setEnablePin(PIN_EN, true); // TMC2209 EN is active LOW
		stepper->setAutoEnable(false);
		stepper->setAcceleration(STEPPER_ACCEL);
		// Explicitly disable stepper at startup to ensure no movement
		// Position is set to 0 (arbitrary) - actual position tracking uses revolutionCounter
		stepper->forceStopAndNewPosition(0);
	}

	// TMC2209 in standalone mode - no UART configuration needed
	// Configuration done via hardware: Vref resistor sets current, MS pins set microstepping
	dbgPrintf("TMC2209 in standalone mode (no UART): %d microsteps expected\n", STEPPER_MICROSTEPS);
	setStepperHold();
	dbgPrintln("Stepper motor initialized: STOPPED with HOLD enabled at startup");

	// Attach servo2 with wide pulse range (500-2500 µs)
	servo2.attach(PIN_SERVO2, 500, 2500);

	servo2.writeMicroseconds(servo2ClosedPos);
	currentServo2Pulse = servo2ClosedPos;
	dbgPrintf("Servo initialized to closed position: %u µs\n", servo2ClosedPos);

	// Initialize revolution counter at maxRevolutions (fully wound)
	// NOTE: This assumes the physical system starts in a fully wound state
	// The operator must ensure the system is fully wound before powering on
	revolutionCounter = maxRevolutions;
	dbgPrintf("Revolution counter initialized at %.2f (fully wound, available for unwinding)\n", revolutionCounter);

	lastStepperUpdateTime = millis();
}

void loop() {
	// Handle web server requests
	server.handleClient();

	const unsigned long now = millis();

	// Check web command timeout
	if (controlMode == CONTROL_WEB && (now - lastWebCommandTime > WEB_COMMAND_TIMEOUT)) {
		// Timeout: switch back to PWM mode and stop
		controlMode = CONTROL_PWM;
		testMode = false;
		webCommandInput1 = INPUT_NEUTRAL;
		webCommandInput2 = INPUT_NEUTRAL;
		dbgPrintln("Web control timeout, switching back to PWM mode");
	}

	// Read both input pulse widths (or use web commands)
	unsigned long pulse1, pulse2;
	if (controlMode == CONTROL_WEB) {
		// Use web commands
		pulse1 = webCommandInput1;
		pulse2 = webCommandInput2;
	} else {
		// Use PWM inputs
		pulse1 = pulseIn(PIN_INPUT1, HIGH, 25000UL);
		pulse2 = pulseIn(PIN_INPUT2, HIGH, 25000UL);
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
	// Web control bypasses arming
	if (waitingForArm && controlMode != CONTROL_WEB) {
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
		// while waiting, hold stepper with torque and servo safe and ignore other inputs
		setStepperHold();
		servo2.writeMicroseconds(servo2ClosedPos);
		currentServo2Pulse = servo2ClosedPos;
		// skip normal control until armed
		return;
	}

	// Process INPUT1 for winding motor control (only when armed)
	// Accept only pulses in valid control range [800..2000]
	if (pulseInAcceptRange(pulse1, 800, 2000)) {
		lastInput1Value = pulse1;
	} else if (pulse1 != 0) {
		// Log out-of-range pulses for diagnostics
		static unsigned long lastOORLog1 = 0;
		if (now - lastOORLog1 > 1000) {
			dbgPrintf("WARN: Input1 out of range: %luus (accepted range: 800-2000)\n", pulse1);
			lastOORLog1 = now;
		}
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
				setStepperEnable(false);
				currentState = STATE_IDLE;
			} else if (newState == STATE_WINDING) {
				if (!allowWinding) {
						dbgPrintln("Transition to WINDING blocked (direction not allowed)");
					// ignore transition
				} else {
					// If starting WINDING from IDLE at zero, reset timers for a full run
					if (currentState == STATE_IDLE && revolutionCounter <= 0.01f) {
						accumulatedWindMs = 0;
						accumulatedUnwindMs = PULSE_TIMEOUT_UNWIND_MS;
						initialTimersInitialized = true;
						dbgPrintf("START: WIND (reset at 0) accumWind:%lums accumUnw:%lums\n", accumulatedWindMs, accumulatedUnwindMs);
					}
					// If starting WINDING from IDLE (cold start), initialize timers
					if (!initialTimersInitialized) {
						accumulatedWindMs = 0;
						accumulatedUnwindMs = PULSE_TIMEOUT_UNWIND_MS;
						dbgPrintf("START: WIND accumWind:%lums accumUnw:%lums\n", accumulatedWindMs, accumulatedUnwindMs);
						initialTimersInitialized = true;
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
					lastStateActiveTime = now;
					currentState = STATE_WINDING;
				}
			} else if (newState == STATE_UNWINDING) {
				if (!allowUnwinding) {
						dbgPrintln("Transition to UNWINDING blocked (direction not allowed)");
					// ignore transition
				} else {
					// If starting UNWINDING from IDLE at max, reset timers for a full run
					if (currentState == STATE_IDLE && revolutionCounter >= (maxRevolutions - 0.01f)) {
						accumulatedUnwindMs = 0;
						accumulatedWindMs = PULSE_TIMEOUT_WIND_MS;
						initialTimersInitialized = true;
						dbgPrintf("START: UNW (reset at max) accumUnw:%lums accumWind:%lums\n", accumulatedUnwindMs, accumulatedWindMs);
					}
					// If starting UNWINDING from IDLE (cold start), initialize timers
					if (!initialTimersInitialized) {
						accumulatedUnwindMs = 0;
						accumulatedWindMs = PULSE_TIMEOUT_WIND_MS;
						dbgPrintf("START: UNW accumUnw:%lums accumWind:%lums\n", accumulatedUnwindMs, accumulatedWindMs);
						initialTimersInitialized = true;
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
					lastStateActiveTime = now;
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
			// Update allow flags based on remaining times AND revolution counter
			// Allow winding only if: counter < max AND timeout remaining (or testMode)
			// Allow unwinding only if: counter > 0 AND timeout remaining (or testMode)
			unsigned long remWind = (accumulatedWindMs >= PULSE_TIMEOUT_WIND_MS) ? 0 : (PULSE_TIMEOUT_WIND_MS - accumulatedWindMs);
			unsigned long remUnw = (accumulatedUnwindMs >= PULSE_TIMEOUT_UNWIND_MS) ? 0 : (PULSE_TIMEOUT_UNWIND_MS - accumulatedUnwindMs);

			if (testMode) {
				// Test mode: ignore revolution limits, only check timeouts
				allowWinding = (remWind > 0);
				allowUnwinding = (remUnw > 0);
			} else {
				// Normal mode: check both revolution limits and timeouts
				allowWinding = (revolutionCounter < maxRevolutions) && (remWind > 0);
				allowUnwinding = (revolutionCounter > 0) && (remUnw > 0);
			}

		// Check timeout (configured timeout per direction)
		uint32_t timeoutMs = (currentState == STATE_WINDING) ? PULSE_TIMEOUT_WIND_MS : PULSE_TIMEOUT_UNWIND_MS;
		unsigned long totalElapsed = (currentState == STATE_WINDING) ? accumulatedWindMs : accumulatedUnwindMs;
		if (totalElapsed > timeoutMs) {
			dbgPrintf("ERROR: Timeout after %lu seconds\n", timeoutMs / 1000);
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
			// disable stepper immediately
			setStepperEnable(false);
		}

		// Check revolution limits (skip in test mode)
		if (!testMode) {
			if (currentState == STATE_WINDING && revolutionCounter >= maxRevolutions) {
				dbgPrintf("ERROR: Maximum revolutions reached (%.2f)\n", maxRevolutions);
				errorFromState = STATE_WINDING;
				allowWinding = false;
				allowUnwinding = true;
				currentState = STATE_ERROR;
				setStepperEnable(false);
			} else if (currentState == STATE_UNWINDING && revolutionCounter <= 0) {
				dbgPrintf("ERROR: Minimum revolutions reached (0.0)\n");
				errorFromState = STATE_UNWINDING;
				allowUnwinding = false;
				allowWinding = true;
				currentState = STATE_ERROR;
				setStepperEnable(false);
			}
		}
	}

	// Control stepper motor (winding/unwinding)
	if (currentState == STATE_WINDING || currentState == STATE_UNWINDING) {
		// Update stepper speed periodically
		if (now - lastStepperUpdateTime >= STEPPER_UPDATE_INTERVAL) {
			// Calculate target speed from input
			int32_t targetSpeed = computeStepperSpeedFromInput(lastInput1Value);

			// Check if remaining timeout for active direction reached zero
			uint32_t timeoutMs = (currentState == STATE_WINDING) ? PULSE_TIMEOUT_WIND_MS : PULSE_TIMEOUT_UNWIND_MS;
			unsigned long segElapsed = 0;
			if (lastStateActiveTime != 0 && now >= lastStateActiveTime) segElapsed = now - lastStateActiveTime;
			unsigned long totalElapsed = ((currentState == STATE_WINDING) ? accumulatedWindMs : accumulatedUnwindMs) + segElapsed;

			// If timeout reached or direction blocked, stop motor
			if (totalElapsed >= timeoutMs) {
				targetSpeed = 0;
			} else if ((targetSpeed > 0 && !allowUnwinding) || (targetSpeed < 0 && !allowWinding)) {
				targetSpeed = 0;
			}

			// Update stepper speed
			updateStepperSpeed(targetSpeed);

			// Update revolution counter based on speed and elapsed time
			unsigned long dt_ms = now - lastStepperUpdateTime;
			if (dt_ms > 0 && currentStepperSpeed != 0) {
				// Calculate revolutions at output shaft (gear ratio applied)
				// dRev = (speed * dt_sec) / STEPS_PER_OUTPUT_REV
				float dt_sec = dt_ms / 1000.0f;
				float dRev = (currentStepperSpeed * dt_sec) / STEPS_PER_OUTPUT_REV;

				// Winding (negative speed) increases counter, unwinding (positive speed) decreases it
				// So we subtract dRev: if speed is positive (unwinding), counter decreases
				// if speed is negative (winding), counter increases
				revolutionCounter -= dRev;

				// Clamp to limits [0, maxRevolutions]
				if (revolutionCounter > maxRevolutions) revolutionCounter = maxRevolutions;
				if (revolutionCounter < 0) revolutionCounter = 0;
			}

			lastStepperUpdateTime = now;
		}
	} else if (currentState == STATE_ERROR || currentState == STATE_IDLE) {
		// Stop motor but keep holding torque in error or idle state
		setStepperHold();
	}

	// Track INPUT2 and control SERVO2 only when deviating from neutral (deadzone)
	// Accept only pulses in valid control range [800..2000]
	if (pulseInAcceptRange(pulse2, 800, 2000)) {
		lastInput2Value = pulse2;
	} else if (pulse2 != 0) {
		// Log out-of-range pulses for diagnostics
		static unsigned long lastOORLog2 = 0;
		if (now - lastOORLog2 > 1000) {
			dbgPrintf("WARN: Input2 out of range: %luus (accepted range: 800-2000)\n", pulse2);
			lastOORLog2 = now;
		}
	}

	// Control SERVO2 (gate) - change only when INPUT2 crosses deadzone thresholds
	{
		const uint16_t upper = INPUT_NEUTRAL + INPUT_DEADZONE;
		const uint16_t lower = INPUT_NEUTRAL - INPUT_DEADZONE;
		uint16_t target2 = currentServo2Pulse; // default: no change
		if (lastInput2Value > upper) {
			// Above neutral => open
			target2 = servo2OpenPos;
		} else if (lastInput2Value < lower) {
			// Below neutral => closed
			target2 = servo2ClosedPos;
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
		dbgPrintf("Mode:%s State:%d Raw[%lu,%lu] Filt[%u,%u] Revs:%.2f/%.2f Speed:%ld S2:%u RemW:%lu RemU:%lu\n",
				  controlMode == CONTROL_WEB ? "WEB" : "PWM",
				  currentState, pulse1, pulse2, lastInput1Value, lastInput2Value,
				  revolutionCounter, maxRevolutions, currentStepperSpeed, currentServo2Pulse,
				  remainingWindSec, remainingUnwindSec);
		lastPrint = now;
	}

	// Step pulses handled by FastAccelStepper
}




