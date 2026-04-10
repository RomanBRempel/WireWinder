#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <FastAccelStepper.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <Update.h>

#define FIRMWARE_VERSION "1.0.5"
#define FIRMWARE_UPDATE_CHECK_INTERVAL 3600000  // Check every 1 hour (in ms)

// WiFi credentials (will be loaded from preferences)
char wifiSSID[64] = "RBR_WiFi";  // Configurable, stored in preferences
char wifiPassword[64] = "87770759";  // Configurable, stored in preferences
// Defaults if preferences are empty
const char* DEFAULT_WIFI_SSID = "RBR_WiFi";
const char* DEFAULT_WIFI_PASSWORD = "87770759";

// Access Point credentials (if can't connect to WiFi)
const char* AP_SSID = "WireWinder";
const char* AP_PASSWORD = "12345678"; // Minimum 8 characters for WPA2

// Firmware update server (fallback base URL)
// For GitHub OTA prefer direct RAW links in FIRMWARE_MANIFEST_URL/FIRMWARE_BINARY_URL.
const char* FIRMWARE_SERVER_URL = "https://raw.githubusercontent.com/RomanBRempel/WireWinder/main/ota";
const char* FIRMWARE_MANIFEST_FILE = "version.json";  // Server should have: version.json, wirewinder.bin
// Optional direct URLs. If set, they are used instead of FIRMWARE_SERVER_URL + file name.
// GitHub RAW format: https://raw.githubusercontent.com/<owner>/<repo>/<branch>/ota/<file>
const char* FIRMWARE_MANIFEST_URL = "https://raw.githubusercontent.com/RomanBRempel/WireWinder/main/ota/version.json";
const char* FIRMWARE_BINARY_URL = "https://raw.githubusercontent.com/RomanBRempel/WireWinder/main/ota/wirewinder.bin";

static String buildFirmwareUrl(const char* fileName) {
	String base = String(FIRMWARE_SERVER_URL);
	if (base.endsWith("/")) {
		return base + String(fileName);
	}
	return base + "/" + String(fileName);
}

static bool isGoogleDriveFolderUrl(const String& url) {
	return url.indexOf("drive.google.com/drive/folders/") != -1;
}

static String getManifestUrl() {
	if (strlen(FIRMWARE_MANIFEST_URL) > 0) {
		return String(FIRMWARE_MANIFEST_URL);
	}
	return buildFirmwareUrl(FIRMWARE_MANIFEST_FILE);
}

static String getBinaryUrl() {
	if (strlen(FIRMWARE_BINARY_URL) > 0) {
		return String(FIRMWARE_BINARY_URL);
	}
	return buildFirmwareUrl("wirewinder.bin");
}

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
static const uint32_t STEPPER_MAX_SPEED = 8500;    // Maximum speed in steps/sec
static const uint32_t STEPPER_ACCEL = 4000;        // Acceleration in steps/sec²
static const uint16_t STEPPER_MICROSTEPS = 8;     // Microstepping setting
static const uint16_t STEPPER_RMS_CURRENT = 800;   // RMS current in mA
static const int32_t STEPS_PER_REV = 200 * STEPPER_MICROSTEPS; // 200 full steps/rev * microsteps
float STEPPER_GEAR_RATIO = 25.0f; // Motor turns per output revolution (configurable)
float STEPS_PER_OUTPUT_REV = (float)STEPS_PER_REV * STEPPER_GEAR_RATIO; // Recalculated when gear ratio changes

// User-configurable servo positions (loaded from non-volatile memory at startup)
uint16_t servo2ClosedPos = 1100;   // Gate closed position (default, will be overridden)
uint16_t servo2OpenPos = 1900;     // Gate open position (default, will be overridden)

// Control parameters
static const uint16_t INPUT_DEADZONE = 30;        // Deadzone around neutral (±30 µs)
static const uint16_t INPUT_NEUTRAL = 1500;       // Neutral position
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
int32_t lastStepperPosition = 0;  // Last stepper position in steps (from getCurrentPosition())
unsigned long lastStepperUpdateTime = 0; // Time of last stepper update
bool allowWinding = true;
bool allowUnwinding = true;
// Remember which direction caused the last ERROR so recovery can be restricted
SystemState errorFromState = STATE_IDLE;

// Max-min sequence tracking for AP mode activation
static uint8_t maxMinSequenceCount = 0;       // Count of max-min sequences detected
static unsigned long lastMaxMinSequenceTime = 0; // Time of last detected max-min sequence
static unsigned long maxMinSequenceTimeout = 10000; // Timeout for sequence (10 seconds)
static bool lastInputWasMax = false;           // Track if last input was a max command
static bool lastInputWasMin = false;           // Track if last input was a min command
static bool needsNeutralForSequence = false;   // Flag: waiting for neutral to complete sequence
// Same tracking for INPUT2
static uint8_t maxMinSequenceCount2 = 0;
static unsigned long lastMaxMinSequenceTime2 = 0;
static bool lastInput2WasMax = false;
static bool lastInput2WasMin = false;
static bool needsNeutralForSequence2 = false;
// AP sequence diagnostics for web UI
static String apSequenceSource = "NONE";
static String apSequenceStatus = "Idle";
static String apLastCapturedCommand = "--";
static bool apSequenceStarted = false;
static bool apEnableCommandCaptured = false;
static unsigned long apSequenceEventMs = 0;
// AP activation sequence thresholds (more tolerant than control deadzone)
static const uint16_t AP_SEQ_MAX_THRESHOLD = 1800;
static const uint16_t AP_SEQ_MIN_THRESHOLD = 1200;
static const uint16_t AP_SEQ_NEUTRAL_LOW = 1400;
static const uint16_t AP_SEQ_NEUTRAL_HIGH = 1600;

// Runtime tracked values for stepper and servo2 and last inputs
static int32_t currentStepperSpeed = 0;  // Current stepper speed in steps/sec
static bool stepperEnabled = false;       // Stepper enable state
static uint16_t currentServo2Pulse = 1100; // Initially closed (default, will be updated in setup)
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

static inline bool isInDeadzone(unsigned long pulse);

static void setApSequenceUiState(const char* source, const char* status, const char* command, bool started, bool commandCaptured) {
	apSequenceSource = source;
	apSequenceStatus = status;
	apLastCapturedCommand = command;
	apSequenceStarted = started;
	apEnableCommandCaptured = commandCaptured;
	apSequenceEventMs = millis();
}

static inline bool isApSequenceNeutral(unsigned long pulse) {
	return (pulse >= AP_SEQ_NEUTRAL_LOW) && (pulse <= AP_SEQ_NEUTRAL_HIGH);
}

static void activateAccessPointMode(const char* source, const char* reason) {
	setApSequenceUiState(source, "AP enable command confirmed. Access Point starting", "AP_ENABLE_CONFIRMED", true, true);
	dbgPrintf("\n===== ENABLING ACCESS POINT MODE (%s) =====\n", reason);
	WiFi.mode(WIFI_AP);
	bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);
	if (apStarted) {
		IPAddress apIP = WiFi.softAPIP();
		dbgPrintf("Access Point ACTIVATED!\n");
		dbgPrintf("AP SSID: %s\n", AP_SSID);
		dbgPrintf("AP Password: %s\n", AP_PASSWORD);
		dbgPrintf("AP IP address: %s\n", apIP.toString().c_str());
		dbgPrintf("Open http://%s in your browser\n", apIP.toString().c_str());
	} else {
		dbgPrintln("Failed to create Access Point!");
	}
}

static void processApSequenceInput1(unsigned long now) {
	if (maxMinSequenceCount > 0 && (now - lastMaxMinSequenceTime > maxMinSequenceTimeout)) {
		dbgPrintf("Max-min sequence timeout: resetting counter from %d to 0\n", maxMinSequenceCount);
		setApSequenceUiState("INPUT1", "Sequence timeout. Counter reset", "TIMEOUT_RESET", false, false);
		maxMinSequenceCount = 0;
		lastInputWasMax = false;
		lastInputWasMin = false;
		needsNeutralForSequence = false;
	}

	bool isMax = (lastInput1Value >= AP_SEQ_MAX_THRESHOLD);
	bool isMin = (lastInput1Value <= AP_SEQ_MIN_THRESHOLD);
	bool isNeutral = isApSequenceNeutral(lastInput1Value);

	if (needsNeutralForSequence) {
		if (isNeutral) {
			needsNeutralForSequence = false;
			if (lastInputWasMax && lastInputWasMin) {
				maxMinSequenceCount++;
				lastMaxMinSequenceTime = now;
				dbgPrintf("Max-min sequence detected! Count: %d/3\n", maxMinSequenceCount);
				setApSequenceUiState("INPUT1", ("Cycle captured: " + String(maxMinSequenceCount) + "/3").c_str(), "CYCLE_COMPLETE", true, false);
				lastInputWasMax = false;
				lastInputWasMin = false;

				if (maxMinSequenceCount >= 3) {
					activateAccessPointMode("INPUT1", "3 max-min sequences detected on INPUT1");
					maxMinSequenceCount = 0;
				}
			}
		}
	} else {
		if (isMax && !lastInputWasMax) {
			lastInputWasMax = true;
			needsNeutralForSequence = true;
			setApSequenceUiState("INPUT1", "Sequence started: MAX captured, waiting NEUTRAL", "MAX", true, false);
			dbgPrintln("Max detected");
		} else if (isMin && !lastInputWasMin && lastInputWasMax) {
			lastInputWasMin = true;
			needsNeutralForSequence = true;
			setApSequenceUiState("INPUT1", "MIN captured, waiting NEUTRAL to complete cycle", "MIN", true, false);
			dbgPrintln("Min detected");
		}
	}
}

static void processApSequenceInput2(unsigned long now) {
	if (maxMinSequenceCount2 > 0 && (now - lastMaxMinSequenceTime2 > maxMinSequenceTimeout)) {
		dbgPrintf("Max-min sequence timeout (INPUT2): resetting counter from %d to 0\n", maxMinSequenceCount2);
		setApSequenceUiState("INPUT2", "Sequence timeout. Counter reset", "TIMEOUT_RESET", false, false);
		maxMinSequenceCount2 = 0;
		lastInput2WasMax = false;
		lastInput2WasMin = false;
		needsNeutralForSequence2 = false;
	}

	bool isMax2 = (lastInput2Value >= AP_SEQ_MAX_THRESHOLD);
	bool isMin2 = (lastInput2Value <= AP_SEQ_MIN_THRESHOLD);
	bool isNeutral2 = isApSequenceNeutral(lastInput2Value);

	if (needsNeutralForSequence2) {
		if (isNeutral2) {
			needsNeutralForSequence2 = false;
			if (lastInput2WasMax && lastInput2WasMin) {
				maxMinSequenceCount2++;
				lastMaxMinSequenceTime2 = now;
				dbgPrintf("Max-min sequence detected on INPUT2! Count: %d/3\n", maxMinSequenceCount2);
				setApSequenceUiState("INPUT2", ("Cycle captured: " + String(maxMinSequenceCount2) + "/3").c_str(), "CYCLE_COMPLETE", true, false);
				lastInput2WasMax = false;
				lastInput2WasMin = false;

				if (maxMinSequenceCount2 >= 3) {
					activateAccessPointMode("INPUT2", "3 max-min sequences detected on INPUT2");
					maxMinSequenceCount2 = 0;
				}
			}
		}
	} else {
		if (isMax2 && !lastInput2WasMax) {
			lastInput2WasMax = true;
			needsNeutralForSequence2 = true;
			setApSequenceUiState("INPUT2", "Sequence started: MAX captured, waiting NEUTRAL", "MAX", true, false);
			dbgPrintln("Max detected (INPUT2)");
		} else if (isMin2 && !lastInput2WasMin && lastInput2WasMax) {
			lastInput2WasMin = true;
			needsNeutralForSequence2 = true;
			setApSequenceUiState("INPUT2", "MIN captured, waiting NEUTRAL to complete cycle", "MIN", true, false);
			dbgPrintln("Min detected (INPUT2)");
		}
	}
}

static String lastErrorMessage = "";

// LED indicator (built-in)
static const int LED_PIN = 2; // onboard LED on most ESP32 dev boards
enum LedMode { LED_OFF=0, LED_SLOW, LED_FAST, LED_ON };
static LedMode ledMode = LED_OFF;
static bool ledState = false;
static unsigned long lastLedToggle = 0;


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
	// Force immediate stop without deceleration and save current position
	stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
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

// WiFi configuration variables
bool wifiConfigChanged = false;
String wifiConnectionStatus = "Disconnected";

// Firmware update variables
String availableFirmwareVersion = "";
String firmwareUpdateStatus = "Idle";
bool firmwareUpdateAvailable = false;
unsigned long lastFirmwareCheckTime = 0;
unsigned long firmwareCheckStartTime = 0;

// ==================== FIRMWARE UPDATE FUNCTIONS ====================

// Check for available firmware updates on the server
void checkFirmwareUpdate() {
	if (!WiFi.isConnected()) {
		firmwareUpdateStatus = "WiFi not connected";
		return;
	}

	firmwareUpdateStatus = "Checking...";
	firmwareCheckStartTime = millis();

	String baseUrl = String(FIRMWARE_SERVER_URL);
	if (isGoogleDriveFolderUrl(baseUrl)) {
		firmwareUpdateAvailable = false;
		availableFirmwareVersion = "";
		firmwareUpdateStatus = "Invalid OTA URL: use direct file link";
		dbgPrintln("[OTA] Invalid OTA URL: Google Drive folder link is not a direct file URL");
		lastFirmwareCheckTime = millis();
		return;
	}

	HTTPClient http;
	String versionUrl = getManifestUrl();
	// Add cache-busting parameter to avoid stale CDN/proxy responses.
	versionUrl += (versionUrl.indexOf('?') == -1 ? "?ts=" : "&ts=") + String(millis());
	
	dbgPrintf("[OTA] Checking firmware update from: %s\n", versionUrl.c_str());
	
	http.begin(versionUrl);
	int httpCode = http.GET();
	
	if (httpCode == HTTP_CODE_OK) {
		String payload = http.getString();
		payload.trim();
		dbgPrintf("[OTA] Response: %s\n", payload.c_str());

		if (!payload.startsWith("{")) {
			firmwareUpdateAvailable = false;
			availableFirmwareVersion = "";
			firmwareUpdateStatus = "Manifest is not JSON";
			dbgPrintln("[OTA] Manifest is not JSON (likely an HTML page)");
			http.end();
			lastFirmwareCheckTime = millis();
			return;
		}
		
		// Simple JSON parsing (looking for "version": "X.X.X")
		int versionStart = payload.indexOf("\"version\"");
		if (versionStart != -1) {
			versionStart = payload.indexOf("\"", versionStart + 10);
			int versionEnd = payload.indexOf("\"", versionStart + 1);
			if (versionStart != -1 && versionEnd != -1) {
				availableFirmwareVersion = payload.substring(versionStart + 1, versionEnd);
				
				// Compare versions
				if (availableFirmwareVersion != FIRMWARE_VERSION) {
					firmwareUpdateAvailable = true;
					firmwareUpdateStatus = "Update available: " + availableFirmwareVersion;
					dbgPrintf("[OTA] Update available! Current: %s, Available: %s\n", FIRMWARE_VERSION, availableFirmwareVersion.c_str());
				} else {
					firmwareUpdateAvailable = false;
					firmwareUpdateStatus = "Already up to date";
					dbgPrintln("[OTA] Firmware is already up to date");
				}
			} else {
				firmwareUpdateAvailable = false;
				availableFirmwareVersion = "";
				firmwareUpdateStatus = "Invalid manifest: bad version value";
				dbgPrintln("[OTA] Invalid manifest: bad version value");
			}
		} else {
			firmwareUpdateAvailable = false;
			availableFirmwareVersion = "";
			firmwareUpdateStatus = "Invalid response format";
			dbgPrintln("[OTA] Invalid response format");
		}
	} else {
		firmwareUpdateAvailable = false;
		availableFirmwareVersion = "";
		firmwareUpdateStatus = "Check failed (HTTP " + String(httpCode) + ")";
		dbgPrintf("[OTA] Check failed with HTTP code: %d\n", httpCode);
	}
	
	http.end();
	lastFirmwareCheckTime = millis();
}

// Download and install firmware update
bool downloadAndUpdateFirmware() {
	if (!WiFi.isConnected()) {
		dbgPrintln("[OTA] WiFi not connected!");
		firmwareUpdateStatus = "WiFi not connected";
		return false;
	}

	if (!firmwareUpdateAvailable || availableFirmwareVersion.isEmpty()) {
		dbgPrintln("[OTA] No update available!");
		firmwareUpdateStatus = "No update available";
		return false;
	}

	firmwareUpdateStatus = "Downloading...";
	dbgPrintf("[OTA] Starting firmware download from: %s\n", FIRMWARE_SERVER_URL);

	String baseUrl = String(FIRMWARE_SERVER_URL);
	if (isGoogleDriveFolderUrl(baseUrl)) {
		dbgPrintln("[OTA] Invalid OTA URL for binary download: Google Drive folder link is not supported");
		firmwareUpdateStatus = "Invalid OTA URL: use direct file link";
		return false;
	}

	// Disable stepper and motors during update
	if (stepper) {
		stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
		stepper->disableOutputs();
	}
	servo2.detach();
	
	HTTPClient http;
	String firmwareUrl = getBinaryUrl();
	http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
	http.setConnectTimeout(15000);
	http.setTimeout(30000);
	
	if (!http.begin(firmwareUrl)) {
		dbgPrintln("[OTA] Failed to initialize HTTP");
		firmwareUpdateStatus = "HTTP init failed";
		return false;
	}

	int httpCode = http.GET();
	if (httpCode != HTTP_CODE_OK) {
		dbgPrintf("[OTA] Firmware download request failed: HTTP %d\n", httpCode);
		http.end();
		firmwareUpdateStatus = "Download failed (HTTP " + String(httpCode) + ")";
		return false;
	}

	int contentLength = http.getSize();
	if (contentLength <= 0) {
		dbgPrintln("[OTA] Content length unknown, using UPDATE_SIZE_UNKNOWN");
	} else {
		dbgPrintf("[OTA] Firmware size: %d bytes\n", contentLength);
	}

	if (!Update.begin(contentLength > 0 ? contentLength : UPDATE_SIZE_UNKNOWN)) {
		dbgPrintln("[OTA] Not enough space to start update");
		http.end();
		firmwareUpdateStatus = "Not enough flash space";
		return false;
	}

	firmwareUpdateStatus = "Flashing...";
	unsigned long downloadStartTime = millis();
	WiFiClient * stream = http.getStreamPtr();
	size_t written = Update.writeStream(*stream);

	if (contentLength > 0 && written != (size_t)contentLength) {
		dbgPrintf("[OTA] Downloaded only %d / %d bytes\n", written, contentLength);
		http.end();
		Update.abort();
		firmwareUpdateStatus = "Download incomplete";
		return false;
	}

	dbgPrintf("[OTA] Written bytes: %d\n", written);

	if (!Update.end(true)) {
		dbgPrintf("[OTA] Update failed, error: %d\n", Update.getError());
		http.end();
		firmwareUpdateStatus = "Update verification failed";
		return false;
	}

	http.end();

	if (!Update.isFinished()) {
		dbgPrintln("[OTA] Update not finished!");
		firmwareUpdateStatus = "Update incomplete";
		return false;
	}

	unsigned long downloadTime = millis() - downloadStartTime;
	dbgPrintf("[OTA] Update successful! Downloaded in %lu ms\n", downloadTime);
	firmwareUpdateStatus = "Update successful, restarting...";
	
	delay(1000);
	dbgPrintln("[OTA] RESTARTING...");
	ESP.restart();
	
	return true;
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
		.status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 10px; align-items: stretch; }
		.status-card { background: #2a2a2a; padding: 12px; border-radius: 8px; border-left: 4px solid #4CAF50; min-width: 0; }
		.status-card h3 { margin: 0 0 8px 0; color: #4CAF50; font-size: 1em; }
		.status-value { font-size: 1.2em; font-weight: bold; color: #fff; }
		.status-label { color: #aaa; font-size: 0.82em; margin-top: 5px; }
		.state-idle { border-left-color: #2196F3; }
		.state-winding { border-left-color: #FF9800; }
		.state-unwinding { border-left-color: #9C27B0; }
		.state-error { border-left-color: #f44336; }
		.connection-status { padding: 10px; border-radius: 5px; margin-bottom: 20px; text-align: center; font-weight: bold; }
		.connected { background: #4CAF50; color: white; }
		.ap-mode { background: #2196F3; color: white; }
		.loading { background: #FF9800; color: white; }
		.clock { position: fixed; left: 10px; bottom: 10px; background: #2a2a2a; color: #e0e0e0; padding: 6px 10px; border-radius: 6px; font-weight: bold; border: 1px solid #444; }
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
		.pwm-map { margin-top: 8px; color: #bbb; font-size: 0.9em; }
		.ap-seq { border-left-color: #FFC107; }
		.ap-seq-confirmed { border-left-color: #4CAF50; }

		/* Keep status cards in a single row on wide screens, reduce columns as width shrinks */
		@media (min-width: 1800px) {
			#status-panel .status-grid { grid-template-columns: repeat(7, minmax(0, 1fr)); }
		}
		@media (min-width: 1500px) and (max-width: 1799px) {
			#status-panel .status-grid { grid-template-columns: repeat(6, minmax(0, 1fr)); }
		}
		@media (min-width: 1200px) and (max-width: 1499px) {
			#status-panel .status-grid { grid-template-columns: repeat(4, minmax(0, 1fr)); }
		}
		@media (min-width: 900px) and (max-width: 1199px) {
			#status-panel .status-grid { grid-template-columns: repeat(3, minmax(0, 1fr)); }
		}
		@media (max-width: 899px) {
			#status-panel .status-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
		}
		@media (max-width: 640px) {
			#status-panel .status-grid { grid-template-columns: 1fr; }
		}
	</style>
</head>
<body>
	<h1>🔧 WireWinder Control Panel</h1>
	<div id="connection-status" class="connection-status loading">Connecting...</div>
	<div id="clock" class="clock">--:--:--</div>

	<div class="control-panel">
		<h2>Manual Control</h2>
		<div class="control-mode" id="control-mode">
			Control Mode: <span class="mode-pwm">PWM</span>
			<div class="pwm-map">PWM physical inputs: CH1/Motor = GPIO16, CH2/Gate = GPIO17</div>
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

	<div class="control-panel" id="status-panel">
		<h2>Status</h2>
		<div class="status-grid">
			<div class="status-card" id="state-card">
				<h3>System State</h3>
				<div class="status-value" id="state">--</div>
				<div class="status-label">Current Operation</div>
				<div id="errorInfo" style="display: none; margin-top: 10px; color: #f44336; font-weight: bold;"></div>
			</div>
			<div class="status-card ap-seq" id="ap-seq-card">
				<h3>AP Activation</h3>
				<div class="status-value" id="apSeqSource">NONE</div>
				<div class="status-label">Detected Input Source</div>
				<div class="status-value" id="apSeqProgress" style="margin-top: 10px;">0/3</div>
				<div class="status-label">Captured Max-Min Cycles</div>
				<div class="status-label" id="apSeqCmd" style="margin-top: 10px;">Last command: --</div>
				<div style="font-size: 0.9em; color: #ddd; margin-top: 8px;" id="apSeqStatus">Status: Idle</div>
			</div>
			<div class="status-card">
				<h3>Input Channels</h3>
				<div class="status-value" id="input1">-- μs</div>
				<div class="status-label">Channel 1 (Motor, GPIO16 - Filtered)</div>
				<div style="font-size: 0.9em; color: #888; margin-top: 5px;" id="rawInput1">Raw: -- μs</div>
				<div class="status-value" id="input2" style="margin-top: 10px;">-- μs</div>
				<div class="status-label">Channel 2 (Gate, GPIO17 - Filtered)</div>
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
				<div style="margin-top: 10px;">
					<label style="color: #aaa; font-size: 0.85em;">Gear Ratio (Motor:Output):</label>
					<input type="number" id="gearRatio" value="25.0" min="1" max="100" step="0.1"
						style="width: 100%; padding: 5px; background: #333; color: #fff; border: 1px solid #555; border-radius: 3px;"
						onblur="saveGearRatio()" onkeypress="if(event.key==='Enter') saveGearRatio()">
				</div>
			</div>
			<div class="status-card">
				<h3>Gate Servo</h3>
				<div class="status-value" id="gateStatus">--</div>
				<div class="status-label">Gate Status</div>
				<div class="status-value" id="servo2">-- μs</div>
				<div class="status-label">Current Position</div>
				<div style="margin-top: 15px;">
					<button class="btn" id="gateToggle" onclick="toggleGate()" style="width: 100%; background: #2196F3;">🔓 Open Gate</button>
				</div>
				<div style="margin-top: 10px;">
					<label style="color: #aaa; font-size: 0.85em;">Open (μs):</label>
					<input type="number" id="servo2Open" value="1900" min="500" max="2500"
						style="width: 100%; padding: 5px; background: #333; color: #fff; border: 1px solid #555; border-radius: 3px;"
						onblur="saveServoSetting('servo2Open')" onkeypress="if(event.key==='Enter') saveServoSetting('servo2Open')">
				</div>
				<div style="margin-top: 5px;">
					<label style="color: #aaa; font-size: 0.85em;">Closed (μs):</label>
					<input type="number" id="servo2Closed" value="1100" min="500" max="2500"
						style="width: 100%; padding: 5px; background: #333; color: #fff; border: 1px solid #555; border-radius: 3px;"
						onblur="saveServoSetting('servo2Closed')" onkeypress="if(event.key==='Enter') saveServoSetting('servo2Closed')">
				</div>
			</div>
		</div>
	</div>

	<div class="control-panel" id="firmware-panel">
		<h2>Firmware Management</h2>
		<div id="fw-status" style="background: #333; padding: 12px; border-radius: 5px; margin-bottom: 15px;">
			<div style="color: #bbb; font-size: 0.9em;">Current Version: <span id="fw-current-version" style="color: #4CAF50; font-weight: bold;">--</span></div>
			<div style="color: #bbb; font-size: 0.9em; margin-top: 5px;">Available Version: <span id="fw-available-version" style="color: #FFC107; font-weight: bold;">--</span></div>
			<div style="color: #ddd; font-size: 0.9em; margin-top: 8px;">Status: <span id="fw-update-status" style="color: #fff;">Checking...</span></div>
		</div>
		<div class="button-grid">
			<button class="btn" onclick="checkFirmwareUpdate()" style="background: #2196F3; width: 100%;">🔍 Check Updates</button>
		</div>
		<div class="button-grid" id="fw-update-button-container" style="display: none;">
			<button class="btn" onclick="downloadFirmwareUpdate()" style="background: #4CAF50; width: 100%;">⬇️ Download & Install Update</button>
		</div>
		<div style="margin-top: 12px; padding: 10px; background: #333; border-radius: 5px; font-size: 0.85em; color: #999;">
			<strong style="color: #bbb;">Note:</strong> System must be IDLE to update. OTA updates are also available via Arduino IDE.
		</div>
	</div>

	<div class="control-panel" id="wifi-panel">
		<h2>Network Settings</h2>
		<div id="wifi-status" style="background: #333; padding: 12px; border-radius: 5px; margin-bottom: 15px;">
			<div style="color: #bbb; font-size: 0.9em;">Status: <span id="wifi-connection-status" style="color: #FFC107; font-weight: bold;">Checking...</span></div>
			<div style="color: #bbb; font-size: 0.9em; margin-top: 5px;">IP Address: <span id="wifi-ip-address" style="color: #4CAF50; font-weight: bold;">--</span></div>
		</div>
		<div style="margin-bottom: 15px;">
			<label style="color: #aaa; font-size: 0.9em; display: block; margin-bottom: 5px;">WiFi Network (SSID):</label>
			<input type="text" id="wifi-ssid" placeholder="Network name" 
				style="width: 100%; padding: 8px; background: #333; color: #fff; border: 1px solid #555; border-radius: 3px; box-sizing: border-box;"
				maxlength="63">
		</div>
		<div style="margin-bottom: 15px;">
			<label style="color: #aaa; font-size: 0.9em; display: block; margin-bottom: 5px;">WiFi Password (min 8 chars):</label>
			<input type="password" id="wifi-password" placeholder="At least 8 characters" 
				style="width: 100%; padding: 8px; background: #333; color: #fff; border: 1px solid #555; border-radius: 3px; box-sizing: border-box;"
				minlength="8" maxlength="63">
		</div>
		<div class="button-grid">
			<button class="btn" onclick="loadWiFiSettings()" style="background: #2196F3; flex: 1; margin-right: 5px;">📖 Load</button>
			<button class="btn" onclick="saveWiFiSettings()" style="background: #4CAF50; flex: 1; margin-left: 5px;">💾 Save</button>
		</div>
		<div style="margin-top: 12px; padding: 10px; background: #333; border-radius: 5px; font-size: 0.85em; color: #999;">
			<strong style="color: #bbb;">Note:</strong> After saving, device will reconnect within 10 seconds.
		</div>
	</div>

	<script>
		const stateNames = ['IDLE', 'WINDING', 'UNWINDING', 'ERROR'];
		const stateClasses = ['state-idle', 'state-winding', 'state-unwinding', 'state-error'];
		const pwmMapHtml = '<div class="pwm-map">PWM physical inputs: CH1/Motor = GPIO16, CH2/Gate = GPIO17</div>';
		let activeCommand = null; // Track active command for heartbeat
		let useUptimeClock = false;
		let uptimeBaseSec = 0;
		let uptimeBaseMs = 0;

		function renderControlMode(mode) {
			if (mode === 'web') {
				document.getElementById('control-mode').innerHTML =
					'Control Mode: <span class="mode-web">WEB CONTROL ACTIVE</span>' + pwmMapHtml;
			} else {
				document.getElementById('control-mode').innerHTML =
					'Control Mode: <span class="mode-pwm">PWM</span>' + pwmMapHtml;
			}
		}

		function loadSettings() {
			fetch('/settings')
				.then(response => response.json())
				.then(data => {
					document.getElementById('servo2Open').value = data.servo2Open;
					document.getElementById('servo2Closed').value = data.servo2Closed;
					document.getElementById('maxRevs').value = data.maxRevolutions;
					document.getElementById('maxRevs-display').textContent = data.maxRevolutions;
					if (data.gearRatio !== undefined) {
						document.getElementById('gearRatio').value = data.gearRatio.toFixed(2);
					}
					if (data.maxMotorRevolutions !== undefined) {
						document.getElementById('maxRevs-motor').textContent = data.maxMotorRevolutions;
					}
					// Trigger initial status update after settings are loaded
					updateStatus();
				})
				.catch(error => {
					console.error('Failed to load settings:', error);
					// Still trigger update even if settings load fails
					updateStatus();
				});
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

		function saveGearRatio() {
			const gearRatio = document.getElementById('gearRatio').value;

			fetch('/settings?gearRatio=' + gearRatio, {
				method: 'POST'
			})
				.then(response => response.json())
				.then(data => {
					if (data.success) {
						if (data.maxMotorRevolutions !== undefined) {
							document.getElementById('maxRevs-motor').textContent = data.maxMotorRevolutions.toFixed(1);
						}
					}
				})
				.catch(error => {
					console.error('Error saving gear ratio:', error);
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

			// Determine if gate is currently closed (within 50us of closed position)
			const isClosed = Math.abs(currentPos - closedPos) < 50;

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
						renderControlMode(data.mode);
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
					// Update connection status with WiFi info
					let connStatusText = 'Connected';
					let connStatusClass = 'connection-status connected';
					if (data.wifiMode === 'ap') {
						connStatusText = '📡 AP Mode: ' + data.apSSID + ' (' + data.apIP + ')';
						connStatusClass = 'connection-status ap-mode';
					} else if (data.wifiConnected) {
						connStatusText = '🌐 WiFi: ' + data.wifiSSID + ' (' + data.wifiIP + ')';
					}
					document.getElementById('connection-status').textContent = connStatusText;
					document.getElementById('connection-status').className = connStatusClass;

					// Update control mode
					renderControlMode(data.controlMode);
					if (data.controlMode !== 'web') {
						activeCommand = null; // Clear active command when switched to PWM
					}

					document.getElementById('state').textContent = stateNames[data.state] || 'UNKNOWN';
					const stateCard = document.getElementById('state-card');
					stateCard.className = 'status-card ' + stateClasses[data.state];

					const errorInfo = document.getElementById('errorInfo');
					if (data.state === 3 && data.errorMessage) {
						errorInfo.textContent = data.errorMessage;
						errorInfo.style.display = 'block';
					} else {
						errorInfo.textContent = '';
						errorInfo.style.display = 'none';
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

					// AP activation sequence diagnostics
					document.getElementById('apSeqSource').textContent = data.apSeqSource || 'NONE';
					document.getElementById('apSeqProgress').textContent = (data.apSeqProgress || 0) + '/3';
					document.getElementById('apSeqCmd').textContent = 'Last command: ' + (data.apSeqLastCommand || '--');
					document.getElementById('apSeqStatus').textContent = 'Status: ' + (data.apSeqStatus || 'Idle');
					const apSeqCard = document.getElementById('ap-seq-card');
					if (data.apEnableCommandCaptured) {
						apSeqCard.className = 'status-card ap-seq-confirmed';
					} else {
						apSeqCard.className = 'status-card ap-seq';
					}

					// Update gate toggle button text based on current position
					const gateToggle = document.getElementById('gateToggle');
					const closedPos = parseInt(document.getElementById('servo2Closed').value);
					const isClosed = Math.abs(data.servo2 - closedPos) < 50;
					if (isClosed) {
						document.getElementById('gateStatus').textContent = 'CLOSED';
						gateToggle.innerHTML = '🔓 Open Gate';
						gateToggle.style.background = '#2196F3';
					} else {
						document.getElementById('gateStatus').textContent = 'OPEN';
						gateToggle.innerHTML = '🔒 Close Gate';
						gateToggle.style.background = '#607D8B';
					}

					if (data.deviceTime) {
						useUptimeClock = false;
					} else if (data.uptimeSec !== undefined) {
						useUptimeClock = true;
						uptimeBaseSec = data.uptimeSec;
						uptimeBaseMs = Date.now();
					}

				})
				.catch(error => {
					document.getElementById('connection-status').textContent = 'Connection Error';
					document.getElementById('connection-status').className = 'connection-status state-error';
				});
		}

		// Load settings on page load FIRST, which will trigger initial updateStatus
		loadSettings();

		// Check firmware status on load and periodically
		checkFirmwareStatus();
		setInterval(checkFirmwareStatus, 60000); // Check every 60 seconds

		// Firmware update functions
		function checkFirmwareStatus() {
			fetch('/firmware/check')
				.then(response => response.json())
				.then(data => {
					document.getElementById('fw-current-version').textContent = data.currentVersion || '--';
					document.getElementById('fw-available-version').textContent = data.availableVersion || 'Checking...';
					document.getElementById('fw-update-status').textContent = data.status || 'Unknown';
					
					const updateButton = document.getElementById('fw-update-button-container');
					if (data.updateAvailable && data.wifiConnected) {
						updateButton.style.display = 'grid';
					} else {
						updateButton.style.display = 'none';
					}
				})
				.catch(error => {
					console.log('Firmware check error:', error);
					document.getElementById('fw-available-version').textContent = 'Error checking';
				});
		}

		function checkFirmwareUpdate() {
			document.getElementById('fw-update-status').textContent = 'Checking...';
			fetch('/firmware/check', { method: 'POST' })
				.then(response => response.json())
				.then(data => {
					console.log('Firmware check result:', data);
					// Wait a moment then check status
					setTimeout(checkFirmwareStatus, 2000);
				})
				.catch(error => {
					alert('Error checking firmware: ' + error);
					document.getElementById('fw-update-status').textContent = 'Check failed';
				});
		}

		function downloadFirmwareUpdate() {
			if (!confirm('Download and install firmware update?\nSystem must stay powered on during update.')) {
				return;
			}

			document.getElementById('fw-update-status').textContent = 'Downloading...';
			const updateButton = document.getElementById('fw-update-button-container').querySelector('button');
			updateButton.disabled = true;

			fetch('/firmware/update', { method: 'POST' })
				.then(response => response.json())
				.then(data => {
					console.log('Firmware update result:', data);
					if (data.error) {
						alert('Update failed: ' + data.error);
						document.getElementById('fw-update-status').textContent = data.status || 'Update failed';
						updateButton.disabled = false;
					} else {
						document.getElementById('fw-update-status').textContent = 'Update successful, restarting...';
						// Connection will be lost during restart
						setTimeout(() => {
							alert('Device restarting with new firmware...');
							checkFirmwareStatus(); // Try to reconnect
						}, 3000);
					}
				})
				.catch(error => {
					console.log('Firmware update error (may be expected during restart):', error);
					document.getElementById('fw-update-status').textContent = 'Update in progress or device restarting...';
					// Connection may be lost during update
					setTimeout(checkFirmwareStatus, 5000);
				});
		}

		// WiFi Configuration Functions
		function loadWiFiSettings() {
			fetch('/wifi/config')
				.then(response => response.json())
				.then(data => {
					document.getElementById('wifi-ssid').value = data.ssid || '';
					document.getElementById('wifi-password').value = data.password || '';
					document.getElementById('wifi-connection-status').textContent = data.connectionStatus || 'Unknown';
					document.getElementById('wifi-ip-address').textContent = data.wifiIP || '--';
					
					// Update status color
					const statusEl = document.getElementById('wifi-connection-status');
					if (data.wifiConnected) {
						statusEl.style.color = '#4CAF50';
					} else {
						statusEl.style.color = '#FFC107';
					}
				})
				.catch(error => console.log('Error loading WiFi settings:', error));
		}

		function saveWiFiSettings() {
			const ssid = document.getElementById('wifi-ssid').value.trim();
			const password = document.getElementById('wifi-password').value.trim();

			// Validate inputs
			if (!ssid || ssid.length === 0) {
				alert('WiFi network name (SSID) is required');
				return;
			}

			if (!password || password.length < 8) {
				alert('WiFi password must be at least 8 characters');
				return;
			}

			const formData = new URLSearchParams();
			formData.append('ssid', ssid);
			formData.append('password', password);

			fetch('/wifi/config', {
				method: 'POST',
				body: formData
			})
				.then(response => response.json())
				.then(data => {
					if (data.success) {
						alert('WiFi settings saved! Device will reconnect shortly.');
						document.getElementById('wifi-connection-status').textContent = 'Reconnecting...';
						document.getElementById('wifi-connection-status').style.color = '#FFC107';
						// Try to reload settings after a delay
						setTimeout(loadWiFiSettings, 5000);
					} else {
						alert('Error saving settings: ' + (data.error || 'Unknown error'));
					}
				})
				.catch(error => {
					console.log('Error saving WiFi settings:', error);
					alert('Error saving WiFi settings: ' + error);
				});
		}

		// Initial WiFi settings load
		loadWiFiSettings();
		// Reload WiFi settings every 5 seconds
		setInterval(loadWiFiSettings, 5000);

		// Continue updating every 500ms
		setInterval(updateStatus, 500);
		renderControlMode('pwm');

		// Update local time display every second
		setInterval(() => {
			const clockEl = document.getElementById('clock');
			if (useUptimeClock) {
				const elapsedSec = Math.max(0, Math.floor((Date.now() - uptimeBaseMs) / 1000));
				const totalSec = uptimeBaseSec + elapsedSec;
				clockEl.textContent = 'Uptime: ' + totalSec + ' s';
			} else {
				const now = new Date();
				clockEl.textContent = now.toLocaleTimeString();
			}
		}, 1000);
	</script>
	<footer style="margin-top: 30px; text-align: center; color: #555; font-size: 0.8em; padding: 10px 0; border-top: 1px solid #333;">WireWinder Firmware v)rawliteral" FIRMWARE_VERSION R"rawliteral(</footer>
</body>
</html>
)rawliteral";
	server.send(200, "text/html", html);
}

void handleStatus() {
	// Read current pulse values (for diagnostics)
	unsigned long rawPulse1, rawPulse2;
	if (controlMode == CONTROL_WEB) {
		rawPulse1 = webCommandInput1;
		rawPulse2 = webCommandInput2;
	} else {
		rawPulse1 = pulseIn(PIN_INPUT1, HIGH, 25000UL);
		rawPulse2 = pulseIn(PIN_INPUT2, HIGH, 25000UL);
	}

	// Build JSON response
	String json = "{";
	json += "\"state\":" + String((int)currentState) + ",";
	json += "\"armed\":true,";
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
	json += "\"controlMode\":\"" + String(controlMode == CONTROL_WEB ? "web" : "pwm") + "\",";
	json += "\"errorMessage\":\"" + lastErrorMessage + "\",";
	json += "\"uptimeSec\":" + String(millis() / 1000) + ",";
	uint8_t apSeqProgress = 0;
	if (apSequenceSource == "INPUT1") {
		apSeqProgress = maxMinSequenceCount;
	} else if (apSequenceSource == "INPUT2") {
		apSeqProgress = maxMinSequenceCount2;
	}
	json += "\"apSeqSource\":\"" + apSequenceSource + "\",";
	json += "\"apSeqStatus\":\"" + apSequenceStatus + "\",";
	json += "\"apSeqLastCommand\":\"" + apLastCapturedCommand + "\",";
	json += "\"apSeqStarted\":" + String(apSequenceStarted ? "true" : "false") + ",";
	json += "\"apEnableCommandCaptured\":" + String(apEnableCommandCaptured ? "true" : "false") + ",";
	json += "\"apSeqProgress\":" + String(apSeqProgress) + ",";
	json += "\"apSeqEventMs\":" + String(apSequenceEventMs) + ",";
	// WiFi mode and connection info
	json += "\"wifiMode\":\"" + String(WiFi.getMode() == WIFI_STA ? "sta" : "ap") + "\",";
	if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
		json += "\"wifiConnected\":true,";
		json += "\"wifiSSID\":\"" + WiFi.SSID() + "\",";
		json += "\"wifiIP\":\"" + WiFi.localIP().toString() + "\"";
	} else if (WiFi.getMode() == WIFI_AP) {
		json += "\"wifiConnected\":false,";
		json += "\"apSSID\":\"" + String(AP_SSID) + "\",";
		json += "\"apIP\":\"" + WiFi.softAPIP().toString() + "\"";
	} else {
		json += "\"wifiConnected\":false";
	}
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

	if (cmd == "wind") {
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
		// Clear error state so test mode always engages
		currentState = STATE_IDLE;
		errorFromState = STATE_IDLE;
		allowWinding = true;
		allowUnwinding = true;
		webCommandInput1 = 1000; // Winding value
		webCommandInput2 = INPUT_NEUTRAL;
		lastWebCommandTime = millis();
		message = "Test winding started (no limits)";
	} else if (cmd == "test_unwind") {
		// Test unwinding (no revolution limits)
		controlMode = CONTROL_WEB;
		testMode = true;
		// Clear error state so test mode always engages
		currentState = STATE_IDLE;
		errorFromState = STATE_IDLE;
		allowWinding = true;
		allowUnwinding = true;
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
		webCommandInput2 = 2000; // PWM value above neutral to trigger open
		lastWebCommandTime = millis();
		message = "Gate opening";
	} else if (cmd == "close") {
		// Close gate (input2 below neutral)
		controlMode = CONTROL_WEB;
		webCommandInput1 = INPUT_NEUTRAL; // Stop motor
		webCommandInput2 = 1000; // PWM value below neutral to trigger close
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
		json += "\"maxRevolutions\":" + String(maxRevolutions, 2) + ",";
		json += "\"maxMotorRevolutions\":" + String(maxRevolutions * STEPPER_GEAR_RATIO, 2) + ",";
		json += "\"gearRatio\":" + String(STEPPER_GEAR_RATIO, 2);
		json += "}";
		server.send(200, "application/json", json);
	} else if (server.method() == HTTP_POST) {
		// Update settings
		bool changed = false;
		bool servoSettingsChanged = false;
		String message = "Settings updated";

		if (server.hasArg("servo2Open")) {
			uint16_t val = server.arg("servo2Open").toInt();
			if (val >= 500 && val <= 2500) {
				servo2OpenPos = val;
				changed = true;
				servoSettingsChanged = true;
			}
		}

		if (server.hasArg("servo2Closed")) {
			uint16_t val = server.arg("servo2Closed").toInt();
			if (val >= 500 && val <= 2500) {
				servo2ClosedPos = val;
				changed = true;
				servoSettingsChanged = true;
			}
		}

		if (server.hasArg("maxRevolutions")) {
			float val = server.arg("maxRevolutions").toFloat();
			if (val > 0 && val <= 100) {
				maxRevolutions = val;
				changed = true;
			}
		}

		if (server.hasArg("gearRatio")) {
			float val = server.arg("gearRatio").toFloat();
			if (val > 0 && val <= 100) {
				STEPPER_GEAR_RATIO = val;
				// Recalculate STEPS_PER_OUTPUT_REV when gear ratio changes
				STEPS_PER_OUTPUT_REV = (float)STEPS_PER_REV * STEPPER_GEAR_RATIO;
				changed = true;
			}
		}

		// Save to non-volatile memory if settings changed
		if (changed) {
			preferences.begin("wirewinder", false);
			preferences.putUShort("servo2Open", servo2OpenPos);
			preferences.putUShort("servo2Closed", servo2ClosedPos);
			preferences.putFloat("maxRevs", maxRevolutions);
			preferences.putFloat("gearRatio", STEPPER_GEAR_RATIO);
			preferences.end();
			dbgPrintln("Settings saved to non-volatile memory");

			// If servo settings changed, re-initialize servo to closed position
			if (servoSettingsChanged) {
				servo2.writeMicroseconds(servo2ClosedPos);
				currentServo2Pulse = servo2ClosedPos;
				dbgPrintf("Servo re-initialized to closed position: %u μs\n", servo2ClosedPos);
			}
		}

		String response = "{\"success\":" + String(changed ? "true" : "false") +
		                  ",\"message\":\"" + message + "\"" +
		                  ",\"maxRevolutions\":" + String(maxRevolutions, 2) +
		                  ",\"maxMotorRevolutions\":" + String(maxRevolutions * STEPPER_GEAR_RATIO, 2) +
		                  ",\"gearRatio\":" + String(STEPPER_GEAR_RATIO, 2) + "}";
		server.send(200, "application/json", response);
	}
}

// Firmware update handler - check for updates
void handleFirmwareCheck() {
	if (server.method() == HTTP_GET) {
		// Return current firmware status
		String json = "{";
		json += "\"currentVersion\":\"" + String(FIRMWARE_VERSION) + "\",";
		json += "\"availableVersion\":\"" + availableFirmwareVersion + "\",";
		json += "\"updateAvailable\":" + String(firmwareUpdateAvailable ? "true" : "false") + ",";
		json += "\"status\":\"" + firmwareUpdateStatus + "\",";
		json += "\"wifiConnected\":" + String(WiFi.isConnected() ? "true" : "false");
		json += "}";
		server.send(200, "application/json", json);
	} else if (server.method() == HTTP_POST) {
		// Trigger firmware check
		checkFirmwareUpdate();
		String json = "{\"status\":\"Checking for updates...\"}";
		server.send(200, "application/json", json);
	}
}

// Firmware update handler - download and install
void handleFirmwareUpdate() {
	if (server.method() == HTTP_POST) {
		if (!firmwareUpdateAvailable) {
			server.send(400, "application/json", "{\"error\":\"No update available\"}");
			return;
		}

		if (currentState != STATE_IDLE) {
			server.send(400, "application/json", "{\"error\":\"System must be IDLE to update\"}");
			return;
		}

		// Start firmware update
		bool success = downloadAndUpdateFirmware();
		
		if (success) {
			server.send(200, "application/json", "{\"status\":\"Update successful, restarting...\"}");
		} else {
			String json = "{\"error\":\"Update failed\",\"status\":\"" + firmwareUpdateStatus + "\"}";
			server.send(400, "application/json", json);
		}
	} else {
		server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
	}
}

// WiFi configuration handler
void handleWiFiConfig() {
	if (server.method() == HTTP_GET) {
		// Return current WiFi settings and status
		String json = "{";
		json += "\"ssid\":\"" + String(wifiSSID) + "\",";
		json += "\"password\":\"" + String(wifiPassword) + "\",";
		json += "\"wifiConnected\":" + String(WiFi.isConnected() ? "true" : "false") + ",";
		json += "\"wifiIP\":\"" + (WiFi.isConnected() ? WiFi.localIP().toString() : "Not connected") + "\",";
		json += "\"wifiMode\":\"" + String(WiFi.getMode() == WIFI_STA ? "STA" : (WiFi.getMode() == WIFI_AP ? "AP" : "OFF")) + "\",";
		json += "\"connectionStatus\":\"" + wifiConnectionStatus + "\"";
		json += "}";
		server.send(200, "application/json", json);
	} else if (server.method() == HTTP_POST) {
		// Update WiFi settings
		bool changed = false;
		String newSSID = "";
		String newPassword = "";

		if (server.hasArg("ssid")) {
			newSSID = server.arg("ssid");
			if (newSSID.length() > 0 && newSSID.length() < 64) {
				newSSID.toCharArray(wifiSSID, sizeof(wifiSSID));
				changed = true;
				dbgPrintf("[WiFi] SSID changed to: %s\n", wifiSSID);
			}
		}

		if (server.hasArg("password")) {
			newPassword = server.arg("password");
			if (newPassword.length() >= 8 && newPassword.length() < 64) {
				newPassword.toCharArray(wifiPassword, sizeof(wifiPassword));
				changed = true;
				dbgPrintf("[WiFi] Password changed (length: %d)\n", strlen(wifiPassword));
			} else if (newPassword.length() > 0) {
				server.send(400, "application/json", "{\"error\":\"Password must be at least 8 characters\",\"success\":false}");
				return;
			}
		}

		if (changed) {
			// Save to preferences
			preferences.begin("wirewinder", false);
			preferences.putString("wifiSSID", String(wifiSSID));
			preferences.putString("wifiPassword", String(wifiPassword));
			preferences.end();
			dbgPrintln("[WiFi] Settings saved to preferences");
			
			wifiConfigChanged = true;
			wifiConnectionStatus = "Reconnecting...";

			// Disconnect and reconnect to apply new credentials
			WiFi.disconnect(true);  // Turn off WiFi
			delay(500);
			
			String response = "{\"success\":true,\"message\":\"WiFi settings saved. Reconnecting...\"}";
			server.send(200, "application/json", response);
		} else {
			server.send(400, "application/json", "{\"error\":\"No settings provided\",\"success\":false}");
		}
	} else {
		server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
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
	STEPPER_GEAR_RATIO = preferences.getFloat("gearRatio", 25.0f); // default 25.0
	
	// Load WiFi credentials from preferences
	String storedSSID = preferences.getString("wifiSSID", "");
	String storedPassword = preferences.getString("wifiPassword", "");
	preferences.end();

	// Use stored credentials or defaults
	if (storedSSID.length() > 0 && storedSSID.length() < 64) {
		storedSSID.toCharArray(wifiSSID, sizeof(wifiSSID));
	} else {
		strcpy(wifiSSID, DEFAULT_WIFI_SSID);
	}

	if (storedPassword.length() > 0 && storedPassword.length() < 64) {
		storedPassword.toCharArray(wifiPassword, sizeof(wifiPassword));
	} else {
		strcpy(wifiPassword, DEFAULT_WIFI_PASSWORD);
	}

	// Recalculate STEPS_PER_OUTPUT_REV based on loaded gear ratio
	STEPS_PER_OUTPUT_REV = (float)STEPS_PER_REV * STEPPER_GEAR_RATIO;

	dbgPrintf("Loaded settings: servo2Open=%u servo2Closed=%u maxRevs=%.1f gearRatio=%.2f\n",
	          servo2OpenPos, servo2ClosedPos, maxRevolutions, STEPPER_GEAR_RATIO);
	dbgPrintf("Loaded WiFi: SSID=%s\n", wifiSSID);

	// Connect to WiFi
	dbgPrintf("Connecting to WiFi: %s\n", wifiSSID);
	WiFi.mode(WIFI_STA);
	WiFi.begin(wifiSSID, wifiPassword);

	unsigned long wifiStartTime = millis();
	// Try to connect for 60 seconds (1 minute)
	while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 60000) {
		delay(500);
		Serial.print(".");
	}

	if (WiFi.status() == WL_CONNECTED) {
		dbgPrintln("\nWiFi connected!");
		dbgPrintf("IP address: %s\n", WiFi.localIP().toString().c_str());
		dbgPrintf("Open http://%s in your browser\n", WiFi.localIP().toString().c_str());
	} else {
		// Failed to connect - create Access Point
		dbgPrintln("\nWiFi connection failed! Creating Access Point...");
		WiFi.mode(WIFI_AP);
		bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);

		if (apStarted) {
			IPAddress apIP = WiFi.softAPIP();
			dbgPrintln("Access Point created successfully!");
			dbgPrintf("AP SSID: %s\n", AP_SSID);
			dbgPrintf("AP Password: %s\n", AP_PASSWORD);
			dbgPrintf("AP IP address: %s\n", apIP.toString().c_str());
			dbgPrintf("Open http://%s in your browser\n", apIP.toString().c_str());
		} else {
			dbgPrintln("Failed to create Access Point!");
		}
	}

	// Setup web server routes
	server.on("/", handleRoot);
	server.on("/status", handleStatus);
	server.on("/command", handleCommand);
	server.on("/settings", handleSettings);
	server.on("/firmware/check", handleFirmwareCheck);
	server.on("/firmware/update", handleFirmwareUpdate);
	server.on("/wifi/config", handleWiFiConfig);
	server.begin();
	dbgPrintln("Web server started");

	// Initialize OTA (Over-The-Air) updates
	if (WiFi.isConnected()) {
		ArduinoOTA.setHostname("wirewinder");
		ArduinoOTA.setRebootOnSuccess(true);
		ArduinoOTA.onStart([]() {
			String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
			dbgPrintf("[OTA] Starting OTA update for %s\n", type.c_str());
			firmwareUpdateStatus = "OTA update started...";
		});
		ArduinoOTA.onEnd([]() {
			dbgPrintln("[OTA] OTA update completed!");
			firmwareUpdateStatus = "OTA update completed, restarting...";
		});
		ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
			int percentage = (progress / (total / 100));
			firmwareUpdateStatus = "OTA Progress: " + String(percentage) + "%";
		});
		ArduinoOTA.onError([](ota_error_t error) {
			dbgPrintf("[OTA] OTA Error[%u]: ", error);
			if (error == OTA_AUTH_ERROR) {
				dbgPrintln("Auth Failed");
				firmwareUpdateStatus = "OTA Auth Failed";
			} else if (error == OTA_BEGIN_ERROR) {
				dbgPrintln("Begin Failed");
				firmwareUpdateStatus = "OTA Begin Failed";
			} else if (error == OTA_CONNECT_ERROR) {
				dbgPrintln("Connect Failed");
				firmwareUpdateStatus = "OTA Connect Failed";
			} else if (error == OTA_RECEIVE_ERROR) {
				dbgPrintln("Receive Failed");
				firmwareUpdateStatus = "OTA Receive Failed";
			} else if (error == OTA_END_ERROR) {
				dbgPrintln("End Failed");
				firmwareUpdateStatus = "OTA End Failed";
			}
		});
		ArduinoOTA.begin();
		dbgPrintln("[OTA] OTA updates enabled");
	}

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

	// Initialize servo position variable BEFORE attaching to prevent glitches
	currentServo2Pulse = servo2ClosedPos;

	// Attach servo2 with wide pulse range (500-2500 µs) and write IMMEDIATELY to prevent glitches
	servo2.attach(PIN_SERVO2, 500, 2500);
	servo2.writeMicroseconds(servo2ClosedPos);

	dbgPrintf("Servo initialized to closed position: %u µs\n", servo2ClosedPos);

	// Initialize revolution counter at maxRevolutions (fully wound)
	// NOTE: This assumes the physical system starts in a fully wound state
	// The operator must ensure the system is fully wound before powering on
	revolutionCounter = maxRevolutions;
	dbgPrintf("Revolution counter initialized at %.2f (fully wound, available for unwinding)\n", revolutionCounter);

	// Initialize stepper position tracking
	if (stepper) {
		lastStepperPosition = stepper->getCurrentPosition();
		dbgPrintf("Stepper position tracking initialized at %ld steps\n", lastStepperPosition);
	}

	lastStepperUpdateTime = millis();
}

void loop() {
	// Handle web server requests
	server.handleClient();

	// Handle OTA updates
	if (WiFi.isConnected()) {
		ArduinoOTA.handle();
	}

	const unsigned long now = millis();

	// Handle WiFi reconnection if config changed
	static unsigned long lastWiFiReconnectAttempt = 0;
	if (wifiConfigChanged && (now - lastWiFiReconnectAttempt > 5000)) {
		// Try to reconnect with new credentials
		if (!WiFi.isConnected()) {
			dbgPrintf("[WiFi] Attempting to connect with new settings: %s\n", wifiSSID);
			WiFi.mode(WIFI_STA);
			WiFi.begin(wifiSSID, wifiPassword);
			lastWiFiReconnectAttempt = now;
			wifiConnectionStatus = "Reconnecting...";
		} else {
			dbgPrintln("[WiFi] Connected with new credentials!");
			wifiConfigChanged = false;
			wifiConnectionStatus = "Connected";
		}
	}

	// Update WiFi connection status
	if (WiFi.isConnected()) {
		wifiConnectionStatus = "Connected";
	} else if (WiFi.getMode() == WIFI_STA) {
		wifiConnectionStatus = "Connecting...";
	}

	// Check for firmware updates periodically (every FIRMWARE_UPDATE_CHECK_INTERVAL milliseconds)
	if (WiFi.isConnected() && (now - lastFirmwareCheckTime > FIRMWARE_UPDATE_CHECK_INTERVAL)) {
		checkFirmwareUpdate();
	}

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
	if (currentState == STATE_ERROR) {
		targetLedMode = LED_FAST;
	} else if (currentState == STATE_IDLE) {
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

	// Process INPUT1 for winding motor control
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

	processApSequenceInput1(now);

	// Determine state based on INPUT1
	SystemState newState = STATE_IDLE;
	if (!isInDeadzone(lastInput1Value)) {
		if (lastInput1Value > INPUT_NEUTRAL) {
			newState = STATE_UNWINDING;
		} else {
			newState = STATE_WINDING;
		}
	}

	// Update allow flags based on revolution counter
	if (testMode) {
		allowWinding = true;
		allowUnwinding = true;
	} else {
		allowWinding = (revolutionCounter < maxRevolutions);
		allowUnwinding = (revolutionCounter > 0);
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
				lastErrorMessage = "";
			}
		} else {
			if (newState == STATE_IDLE) {
					dbgPrintln("State: IDLE");
				setStepperEnable(false);
				currentState = STATE_IDLE;
			} else if (newState == STATE_WINDING) {
				if (!allowWinding) {
						dbgPrintln("Transition to WINDING blocked (direction not allowed)");
					// ignore transition
				} else {
					dbgPrintln("State: WINDING");
					lastStepperUpdateTime = now;
					currentState = STATE_WINDING;
				}
			} else if (newState == STATE_UNWINDING) {
				if (!allowUnwinding) {
						dbgPrintln("Transition to UNWINDING blocked (direction not allowed)");
					// ignore transition
				} else {
					dbgPrintln("State: UNWINDING");
					lastStepperUpdateTime = now;
					currentState = STATE_UNWINDING;
				}
			}
		}
	}

	// Control stepper motor (winding/unwinding)
	if (currentState == STATE_WINDING || currentState == STATE_UNWINDING) {
		// Update stepper speed periodically
		if (now - lastStepperUpdateTime >= STEPPER_UPDATE_INTERVAL) {
			// Calculate target speed from input
			int32_t targetSpeed = computeStepperSpeedFromInput(lastInput1Value);

			// If direction blocked, stop motor
			if ((targetSpeed > 0 && !allowUnwinding) || (targetSpeed < 0 && !allowWinding)) {
				targetSpeed = 0;
			}

			// Update stepper speed
			updateStepperSpeed(targetSpeed);

			// Update revolution counter based on actual stepper position
			if (stepper) {
			int32_t currentStepperPosition = stepper->getCurrentPosition();
			int32_t stepsDelta = currentStepperPosition - lastStepperPosition;

			if (stepsDelta != 0) {
				// Convert steps to output revolutions (accounting for gear ratio)
				float dRev = (float)stepsDelta / STEPS_PER_OUTPUT_REV;

				// Positive stepsDelta = forward (unwinding), negative = backward (winding)
				// Unwinding decreases counter, winding increases counter
				revolutionCounter -= dRev;

				// Clamp to limits [0, maxRevolutions]
				if (revolutionCounter > maxRevolutions) revolutionCounter = maxRevolutions;
				if (revolutionCounter < 0) revolutionCounter = 0;

				// Update last position
				lastStepperPosition = currentStepperPosition;

				// Check revolution limits immediately after updating counter (skip in test mode)
				if (!testMode) {
					if (currentState == STATE_WINDING && revolutionCounter >= maxRevolutions) {
						dbgPrintf("STOP: Maximum revolutions reached (%.2f)\n", maxRevolutions);
						lastErrorMessage = "";
						allowWinding = false;
						allowUnwinding = true;
						currentState = STATE_IDLE;
						setStepperHold();
						if (controlMode == CONTROL_WEB) {
							testMode = false;
							webCommandInput1 = INPUT_NEUTRAL;
							webCommandInput2 = INPUT_NEUTRAL;
							lastWebCommandTime = millis();
						}
					} else if (currentState == STATE_UNWINDING && revolutionCounter <= 0) {
						dbgPrintf("STOP: Minimum revolutions reached (0.0)\n");
						lastErrorMessage = "";
						allowUnwinding = false;
						allowWinding = true;
						currentState = STATE_IDLE;
						setStepperHold();
						if (controlMode == CONTROL_WEB) {
							testMode = false;
							webCommandInput1 = INPUT_NEUTRAL;
							webCommandInput2 = INPUT_NEUTRAL;
							lastWebCommandTime = millis();
						}
					}
				}
			}
		}

			lastStepperUpdateTime = now;
		}
	} else if (currentState == STATE_ERROR || currentState == STATE_IDLE) {
		// Stop motor but keep holding torque in error or idle state
		setStepperHold();
		lastStepperUpdateTime = now;
	}

	// Track INPUT2 and control SERVO2 only when deviating from neutral (deadzone)
	// In WEB mode, accept all servo values (500-2500) directly
	// In PWM mode, accept only pulses in valid control range [800..2000]
	if (controlMode == CONTROL_WEB) {
		// Web mode: trust commanded values (already validated in settings 500-2500)
		lastInput2Value = pulse2;
	} else if (pulseInAcceptRange(pulse2, 800, 2000)) {
		// PWM mode: accept only valid range
		lastInput2Value = pulse2;
	} else if (pulse2 != 0) {
		// Log out-of-range pulses for diagnostics (PWM mode only)
		static unsigned long lastOORLog2 = 0;
		if (now - lastOORLog2 > 1000) {
			dbgPrintf("WARN: Input2 out of range: %luus (accepted range: 800-2000)\n", pulse2);
			lastOORLog2 = now;
		}
	}

	processApSequenceInput2(now);

	// Control SERVO2 (gate) - change only when INPUT2 crosses deadzone thresholds
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

	// Diagnostics
	static unsigned long lastPrint = 0;
	if (now - lastPrint > 500) {
		dbgPrintf("Mode:%s State:%d Raw[%lu,%lu] Filt[%u,%u] Revs:%.2f/%.2f Speed:%ld S2:%u\n",
				  controlMode == CONTROL_WEB ? "WEB" : "PWM",
				  currentState, pulse1, pulse2, lastInput1Value, lastInput2Value,
				  revolutionCounter, maxRevolutions, currentStepperSpeed, currentServo2Pulse);
		lastPrint = now;
	}

	// Step pulses handled by FastAccelStepper
}




