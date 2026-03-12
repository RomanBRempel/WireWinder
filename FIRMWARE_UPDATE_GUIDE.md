# WireWinder Firmware Update & Network Configuration Guide

## Overview

The WireWinder now supports:
- **Automatic firmware updates** via HTTP download
- **WiFi network configuration** through web UI with persistent storage
- **Over-the-air (OTA) updates** via Arduino IDE

You can store firmware on Google Drive, a web server, or any HTTP-accessible location.

## Features

- ✅ Automatic firmware version checking
- ✅ Over-the-air (OTA) updates from web UI  
- ✅ ArduinoOTA support for IDE updates
- ✅ Safe update process (stops motors, verifies checksums)
- ✅ Progress tracking and status feedback
- ✅ Easy Google Drive integration
- ✅ **WiFi network configuration via web UI** (NEW!)
- ✅ **Persistent network settings** stored in device memory (NEW!)
- ✅ **Multiple network support** - easy to switch between locations (NEW!)

## Configuration

### 1. On Google Drive (Firmware Storage)

#### Step 1: Create a folder for firmware
1. Go to [Google Drive](https://drive.google.com)
2. Create a new folder called `wirewinder-firmware` or similar
3. Note the folder ID from the URL: `https://drive.google.com/drive/folders/{FOLDER_ID}`

#### Step 2: Upload files to Google Drive

Upload these files to the folder:

**a) version.json** - Contains version information
```json
{
  "version": "1.1.0",
  "description": "Updated motor control with improved safety features",
  "date": "2026-03-12"
}
```

**b) wirewinder.bin** - The compiled firmware binary
- Build using PlatformIO: `platformio run --environment esp32dev`
- Binary location: `.pio/build/esp32dev/firmware.bin`
- Rename to `wirewinder.bin`

#### Step 3: Get direct download links

For each file, right-click → "Get link" → Change to "Viewer" access

The link will be: `https://drive.google.com/file/d/{FILE_ID}/view?usp=share_link`

**Convert to direct download URL:**
```
https://drive.google.com/uc?export=download&id={FILE_ID}
```

### 2. In WireWinder Code

Edit `src/main.cpp` and update these constants (around line 24):

```cpp
// Firmware update server (configure to your server or Google Drive)
const char* FIRMWARE_SERVER_URL = "https://drive.google.com/uc?export=download&id={FOLDER_ID}/";
const char* FIRMWARE_MANIFEST_FILE = "version.json";  // Server should have: version.json, wirewinder.bin
```

**For Google Drive example:**
```cpp
const char* FIRMWARE_SERVER_URL = "https://drive.google.com/uc?export=download&id=1a2b3c4d5e6f7g8h9i0j/";
```

Or create a simple redirection server that maps file names to Google Drive download URLs.

### 3. Alternative: Use a Simple Web Server

If you have a web server, upload the files and set:

```cpp
const char* FIRMWARE_SERVER_URL = "http://your-domain.com/firmware/wirewinder/";
```

Directory structure:
```
/firmware/wirewinder/
├── version.json
└── wirewinder.bin
```

## Updating Firmware

### Via Web UI

1. Open WireWinder web interface: `http://<device-ip>/`
2. Go to "Firmware Management" section
3. Click **"Check Updates"** button
4. If update available, click **"Download & Install Update"**
5. System will automatically restart with new firmware

### Via Arduino IDE (OTA)

1. In Arduino IDE: Tools → Port → Network ports
2. Select `wirewinder` device
3. Upload sketch as usual
4. Update happens wirelessly

## Network Configuration

### Configuring WiFi Settings

The WireWinder can now be configured to connect to different WiFi networks without code changes!

#### Via Web UI (Recommended)

1. Open WireWinder web interface: `http://<device-ip>/`
2. Scroll down to **"Network Settings"** panel
3. Enter WiFi network name (SSID) in the **"WiFi Network (SSID)"** field
4. Enter WiFi password (minimum 8 characters) in the **"WiFi Password"** field
5. Click **"Load"** button to verify current settings
6. Click **"Save"** button to apply new settings
7. Device will reconnect to the new network within 10 seconds

#### Default Settings

- **Default SSID:** RBR_WiFi
- **Default Password:** 87770759

These defaults are used if no custom settings are saved.

#### Persistent Storage

WiFi credentials are automatically saved to the device's non-volatile memory (flash storage). They will be retained even after power loss or firmware updates (unless factory reset is performed).

### Using Different Networks

You can easily switch between different WiFi networks:

**For home:** 
- SSID: `HomeNetwork`
- Password: `homepassword123`

**For workshop:**
- SSID: `WorkshopNetwork`
- Password: `workshoppassword123`

Simply update the Network Settings panel and click Save. The device remembers the last configured network and connects automatically on startup.

### Access Point Mode

If WiFi connection fails (e.g., wrong password or network not available), the device will create an Access Point (AP) for 2 minutes, allowing direct connection:

- **AP SSID:** WireWinder
- **AP Password:** 12345678

You can then connect to the device directly and reconfigure WiFi settings.

### Troubleshooting WiFi Connection

**"Reconnecting..." stays too long:**
- Check if SSID and password are correct
- Verify WiFi network is in range
- Check if network uses special characters that might cause encoding issues
- Try using the Access Point mode to reconfigure

**Can't find WiFi network in AP mode:**
- Move device closer to router
- Check WiFi router is powered on
- Restart device if connection times out

**Device lost connectivity:**
- Check WiFi network status (network down, SSID changed, etc.)
- Use AP mode to reconfigure and save new settings
- Device will attempt to reconnect every 5 seconds


## Building Firmware Binary

To create the `.bin` file for distribution:

1. Open terminal in project root:
```bash
platformio run --environment esp32dev
```

2. Binary is created at:
```
.pio/build/esp32dev/firmware.bin
```

3. Copy to your firmware server (Google Drive or web server)

## Version Management

The version is defined in `src/main.cpp`:
```cpp
#define FIRMWARE_VERSION "1.0.0"
```

### Versioning scheme:
- **Major.Minor.Patch** (e.g., 1.0.0)
- Update this before building new firmware releases

## Troubleshooting

### "Update available but can't download"
- Check WiFi connection
- Verify Google Drive URL is correct
- Check that `version.json` exists on server
- Ensure device has enough free flash space

### "No update available"
- Version in `version.json` matches current firmware version
- Update the version number and reupload files

### Update fails midway
- Device may need power cycle
- Check WiFi signal strength
- Verify file integrity on server

### Can't check firmware from AP mode
- Access Point mode has no internet connection
- Connect to WiFi first, then check updates

## Status Endpoints

### Check firmware status:
```
GET http://device-ip/firmware/check
```

Response:
```json
{
  "currentVersion": "1.0.0",
  "availableVersion": "1.1.0",
  "updateAvailable": true,
  "status": "Update available: 1.1.0",
  "wifiConnected": true
}
```

### Trigger update:
```
POST http://device-ip/firmware/update
```

### Get WiFi configuration:
```
GET http://device-ip/wifi/config
```

Response:
```json
{
  "ssid": "RBR_WiFi",
  "password": "87770759",
  "wifiConnected": true,
  "wifiIP": "192.168.1.100",
  "wifiMode": "STA",
  "connectionStatus": "Connected"
}
```

### Update WiFi configuration:
```
POST http://device-ip/wifi/config
```

Parameters:
- `ssid` - WiFi network name (1-63 characters)
- `password` - WiFi password (8-63 characters)

Example:
```bash
curl -X POST "http://device-ip/wifi/config" \
  -d "ssid=MyNetwork&password=mypassword123"
```

Response (success):
```json
{
  "success": true,
  "message": "WiFi settings saved. Reconnecting..."
}
```

Response (error):
```json
{
  "error": "Password must be at least 8 characters",
  "success": false
}
```

## Safety Features

✅ **Motors disabled** during update
✅ **Checksum verification** of downloaded file
✅ **Automatic restart** on successful completion
✅ **Status feedback** during download
✅ **IDLE-only updates** to prevent accidental mid-operation updates
✅ **Password validation** (minimum 8 characters)
✅ **Persistent WiFi settings** stored in device flash

## Automatic Update Checking

By default, firmware is checked every **60 minutes** (configurable).

To change interval, edit in `main.cpp`:
```cpp
#define FIRMWARE_UPDATE_CHECK_INTERVAL 3600000  // milliseconds (1 hour)
```

Examples:
- 10 minutes: `600000`
- 30 minutes: `1800000`
- 4 hours: `14400000`

## Files Structure

```
WireWinder/
├── src/
│   └── main.cpp              (firmware source)
├── .pio/build/esp32dev/
│   └── firmware.bin          (compiled binary for upload)
├── platformio.ini            (build configuration)
└── FIRMWARE_UPDATE_GUIDE.md  (this file)
```

## Notes

- **Google Drive may have bandwidth limits** for high-volume downloads
- For production, consider using a dedicated server or cloud storage
- Keep version.json updated whenever firmware is released
- Test updates on dev device before public release

## Support

For issues or questions:
1. Check serial monitor for debug output
2. Review firmware check status in web UI
3. Verify files exist on storage server
4. Check Internet connectivity
