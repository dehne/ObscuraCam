/****
 * ObscuraCam v1.0.0
 * 
 * main.cpp
 * 
 * This is the Arduino framework-based firmware for a camera, the ObscuraCam, for Port Townsend's 
 * Mobile Camera Obscura, an enclosed trailer fitted with a wall in the rear containing an iris. 
 * The trailer is fitted with a door on the side to let people into the dark interior. Inside, on 
 * the at the end of the trailer opposite the iris is a screen. An image of whatever the back of 
 * the trailer points to is projected onto the screen. The ObscuraCam is mounted on a small shelf 
 * just above the iris and faces the screen.
 * 
 * The ObscuraCam presents the world with a Wifi Access Point with ssid "ObscuraCam" and password 
 * "CameraObscura". Attaching to the AP and pointing a browser at http://obscuracam.local brings 
 * up a page with a button on it. Clicking the button takes causes the ObscuraCam to take a photo 
 * of the screen it's pointed at. It then sends the result to the browser (and stores it on an SD 
 * card). That's it. 
 * 
 * The web server at the heart of this liberally borrows from the example from Expressif located 
 * at:
 * https://github.com/espressif/arduino-esp32/tree/master/libraries/WebServer/examples/SDWebServer
 * 
 * I've included the support from the example for viewing and manipulating files on the SD card. 
 * It was useful during development and should let us get the photos off the SD card without 
 * having to take the ObscuraCam apart.
 * 
 * The hardware for this is an ESP32-CAM MB module. It's just an AI Thinker ESP-CAM clone plugged 
 * into a mother board containing a USB-to-Serial chip and a couple of switches which I don't use. 
 * The clone is not exact, though, because it messes with the pins the origninal uses for the 
 * serial dtr and rts signals. As a result, platformio.ini file needs a couple of extra lines for 
 * things to work.
 * 
 ****
 *
 * Copyright 2024 by D.L. Ehnebuske
 * License: GNU Lesser General Public License v2.1
 * 
 * This is free software; you can redistribute it and/or modify it under the terms of the GNU 
 * Lesser General Public License as published by the Free Software Foundation; eitherversion 2.1 
 * of the License, or (at your option) any later version.
 * 
 * This code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without 
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details.
 * 
 ****/

#include "Arduino.h"                              // Arduino framework
#include "WiFi.h"                                 // WiFi support
#include "ESPmDNS.h"                              // mDNS support
#include "WebServer.h"                            // Web server support
#include "esp_camera.h"                           // Camera support
#include "sensor.h"                               // Camera sensor support
#include "FS.h"                                   // File system
#include "SD_MMC.h"                               // SD Card support
#include "esp_log.h"                              // log_?() support
#include "soc/soc.h"                              // Disable brownout checking
#include <EEPROM.h>                               // EEPROM access

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     (32)
#define RESET_GPIO_NUM    (-1)
#define XCLK_GPIO_NUM     (0)
#define SIOD_GPIO_NUM     (26)
#define SIOC_GPIO_NUM     (27)
#define Y9_GPIO_NUM       (35)
#define Y8_GPIO_NUM       (34)
#define Y7_GPIO_NUM       (39)
#define Y6_GPIO_NUM       (36)
#define Y5_GPIO_NUM       (21)
#define Y4_GPIO_NUM       (19)
#define Y3_GPIO_NUM       (18)
#define Y2_GPIO_NUM       (5)
#define VSYNC_GPIO_NUM    (25)
#define HREF_GPIO_NUM     (23)
#define PCLK_GPIO_NUM     (22)

// Misc compile-time definitions
#define BANNER            "\nObscuraCam v0.1.0\n"
#define IC_ADDR           (0)                       // Image counter address in "EEPROM"
#define SERIAL_MILLIS     (3000)                    // Millis to wait for Serial to become ready
#define AP_MILLIS         (100)                     // Millis to wait for AP to become ready
#define FLASH_MILLIS      (200)                     // LED_BUILTIN default flash length (millis())
#define FAIL_MILLIS       (1000)                    // millis() between flash groups for init failures
#define READY_FLASH_COUNT  (5)                      // Number of flashes to say hello/goodbye
#define SNAP_FLASH_COUNT  (1)                       // Number of times to flash on shutter release
#define CAMI_FLASH_COUNT  (2)                       // Number of times to flash if camera init fails
#define SDMI_FLASH_COUNT  (3)                       // Number of times to flash if SD card mount fails
#define SDCI_FLASH_COUNT  (4)                       // Number of times to flash if no SD card found
#define LED_BUILTIN       (GPIO_NUM_33)             // The GPIO for the little red LED (active LOW)
#define AWAKE_MILLIS      (300000UL)                // millis() to stay awake waiting for shutter press
#define PHOTO_PATH        "/photos/"                // The full path for dir where photos are to be kept
#define PHOTO_PREFIX      "Image"                   // The filename prefix for the photos taken
#define VIEW_URL_FRONT    "/view.htm?image="        // The first part of the url for the page to view the new pix

// AP- and Webserver-related constants
#define SSID              "ObscuraCam"              // The AP's SSID and the mDNS name for the web server
#define PASSWORD          "CameraObscura"           // The password needed to connect to the AP
#define LOCAL_IP          (192, 168, 1, 1)          // The IP of the web server
#define GATEWAY           (192, 168, 1, 1)          // The gateway
#define SUBNET            (255, 255, 255, 0)        // The subnet mask
#define PORT              (80)                      // The web server's port

// Global variables
WebServer server(PORT);                             // The web server
uint16_t imageCtr;                                  // The image counter for numbering image files
File uploadFile;                                    // File handle for uploading files

/**
 * @brief Flash the built-in little red LED
 * 
 * @param flashCount  Number of times to flash
 * @param flashLen    Length of flashes (millis())
 */
void flashBuiltinLed(uint8_t flashCount, uint8_t flashLen = FLASH_MILLIS) {
    for (uint8_t i = 0; i < flashCount; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(flashLen);
    digitalWrite(LED_BUILTIN, HIGH);
    if (i + 1 < flashCount) {
      delay(flashLen);
    }  
  }
}

/**
 * @brief   Convenience function to send http 200 response and no content
 * 
 */
void returnOK() {
  server.send(200, "text/plain", "");
}


/**
 * @brief   Convenience method to send http 500 failure + message response
 * 
 * @param msg The message to include in the response
 */
void returnFail(String msg) {
  server.send(500, "text/plain", msg + "\r\n");
}


/**
 * @brief   Send the contents of a file on the SD card as the response to an http GET request
 * 
 * @details A path ending with "/" serves index.htm (remember: SD card file names are 8.3). A file 
 *          ending in ".src" will have the ".src" stripped off and whatever is left will be served 
 *          as a "text/plain" file. So, e.g., asking for "/index.htm.src" will serve "/index.htm" 
 *          as a plain text file.
 * 
 *          Normally, the files are served labeled with their correct MIME data type. But, if the 
 *          http request includes the argument "download" the file will be labeled 
 *          "application/octet-stream".
 * 
 *          When successful, an HTTP 200 response is sent.
 * 
 * @param path    The complete path for the file to be served
 * @return true   Operation succeeded; file has been served
 * @return false  Nope. Couldn't find the file.
 */
bool loadFromSdCard(String path) {
  String dataType = "text/plain";
  if (path.endsWith("/")) {
    path += "index.htm";
  }

  log_d("Sending file: \"%s\"", path.c_str());

  if (path.endsWith(".src")) {
    path = path.substring(0, path.lastIndexOf("."));
  } else if (path.endsWith(".htm")) {
    dataType = "text/html";
  } else if (path.endsWith(".css")) {
    dataType = "text/css";
  } else if (path.endsWith(".js")) {
    dataType = "application/javascript";
  } else if (path.endsWith(".png")) {
    dataType = "image/png";
  } else if (path.endsWith(".gif")) {
    dataType = "image/gif";
  } else if (path.endsWith(".jpg")) {
    dataType = "image/jpeg";
  } else if (path.endsWith(".ico")) {
    dataType = "image/x-icon";
  } else if (path.endsWith(".xml")) {
    dataType = "text/xml";
  } else if (path.endsWith(".pdf")) {
    dataType = "application/pdf";
  } else if (path.endsWith(".zip")) {
    dataType = "application/zip";
  }

  File dataFile = SD_MMC.open(path.c_str());
  if (dataFile.isDirectory()) {
    path += "/index.htm";
    dataType = "text/html";
    dataFile = SD_MMC.open(path.c_str());
  }

  if (!dataFile) {
    log_i("File \"%s\" not found.", path.c_str());
    return false;
  }

  if (server.hasArg("download")) {
    dataType = "application/octet-stream";
  }
  size_t nSent = server.streamFile(dataFile, dataType);
  if (nSent != dataFile.size()) {
    log_e("Expected to send %d bytes, but %d were actually sent.", dataFile.size(), nSent);
  }

  dataFile.close();
  return true;
}

/**
 * @brief HTTP POST handler for use by /edit/index.htm. Haven't analyzed it.
 * 
 */
void handleFileUpload() {
  if (server.uri() != "/edit") {
    return;
  }
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (SD_MMC.exists((char *)upload.filename.c_str())) {
      SD_MMC.remove((char *)upload.filename.c_str());
    }
    uploadFile = SD_MMC.open(upload.filename.c_str(), FILE_WRITE);
    log_d("Upload: START, filename: %s", upload.filename.c_str());
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadFile.write(upload.buf, upload.currentSize);
    }
    log_d("Upload: WRITE, Bytes: %d", upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
    }
    log_d("Upload: END, Size: %d", upload.totalSize);
  }
}

/**
 * @brief function used by handleDelete. Haven't analyzed it.
 * 
 * @param path 
 */
void deleteRecursive(String path) {
  File file = SD_MMC.open((char *)path.c_str());
  if (!file.isDirectory()) {
    file.close();
    SD_MMC.remove((char *)path.c_str());
    return;
  }

  file.rewindDirectory();
  while (true) {
    File entry = file.openNextFile();
    if (!entry) {
      break;
    }
    String entryPath = path + "/" + entry.name();
    if (entry.isDirectory()) {
      entry.close();
      deleteRecursive(entryPath);
    } else {
      entry.close();
      SD_MMC.remove((char *)entryPath.c_str());
    }
    yield();
  }

  SD_MMC.rmdir((char *)path.c_str());
  file.close();
}

/**
 * @brief HTTP DELETE handler for /edit/index.htm. Haven't analyzed it.
 * 
 */
void handleDelete() {
  if (server.args() == 0) {
    return returnFail("BAD ARGS");
  }
  String path = server.arg(0);
  if (path == "/" || !SD_MMC.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }
  deleteRecursive(path);
  returnOK();
}

/**
 * @brief HTTP PUT handler for /edit/index.htm. Haven't analyzed it.
 * 
 */
void handleCreate() {
  if (server.args() == 0) {
    return returnFail("BAD ARGS");
  }
  String path = server.arg(0);
  if (path == "/" || SD_MMC.exists((char *)path.c_str())) {
    returnFail("BAD PATH");
    return;
  }

  if (path.indexOf('.') > 0) {
    File file = SD_MMC.open((char *)path.c_str(), FILE_WRITE);
    if (file) {
      file.write(0);
      file.close();
    }
  } else {
    SD_MMC.mkdir((char *)path.c_str());
  }
  returnOK();
}

/**
 * @brief HTTP GET handler for /list. Used by /edit/index.htm. Haven't analyzed it.
 * 
 */
void printDirectory() {
  if (!server.hasArg("dir")) {
    return returnFail("BAD ARGS");
  }
  String path = server.arg("dir");
  if (path != "/" && !SD_MMC.exists((char *)path.c_str())) {
    return returnFail("BAD PATH");
  }
  File dir = SD_MMC.open((char *)path.c_str());
  path = String();
  if (!dir.isDirectory()) {
    dir.close();
    return returnFail("NOT DIR");
  }
  dir.rewindDirectory();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/json", "");

  server.sendContent("[");
  for (int cnt = 0; true; ++cnt) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }

    String output;
    if (cnt > 0) {
      output = ',';
    }

    output += "{\"type\":\"";
    output += (entry.isDirectory()) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += entry.path();
    output += "\"";
    output += "}";
    server.sendContent(output);
    entry.close();
  }
  server.sendContent("]");
  dir.close();
}

/**
 * @brief HTTP GET handler for /snap. User's browser is redirected to this "page" when the user 
 *        clicks the "Take photo" button on /index.htm on on /view.htm. Here we take a photo and 
 *        store on the SD card. Once this is accomplished, the user's browser is redirected to 
 *        /view.htm?image=<path to stored image>, which displays the image for the user.
 * 
 */
void onSnap() {
  // Capture image
  camera_fb_t * fb = esp_camera_fb_get();  
  if(!fb) {
    returnFail("Camera capture failed.");
    return;
  }
  log_d("Got the framebuffer.");

  // Figure out what to call the image file
  String imageFilePath = String(PHOTO_PATH) + PHOTO_PREFIX + String(++imageCtr) + ".jpg";
  log_d("The file name for the image is '%s'.", imageFilePath.c_str());

  // Save the image
  File file = SD_MMC.open(imageFilePath.c_str(), FILE_WRITE);
  if(!file){
    returnFail("Unable to create the file for the image.");
    return;
  }
  size_t sz = file.write(fb->buf, fb->len);
  log_d("Saved image to: '%s' (%d bytes)", imageFilePath.c_str(), fb->len);
  EEPROM.writeUShort(IC_ADDR, imageCtr);
  EEPROM.commit();

  flashBuiltinLed(SNAP_FLASH_COUNT);
  log_d("Committed imageCtr (%d) to 'eeprom'.", imageCtr);

  // Clean up
  file.close();
  esp_camera_fb_return(fb);

  // Redirect request to the page that will show the new photo
  server.sendHeader("Location", VIEW_URL_FRONT + imageFilePath, true);
  server.send(302, "Found");
}

void onNotFound() {
  // Not a request for something handled programmatically; try to get it from the SD card
  if (loadFromSdCard(server.uri())) {
    return;
  }

  log_d("Handling page not found.");

  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
}

/**
 * @brief   Arduino setup function: Called once at power-on or reset
 * 
 */
void setup() {
  // Get Serial going
  Serial.begin(9600);
  delay(SERIAL_MILLIS);
  Serial.print(BANNER);
  Serial.setDebugOutput(true);

  // Initialize the builtin little red LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // It's active low

  // Initialize the AP
  WiFi.softAP(SSID, PASSWORD);
  IPAddress localIP LOCAL_IP;
  IPAddress gateway GATEWAY;
  IPAddress subnet SUBNET;
  WiFi.softAPConfig(localIP, gateway, subnet);
  delay(AP_MILLIS);

  // Initialize mDNS, setting the name to be the same as <our SSID>.local
  if (!MDNS.begin(SSID)) {
    log_w("mDNS initialization failed.");
  }

  // Register the request handlers
  server.on("/list", HTTP_GET, printDirectory);
  server.on("/edit", HTTP_DELETE, handleDelete);
  server.on("/edit", HTTP_PUT, handleCreate);
  server.on(
    "/edit", HTTP_POST,
    []() {
      returnOK();
    },
    handleFileUpload
  );
  server.on("/snap", HTTP_GET, onSnap);
  server.onNotFound(onNotFound);

  //Start the Web server
  server.begin();

  log_d("HTTP server started successfully.");

  // Set up the camera configuration we'll use
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  
  if(psramFound()){
    log_i("Using UXGA resolution.");
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    log_i("Using SVGA resolution because PSRAM not present.");
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Initialize the camera with the configuration we just set up
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    log_e("Camera init failed with error 0x%x.", err);
    while (true) {
      flashBuiltinLed(CAMI_FLASH_COUNT);
      delay(FAIL_MILLIS);
    }
  }

  // Tell the sensor to flip the image it sends upside down and left-to-right
  sensor_t *s = esp_camera_sensor_get();
  int sErr = s->set_hmirror(s, 1);
  if (sErr >= 0) {
    sErr = s->set_vflip(s, 1);
  }
  if (sErr < 0) {
    log_e("Flipping the camera sensor orientation failed.");
  }
  
  // Mount SD card
  if(!SD_MMC.begin("/sdcard", true)){
    log_e("SD Card Mount failed.");
    while (true) {
      flashBuiltinLed(SDMI_FLASH_COUNT);
      delay(FAIL_MILLIS);
    }
  }
  
  log_d("SD card mounted.");
  
  // Verify there's a card in it
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    log_e("No SD Card inserted.");
    while (true) {
      flashBuiltinLed(SDCI_FLASH_COUNT);
      delay(FAIL_MILLIS);
    }
  }
  log_d("The SD card reader seems to have a card in it.");
  
  // Get "EEPROM" going (it's really flash memory)
  EEPROM.begin(sizeof((uint16_t)0));

  // Uncomment to reset image counter in EEPROM to 0
  //EEPROM.writeUShort(IC_ADDR, (uint16_t)0);
  //EEPROM.commit();

  // Initialize the image counter
  imageCtr = EEPROM.readUShort(IC_ADDR);
  log_d("Last stored image was Image%d.jpg.", imageCtr);

  // Show we're ready
  flashBuiltinLed(READY_FLASH_COUNT);
  log_i("Initialization complete.");
}

/**
 * @brief   Arduino loop() function. Called repeatedly once setup() returns.
 * 
 */
void loop() {
  // Let the Web server do its thing
  server.handleClient();
  delay(2); // Relinquish control
}
