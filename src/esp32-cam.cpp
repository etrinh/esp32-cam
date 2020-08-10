/*********
  IMPORTANT BEFORE TO DOWNLOAD SKETCH !!!
   - Install ESP32 libraries
   - Select Board "ESP32 Wrover Module"
   - Select the Partion Scheme "Minimal SPIFFS"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
// https://RandomNerdTutorials.com
// https://github.com/zhouhan0126/WIFIMANAGER-ESP32 (patch required)
// https://github.com/bartlomiejcieszkowski/Micro-RTSP/tree/bcieszko-multi-client-rtsp
// 
#define NO_GLOBAL_ARDUINOOTA
#include <ArduinoOTA.h>
#include <WiFiManager.h>  // Need to patch to transfer to cpp some duplicated definitions with WebServer.h, make connectWifi public and change HTTP_HEAD to _HTTP_HEAD
#include <esp_pm.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"           //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include <WebServer.h>
#include "CRtspSession.h"
#include <WiFiClient.h>
#include <EEPROM.h>
#include <atomic>

#define MIN(a,b)  ((a)<(b)?(a):(b))
#define MAX(a,b)  ((a)>(b)?(a):(b))

#define ENABLE_RTSPSERVER
#define SERIAL_DEBUG true               // Enable / Disable log - activer / désactiver le journal
#define ESP_LOG_LEVEL ESP_LOG_NONE      // ESP_LOG_NONE, ESP_LOG_VERBOSE, ESP_LOG_DEBUG, ESP_LOG_ERROR, ESP_LOG_WARM, ESP_LOG_INFO

#define VERSION   "1.06"

#define FLASH_PIN 4

#define EEPROM_ORIENTATION_ADDRESS 0
#define EEPROM_ORIENTATION_FLIP_MASK 0x1
#define EEPROM_ORIENTATION_MIRROR_MASK 0x2

// Web server port - port du serveur web
#define WEB_SERVER_PORT 80
#define URI_STATIC_JPEG "/jpg/image.jpg"
#define URI_FLASH "/flash"
#define URI_ORIENTATION "/orientation"
#define URI_WIFI "/reset"
#define URI_REBOOT "/reboot"
#define URI_OTA "/ota"
#define URI_UPDATE "/update"
#define URI_STATUS "/status"
#define URI_ROOT "/"
#define URI_USAGE "/help"
#define URI_INFO "/info"
WebServer server ( WEB_SERVER_PORT );

#define OTA_TIMER (10*60)
#define FLASH_TIMER (60)
#define REBOOT_TIMER (3)
#define OTA_REBOOT_TIMER (1)

#define WATCHDOG_TIMEOUT  (2 * 60 * 1000 * 1000)
hw_timer_t *watchdog = NULL;

#define IMAGE_COMPRESSION 10   //0-63 lower number means higher quality - Plus de chiffre est petit, meilleure est la qualité de l'image, plus gros est le fichier
#define CAMERA_SNAPSHOT_PERIOD 200

#define TAG "esp32-cam"

// Uncomment your dev board model - Décommentez votre carte de développement
// This project was only tested with the AI Thinker Model - le croquis a été testé uniquement avec le modèle AI Thinker
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26     //Flash LED - Flash Light is On if SD card is present
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    15
#define XCLK_GPIO_NUM     27
#define SIOD_GPIO_NUM     25
#define SIOC_GPIO_NUM     23
#define Y9_GPIO_NUM       19
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM        5
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM       35
#define Y2_GPIO_NUM       32
#define VSYNC_GPIO_NUM    22
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26  //Flash LED - Flash Light is On if SD card is present
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#else
#error "Camera model not selected"
#endif

int64_t ledOnTimer = 0;

class BufferManager
{
    std::atomic<int> m_counter;
    camera_fb_t *m_fb = NULL;
    uint32_t m_lastFrame = 0;
public:
    BufferManager(): m_counter(0) {}
    camera_fb_t * get()
    {
        uint32_t now = millis();
        if (m_counter++ == 0 && now > m_lastFrame + CAMERA_SNAPSHOT_PERIOD) {
            if (ledOnTimer)
                digitalWrite(FLASH_PIN, 1);
            if (m_fb) {
                esp_camera_fb_return(m_fb);
            }
            m_fb = esp_camera_fb_get();
            digitalWrite(FLASH_PIN, 0);
            m_lastFrame = now;
        }
        return m_fb;
    }
    void release()
    {
        --m_counter;
    }
} CameraBuffer;

#ifdef ENABLE_RTSPSERVER
TaskHandle_t TaskRTSP;
class MyRtspServer
{
public:
  MyRtspServer(u_short width, u_short height, uint16_t port = 554): m_streamer(width, height), m_rtspServer(port) { m_lastImage = millis(); m_rtspServer.setNoDelay(true); }
  void begin() { m_rtspServer.begin(); }
  void end() { m_rtspServer.end(); }
  void setFrameRate(uint32_t msecPerFrame) { m_msecPerFrame = msecPerFrame; }
  int clientCount() {
    int count = 0;
    LinkedListElement* element = m_streamer.getClientsListHead()->m_Next;
    while(element != m_streamer.getClientsListHead()) {
      ++count;
      element = element->m_Next;
    }
    return count;
  }
  void handle()
  {
    if (!m_rtspServer) {
      return;
    }

    // If we have an active client connection, just service that until gone
    WiFiClient rtspClient = m_rtspServer.accept();
    if(rtspClient) {
      m_streamer.addSession(rtspClient);
    }

    // If we have an active client connection, just service that until gone
    m_streamer.handleRequests(0); // we don't use a timeout here,
    // instead we send only if we have new enough frames
    if(m_streamer.anySessions()) {
      uint32_t now = millis();
      if(now > m_lastImage + m_msecPerFrame || now < m_lastImage) { // handle clock rollover
        camera_fb_t * fb = CameraBuffer.get();
        if (fb) {
          m_streamer.setBuffer(fb);
          m_streamer.streamImage(now);
        }
        CameraBuffer.release();
        m_lastImage = now;
      }
    }
  }
protected:
  uint32_t m_msecPerFrame = CAMERA_SNAPSHOT_PERIOD;
  uint32_t m_lastImage;
  class MyStreamer : public CStreamer
  {
    camera_fb_t* m_buffer = NULL;
  public:
    MyStreamer(u_short width, u_short height): CStreamer(width, height) {}
    void setBuffer(camera_fb_t* buffer) { m_buffer = buffer; }
    void streamImage(uint32_t curMsec) { if (m_buffer) { streamFrame(m_buffer->buf, m_buffer->len, curMsec); } }
  };
  MyStreamer m_streamer;
  WiFiServer m_rtspServer;
} rtspServer(1600, 1200, 554);
#endif


int64_t rebootRequested = 0;
int64_t otaOnTimer = 0;

static void switchLed(bool enable)
{
  if (enable) {
    int64_t fr_start = esp_timer_get_time();
    ledOnTimer = fr_start + FLASH_TIMER * 1000 * 1000;
  }
  else {
    digitalWrite(FLASH_PIN, 0);
    ledOnTimer = 0;
  }
}

static void led_handler() {
    String param = server.arg("action");
    if (param == "on") {
        switchLed(true);
    }
    else if (param == "off") {
        switchLed(false);
    }
    else if (param == "toggle") {
        switchLed(ledOnTimer == 0);
    }
    server.send(200);
}

static void orientation_handler() {
    byte val = 0;
    String param = server.arg("mirror");
    if (param == "1" || param == "true") {
        val |= EEPROM_ORIENTATION_MIRROR_MASK;
    }
    param = server.arg("flip");
    if (param == "1" || param == "true") {
        val |= EEPROM_ORIENTATION_FLIP_MASK;
    }
    byte readVal = EEPROM.read(EEPROM_ORIENTATION_ADDRESS);
    if (readVal != val) {
        sensor_t * s = esp_camera_sensor_get();
        s->set_vflip(s, val & EEPROM_ORIENTATION_FLIP_MASK);
        s->set_hmirror(s, val & EEPROM_ORIENTATION_MIRROR_MASK);
        EEPROM.write(EEPROM_ORIENTATION_ADDRESS, val);
        EEPROM.commit();
    }
    server.send(200);
}

static void wifi_handler() {
    esp_wifi_restore();
    esp_restart();
    server.send(200);
}

static void reboot_handler() {
    rebootRequested = esp_timer_get_time() + REBOOT_TIMER * 1000 * 1000;
    server.send(200);
}

ArduinoOTAClass * OTA = NULL;

static void enableOTA(bool enable, bool forceCommit = false)
{
  if (enable) {
    if (OTA == NULL) {
      OTA = new ArduinoOTAClass();
      OTA->setHostname(TAG);
      OTA->setPasswordHash("913f9c49dcb544e2087cee284f4a00b7");   // MD5("device")
      OTA->begin();
      int64_t fr_start = esp_timer_get_time();
      otaOnTimer = fr_start + OTA_TIMER * 1000 * 1000; // 10 minutes
    }
  }
  else {
    if (OTA) {
      if (forceCommit) {
        delete OTA;
        OTA = NULL;
        otaOnTimer = 0;
      }
      else {
        otaOnTimer = 1; // to be disabled on next loop
      }
    }
  }
}

static void ota_handler() {
    String param = server.arg("action");      /* Get value of expected key from query string */
    if (param == "on") {
        enableOTA(true);
    }
    else if (param == "off") {
        enableOTA(false);
    }
    else if (param == "toggle") {
        enableOTA(OTA==NULL);
    }
    server.send(200);
}

static void update_handler() {
    timerWrite(watchdog, 0); //reset timer (feed watchdog)
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        vTaskDelete(TaskRTSP);
        ESP_LOGI(TAG, "Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
            ESP_LOGW(TAG, "%s", Update.errorString());
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            ESP_LOGE(TAG, "%s", Update.errorString());
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
            ESP_LOGI(TAG, "Update Success: %u\nRebooting...\n", upload.totalSize);
            rebootRequested = esp_timer_get_time() + 1000;
        } else {
            ESP_LOGE(TAG, "%s", Update.errorString());
        }
    }
}

static void usage_handler() {
    static const __FlashStringHelper* info =
F("<html>"
  "<head>"
    "<title>esp32-cam - API</title>"
    "<style>"
    "</style>"
  "</head>"
  "<body>"
    "<h1>" TAG "<span id=\"version\"> (v" VERSION ") - API Usage</span></h1>"
    "<ul>"
      "<li>Led on/off/toggle: " URI_FLASH "?action=[on|off|toggle]</li>"
      "<li>Flip/Mirror camera: " URI_ORIENTATION "?mirror=[0|false|1|true]&flip=[0|false|1|true]</li>"
      "<li>Snapshot JPEG: " URI_STATIC_JPEG "</li>"
      "<li>Reboot: " URI_REBOOT "</li>"
      "<li>Reset Wifi: " URI_WIFI "</li>"
      "<li>OTA on/off/toggle: " URI_OTA "?action=[on|off|toggle]</li>"
      "<li>Status (JSON): " URI_STATUS "</li>"
      "<li>RTSP Streaming: rtsp://&lt;IP&gt;/mjpeg/1</li>"
    "</ul>"
  "</body>"
"</html>");
    server.send(200, "text/html", info);
}

static void status_handler() {
    byte val = EEPROM.read(EEPROM_ORIENTATION_ADDRESS);
    int64_t time = esp_timer_get_time();
    String info = "{"
    "\"version\":\"" VERSION "\","
    "\"led\":\"" + String(ledOnTimer?"true":"false") + "\","
    "\"ledTimer\":\"" + String(MAX(0, int((ledOnTimer - time) / 1000000))) + "\","
    "\"flip\":\"" + String(val&(EEPROM_ORIENTATION_FLIP_MASK)?"1":"0") + "\","
    "\"mirror\":\"" + String(val&(EEPROM_ORIENTATION_MIRROR_MASK)?"1":"0") + "\","
    "\"ssid\":\"" + WiFi.SSID() + "\","
    "\"rssi\":\"" + String(WiFi.RSSI()) + "\","
    "\"ip\":\"" + WiFi.localIP().toString() + "\","
    "\"mac\":\"" + WiFi.macAddress() + "\","
    "\"chipId\":\"" + ESP_getChipId() + "\","
    "\"ota\":\"" + String(OTA?"true":"false") + "\","
    "\"otaTimer\":\"" + String(MAX(0, int((otaOnTimer - time) / 1000000))) + "\","
    "\"reboot\":\"" + String(rebootRequested>0?"true":"false") + "\","
    "\"rebootTimer\":\"" + String(MAX(0, int((rebootRequested - time) / 1000000))) + "\","
    "\"rtspClients\":\"" + String(rtspServer.clientCount()) + "\""
  "}";
    server.send(200, "text/json", info);
}

static void info_handler() {
  static const __FlashStringHelper* info =
F("<html>"
  "<head>"
    "<title>esp32-cam</title>"
    "<script type=\"text/javascript\">"
      "function invoke(url)"
      "{"
        "var xhr = new XMLHttpRequest();"
        "xhr.open(\"GET\", url, true);"
        "xhr.send(null);"
      "};"
      "function update()"
      "{"
        "var xhr = new XMLHttpRequest();"
        "xhr.open(\"GET\", \"" URI_STATUS "\", true);"
        "xhr.onload = function (e) {"
          "if (xhr.readyState === 4) {"
            "if (xhr.status === 200) {"
              "var obj = JSON.parse(xhr.responseText);"
              "document.getElementById(\"ssid\").innerHTML = obj.ssid;"
              "document.getElementById(\"rssi\").innerHTML = obj.rssi;"
              "document.getElementById(\"ip\").innerHTML = obj.ip;"
              "document.getElementById(\"mac\").innerHTML = obj.mac;"
              "document.getElementById(\"ledTimer\").innerHTML = obj.ledTimer>0?\" - \"+obj.ledTimer:\"\";"
              "document.getElementById(\"reboot\").innerHTML = obj.rebootTimer>0?\" - \"+obj.rebootTimer:\"\";"
              "document.getElementById(\"ota\").innerHTML = obj.ota==\"true\"?\" - On (\"+Math.round(obj.otaTimer/60)+\"min)\":\" - Off\";"
            "}"
          "}"
        "};"
        "xhr.send(null);"
      "};"
      "function preview()"
      "{"
        "document.getElementById(\"snapshot\").src = \"" URI_STATIC_JPEG "?random=\"+new Date().getTime() ;"
      "};"
      "update();"
      "setInterval(update, 2000);"
      "setInterval(preview, 5000);"
    "</script>"
    "<style>"
    ".snapshot {"
      "width: 500;"
    "}"
    "</style>"
  "</head>"
  "<body>"
    "<h1>" TAG "<span id=\"version\"> (v" VERSION ")</span></h1>"
    "<table style=\"height: 60px;\" width=\"100%\">"
      "<tbody>"
        "<tr>"
          "<td style=\"width: 50%;\">"
            "<span class=\"info\">SSID: </span><span id=\"ssid\"></span>"
            "<br/>"
            "<span class=\"info\">RSSI: </span><span id=\"rssi\"></span>"
            "<br/>"
            "<span class=\"info\">IP: </span><span id=\"ip\"></span>"
            "<br/>"
            "<span class=\"info\">MAC: </span><span id=\"mac\"></span>"
          "</td>"
          "<td style=\"width: 50%;\" rowspan=\"2\">"
            "<img class=\"snapshot\" id=\"snapshot\"/>"
          "</td>"
        "</tr>"
        "<tr>"
          "<td style=\"width: 50%;\">"
            "<a class=\"link\" href=\"\" onclick=\"invoke(\'" URI_FLASH "?action=on\');return false;\">Switch On Flash</a><span id=\"ledTimer\"></span>"
            "<br/>"
            "<a class=\"link\" href=\"\" onclick=\"invoke(\'" URI_FLASH "?action=off\');return false;\">Switch Off Flash</a>"
            "<br/>"
            "<a class=\"link\" href=\"\" onclick=\"invoke(\'" URI_FLASH "?action=toggle\');return false;\">Toggle Flash</a>"
            "<br/>"
            "<a class=\"link\" href=\"" URI_STATIC_JPEG "\">Snapshot</a>"
            "<br/>"
            "<a class=\"link\" href=\"\" onclick=\"invoke(\'" URI_REBOOT "\');return false;\">Reboot Device</a><span id=\"reboot\"></span>"
            "<br/>"
            "<a class=\"link\" href=\"\" onclick=\"invoke(\'" URI_WIFI "\');return false;\">Reset Device</a>"
            "<br/>"
            "<a class=\"link\"  href=\"\" onclick=\"invoke(\'" URI_OTA "?action=toggle\');return false;\">Toggle OTA</a><span id=\"ota\"></span>"
            "<br/>"
            "<form id=\"upgradeForm\" method=\"post\" enctype=\"multipart/form-data\" action=\"" URI_UPDATE "\"><span class=\"action\">Upgrade Firmware: </span><input type=\"file\" name=\"fileToUpload\" id=\"upgradeFile\" /><input type=\"submit\" value=\"Upgrade\" id=\"upgradeSubmit\"/></form>"
            "<br/>"
          "</td>"
        "</tr>"
      "</tbody>"
    "</table>"
    "<a href=\"" URI_USAGE "\">API Usage</a>"
  "</body>"
"</html>");
    server.send(200, "text/html", info);
}

/*
   This method only stream one JPEG image - Cette méthode ne publie qu'une seule image JPEG
   Compatible with/avec Jeedom / NextDom / Domoticz
*/
static void capture_handler() {
    ESP_LOGI(TAG, "Take a picture");
    camera_fb_t *fb = CameraBuffer.get();//esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        server.send(500);
    } else {
        server.sendHeader("Content-Disposition", "inline; filename=image.jpg");
        server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
    }
    CameraBuffer.release();
}

void startCameraServer() {
    server.on(URI_STATIC_JPEG, capture_handler);
    server.on(URI_FLASH, led_handler);
    server.on(URI_ORIENTATION, orientation_handler);
    server.on(URI_WIFI, wifi_handler);
    server.on(URI_REBOOT, reboot_handler);
    server.on(URI_OTA, ota_handler);
    server.on(URI_UPDATE, HTTP_POST, []() {
        String html = "<html>"
                        "<head>"
                        "<title>" TAG " - OTA</title>" +
                        String(!Update.hasError() ? "<meta http-equiv=\"refresh\" content=\"3; url=/\">" : "") +
                        "</head>"
                        "<body>Update " + (Update.hasError() ? "failed" : "succeeded") + "</body>"
                    "</html>";
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", html);
      }, update_handler );
    server.on(URI_STATUS, status_handler);
    server.on(URI_INFO, info_handler);
    server.on(URI_USAGE, usage_handler);
    server.on(URI_ROOT, info_handler);
    server.begin();
}

void setup() {
    watchdog = timerBegin(0, 80, true); //timer 0, div 80
    timerAttachInterrupt(watchdog, &esp_restart, true);
    timerAlarmWrite(watchdog, WATCHDOG_TIMEOUT, false); //set time in us
    timerAlarmEnable(watchdog); //enable interrupt

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

    pinMode (FLASH_PIN, OUTPUT);//back led

    // Stop Bluetooth
    btStop();

    // set frequency to 80Mhz
    esp_pm_config_esp32_t pm_config;
    pm_config.max_freq_mhz = 80;
    pm_config.min_freq_mhz = 80;
    pm_config.light_sleep_enable = false;
    esp_pm_configure(&pm_config);

    Serial.begin(115200);
    Serial.setDebugOutput(SERIAL_DEBUG);
    esp_log_level_set("*", ESP_LOG_LEVEL);

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
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;       //XCLK 20MHz or 10MHz
    config.pixel_format = PIXFORMAT_JPEG; //YUV422,GRAYSCALE,RGB565,JPEG
    config.frame_size = FRAMESIZE_UXGA;   //UXGA SVGA VGA QVGA Do not use sizes above QVGA when not JPEG
    config.jpeg_quality = IMAGE_COMPRESSION;
    config.fb_count = 1;//2;                  //if more than one, i2s runs in continuous mode. Use only with JPEG

    // Camera init - Initialise la caméra
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
    } else {
        ESP_LOGD(TAG, "Camera correctly initialized ");
        sensor_t * s = esp_camera_sensor_get();
        EEPROM.begin(1);
        byte val = EEPROM.read(EEPROM_ORIENTATION_ADDRESS);
        if (val & ~(EEPROM_ORIENTATION_FLIP_MASK|EEPROM_ORIENTATION_MIRROR_MASK)) { // Invalid content
            val = 0; // No flip nor mirror
            EEPROM.write(EEPROM_ORIENTATION_ADDRESS, val);
            EEPROM.commit();
        }
        s->set_vflip(s, val & EEPROM_ORIENTATION_FLIP_MASK);
        s->set_hmirror(s, val & EEPROM_ORIENTATION_MIRROR_MASK);
    }

    // Wi-Fi connection - Connecte le module au réseau Wi-Fi
    ESP_LOGD(TAG, "Start Wi-Fi connexion ");
    // attempt to connect; should it fail, fall back to AP
    WiFiManager().autoConnect(TAG + ESP_getChipId(), "");
    ESP_LOGD(TAG, "Wi-Fi connected ");
    
    // Start streaming web server
    startCameraServer();

#ifdef ENABLE_RTSPSERVER
    rtspServer.begin();
    ESP_LOGI(TAG, "Camera Stream Ready");
    xTaskCreate([](void *){
                    for(;;){
                        rtspServer.handle();
                        delay(rtspServer.clientCount() > 0?100:1000);
                    }
                },   /* Task function. */
                "RTSP",     /* name of task. */
                8192,       /* Stack size of task */
                NULL,        /* parameter of the task */
                1,           /* priority of the task */
                &TaskRTSP);  /* Task handle to keep track of created task */
#endif
}

void loop() {
    timerWrite(watchdog, 0); //reset timer (feed watchdog)
    server.handleClient();
    if (OTA)  OTA->handle();
    if (ledOnTimer != 0) {
        if (esp_timer_get_time() > ledOnTimer) {
            switchLed(false);
        }
    }
    if (rebootRequested != 0) {
        if (esp_timer_get_time() > rebootRequested) {
            rebootRequested = 0;
            esp_restart();
        }
    }
    if (otaOnTimer != 0) {
        if (esp_timer_get_time() > otaOnTimer) {
            enableOTA(false, true);
        }
    }
    delay(10);
}
