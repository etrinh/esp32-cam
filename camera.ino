/*********
  projetsdiy.fr - diyprojects.io
  Complete project details at https://RandomNerdTutorials.com

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
#include "esp_http_server.h"    // API https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/protocols/esp_http_server.html

#define SERIAL_DEBUG false               // Enable / Disable log - activer / désactiver le journal
#define ESP_LOG_LEVEL ESP_LOG_VERBOSE    // ESP_LOG_NONE, ESP_LOG_VERBOSE, ESP_LOG_DEBUG, ESP_LOG_ERROR, ESP_LOG_WARM, ESP_LOG_INFO

#define VERSION   "1.01"

// Web server port - port du serveur web
#define WEB_SERVER_PORT 80
#define URI_STATIC_JPEG "/jpg/image.jpg"
#define URI_STREAM "/stream"
#define URI_FLASH "/flash"
#define URI_WIFI "/reset"
#define URI_REBOOT "/reboot"
#define URI_OTA "/ota"
#define URI_UPDATE "/update"
#define URI_STATUS "/status"
#define URI_ROOT "/"
#define URI_USAGE "/help"
#define URI_INFO "/info"

#define OTA_TIMER (10*60)
#define FLASH_TIMER (10)
#define REBOOT_TIMER (3)
#define OTA_REBOOT_TIMER (1)

// Basic image Settings (compression, flip vertical orientation) - Réglages basiques de l'image (compression, inverse l'orientation verticale)
#define FLIP_V false            // Vertical flip - inverse l'image verticalement
#define MIRROR_H false          // Horizontal mirror - miroir horizontal
#define IMAGE_COMPRESSION 10   //0-63 lower number means higher quality - Plus de chiffre est petit, meilleure est la qualité de l'image, plus gros est le fichier

#define TAG "esp32-cam"

/*
   Handler for video streaming - Entête pour le flux vidéo
*/
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static const char* _UPDATE_PART = "Content-Type: text/html\r\nContent-Length: %u\r\n\r\n";

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

httpd_handle_t stream_httpd = NULL;

#define FLASH_PIN 4
int64_t ledOnTimer = 0;
int64_t rebootRequested = 0;
int64_t otaOnTimer = 0;

static void switchLed(bool enable)
{
  if (enable) {
    int64_t fr_start = esp_timer_get_time();
    ledOnTimer = fr_start + FLASH_TIMER * 1000 * 1000;
    digitalWrite(FLASH_PIN, 1);
  }
  else {
    ledOnTimer = 0;
    digitalWrite(FLASH_PIN, 0);
  }
}

static esp_err_t led_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;

  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    char* buf = (char*)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      char param[32];
      /* Get value of expected key from query string */
      if (httpd_query_key_value(buf, "action", param, sizeof(param)) == ESP_OK) {
        if (strncmp(param, "on", 32) == 0) {
          switchLed(true);
        }
        else if (strncmp(param, "off", 32) == 0) {
          switchLed(false);
        }
        else if (strncmp(param, "toggle", 32) == 0) {
          switchLed(ledOnTimer == 0);
        }
      }
    }
    free(buf);
  }  
  httpd_resp_send(req, NULL, 0);  // Response body can be empty
  return res;
}

static esp_err_t wifi_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  system_restore();
  esp_restart();
  httpd_resp_send(req, NULL, 0);  // Response body can be empty
  return res;
}

static esp_err_t reboot_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  rebootRequested = esp_timer_get_time() + REBOOT_TIMER * 1000 * 1000;
  httpd_resp_send(req, NULL, 0);  // Response body can be empty
  return res;
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

static esp_err_t ota_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;

  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    char* buf = (char*)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      char param[32];
      /* Get value of expected key from query string */
      if (httpd_query_key_value(buf, "action", param, sizeof(param)) == ESP_OK) {
        if (strncmp(param, "on", 32) == 0) {
          enableOTA(true);
        }
        else if (strncmp(param, "off", 32) == 0) {
          enableOTA(false);
        }
        else if (strncmp(param, "toggle", 32) == 0) {
          enableOTA(OTA==NULL);
        }
      }
    }
    free(buf);
  }  
  res = httpd_resp_send(req, NULL, 0);
  return res;
}
#define MIN(a,b)  ((a)<(b)?(a):(b))
#define MAX(a,b)  ((a)>(b)?(a):(b))
//#define DEBUG_OTA
//#define PROGRESSION
static esp_err_t update_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  char buf[512];
  int ret, remaining = req->content_len;
  String content;
  char part_buf[64];

  int footerSize = -1;
#ifndef DEBUG_OTA
  bool canBegin = Update.begin(remaining);
#else
  bool canBegin = true;
#endif
  // If yes, begin
  if (canBegin) {
#ifdef PROGRESSION
    std::string progression;
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
#endif
    while (remaining > 0) {
      // Read the data for the request
      if ((ret = httpd_req_recv(req, buf,
                    MIN(remaining, sizeof(buf) - 1))) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
          // Retry receiving if timeout occurred
          continue;
        }
        return ESP_FAIL;
      }
      remaining -= ret;

#ifdef PROGRESSION
      progression += ".\r\n";
      size_t hlen = snprintf(part_buf, 64, _UPDATE_PART, progression.length());
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      res = httpd_resp_send_chunk(req, (const char *)progression.c_str(), progression.length());
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
#endif
      char* update_ptr = buf;
      if (footerSize == -1) {
        buf[ret] = 0;
        char * endBoundary = strstr(buf, "\r\n");
        if (endBoundary) {
          char * startContent = strstr(endBoundary, "\r\n\r\n");
          if (startContent) {
            update_ptr = startContent + 4;
            ret -= update_ptr - buf;
            footerSize = endBoundary - buf + 2 + 4/* BOUNDARY + "--\r\n\r\n" */;
          }
          else {
            return ESP_FAIL;
          }
        }
        else {
          return ESP_FAIL;
        }
      }
#ifndef DEBUG_OTA
      Update.write((uint8_t*)update_ptr, ret - MAX(0, footerSize - remaining));
#elif !defined(PROGRESSION)
      httpd_resp_send_chunk(req, update_ptr, ret - MAX(0, footerSize - remaining));
#endif
    }

#ifndef DEBUG_OTA
    if (Update.end(true)) {
      if (Update.isFinished()) {
        content = "Update successfully completed. Rebooting.";
        rebootRequested = esp_timer_get_time() + OTA_REBOOT_TIMER * 1000 * 1000;
      } else {
        content = "Update not finished? Something went wrong!";
      }
    } else {
      content = "Error Occurred. Error #: " + String(Update.getError()) + ": " + String(Update.errorString());
    }
#endif
  } else {
    // not enough space to begin OTA
    // Understand the partitions and
    // space availability
    content = "Not enough space to begin OTA";
  }
#ifndef DEBUG_OTA
 #ifdef PROGRESSION
  size_t hlen = snprintf(part_buf, 64, _UPDATE_PART, content.length());
  res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
  res = httpd_resp_send_chunk(req, (const char *)content.c_str(), content.length());
  res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
 #endif
  res = httpd_resp_send(req, content.c_str(), content.length());
#else
  httpd_resp_send_chunk(req, NULL, 0);
#endif
  return res;
}
static esp_err_t usage_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  std::string info =
"<html>"
  "<head>"
    "<title>esp32-cam - API</title>"
    "<style>"
    "</style>"
  "</head>"
  "<body>"
    "<h1>" TAG "<span id=\"version\"> (v" VERSION ") - API Usage</span></h1>"
    "<ul>"
      "<li>Led on/off/toggle: " URI_FLASH "?action=[on|off|toggle]</li>"
      "<li>Snapshot JPEG: " URI_STATIC_JPEG "</li>"
      "<li>Stream MJPEG: " URI_STREAM "</li>"
      "<li>Reboot: " URI_REBOOT "</li>"
      "<li>Reset Wifi: " URI_WIFI "</li>"
      "<li>OTA on/off/toggle: " URI_OTA "?action=[on|off|toggle]</li>"
      "<li>Status (JSON): " URI_STATUS "</li>"
    "</ul>"
  "</body>"
"</html>";  
  res = httpd_resp_send(req, info.c_str(), info.length());
  return res;
}

String ip2Str(IPAddress ip){
  String s;
  for (int i=0; i<4; i++) {
    s += i  ? "." + String(ip[i]) : String(ip[i]);
  }
  return s;
}

static esp_err_t status_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  String info = "{"
    "\"version\":\"" VERSION "\","
    "\"led\":\"" + String(ledOnTimer?"true":"false") + "\","
    "\"ledTimer\":\"" + String(MAX(0, int((ledOnTimer - esp_timer_get_time()) / 1000000))) + "\","
    "\"ssid\":\"" + WiFi.SSID() + "\","
    "\"rssi\":\"" + String(WiFi.RSSI()) + "\","
    "\"ip\":\"" + ip2Str(WiFi.localIP()) + "\","
    "\"mac\":\"" + WiFi.macAddress() + "\","
    "\"chipId\":\"" + ESP_getChipId() + "\","
    "\"ota\":\"" + String(OTA?"true":"false") + "\","
    "\"otaTimer\":\"" + String(MAX(0, int((otaOnTimer - esp_timer_get_time()) / 1000000))) + "\","
    "\"reboot\":\"" + String(rebootRequested>0?"true":"false") + "\","
    "\"rebootTimer\":\"" + String(MAX(0, int((rebootRequested - esp_timer_get_time()) / 1000000))) + "\""
  "}";
  res = httpd_resp_set_type(req, "text/json");
  res = httpd_resp_send(req, info.c_str(), info.length());
  return res;
}

static esp_err_t info_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  std::string info =
"<html>"
  "<head>"
    "<title>esp32-cam</title>"
    "<script type=\"text/javascript\">"
      "function invoke(url)"
      "{"
        "var xhr = new XMLHttpRequest();"
        "xhr.open(\"GET\", url, true);"
        "xhr.send(null);"
      "};"          
      "setInterval(function() { update(); }, 1000);"
      "setInterval(function() { document.getElementById(\"snapshot\").src = \"" URI_STATIC_JPEG "?random=\"+new Date().getTime(); }, 3000);"
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
      "update();"
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
            "<a class=\"link\" href=\"" URI_STREAM "\">Stream</a>"
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
"</html>";
  res = httpd_resp_send(req, info.c_str(), info.length());
  return res;
}

bool STREAMING = false;
/*
   This method only stream one JPEG image - Cette méthode ne publie qu'une seule image JPEG
   Compatible with/avec Jeedom / NextDom / Domoticz
*/
static esp_err_t capture_handler(httpd_req_t *req) {
  if (STREAMING) {
      httpd_resp_send_500(req);        
      return ESP_OK;
  }
  STREAMING = true;

  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t fb_len = 0;
  res = httpd_resp_set_type(req, "image/jpeg");
  if (res == ESP_OK) {
    res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=image.jpg");  //capture
  }
  if (res == ESP_OK) {
    ESP_LOGI(TAG, "Take a picture");
    fb = esp_camera_fb_get();
    if (!fb)
    {
      ESP_LOGE(TAG, "Camera capture failed");
      httpd_resp_send_500(req);
      res = ESP_FAIL;
    } else {
      fb_len = fb->len;
      res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
      esp_camera_fb_return(fb);
    }
  }
  STREAMING = false;
  return res;
}

/*
   This method stream continuously a video
   Compatible with/avec Home Assistant, HASS.IO
*/
esp_err_t stream_handler(httpd_req_t *req) {
  if (STREAMING) {
      httpd_resp_send_500(req);        
      return ESP_OK;
  }
  STREAMING = true;

  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len;
  uint8_t * _jpg_buf;
  char part_buf[64];
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    STREAMING = false;
    return res;
  }
  ESP_LOGI(TAG, "Start video streaming");
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGE(TAG, "Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        if (!jpeg_converted) {
          ESP_LOGE(TAG, "JPEG compression failed");
          esp_camera_fb_return(fb);
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb->format != PIXFORMAT_JPEG) {
      free(_jpg_buf);
    }
    esp_camera_fb_return(fb);
    if (res != ESP_OK) {
      break;
    }
  }
  STREAMING = false;
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = WEB_SERVER_PORT;
  // endpoints
  static const httpd_uri_t handlers[] = {
    {
      .uri       = URI_STATIC_JPEG,
      .method    = HTTP_GET,
      .handler   = capture_handler,
      .user_ctx  = &config
    },
    {
      .uri       = URI_STREAM,
      .method    = HTTP_GET,
      .handler   = stream_handler,
      .user_ctx  = &config
    },
    {
      .uri       = URI_FLASH,
      .method    = HTTP_GET,
      .handler   = led_handler,
      .user_ctx  = NULL
    },
    {
      .uri       = URI_WIFI,
      .method    = HTTP_GET,
      .handler   = wifi_handler,
      .user_ctx  = NULL
    },
    {
      .uri       = URI_REBOOT,
      .method    = HTTP_GET,
      .handler   = reboot_handler,
      .user_ctx  = NULL
    },
    {
      .uri       = URI_OTA,
      .method    = HTTP_GET,
      .handler   = ota_handler,
      .user_ctx  = NULL
    },
    {
      .uri       = URI_UPDATE,
      .method    = HTTP_POST,
      .handler   = update_handler,
      .user_ctx  = NULL
    },
    {
      .uri       = URI_STATUS,
      .method    = HTTP_GET,
      .handler   = status_handler,
      .user_ctx  = NULL
    },
    {
      .uri       = URI_INFO,
      .method    = HTTP_GET,
      .handler   = info_handler,
      .user_ctx  = NULL
    },
    {
      .uri       = URI_USAGE,
      .method    = HTTP_GET,
      .handler   = usage_handler,
      .user_ctx  = NULL
    },
    {
      .uri       = URI_ROOT,
      .method    = HTTP_GET,
      .handler   = info_handler,
      .user_ctx  = NULL
    }
  };
  config.max_uri_handlers = sizeof(handlers)/sizeof(handlers[0]);

  ESP_LOGI(TAG, "Register URIs and start web server");
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    for (int i = 0; i < config.max_uri_handlers; ++i) {
      if ( httpd_register_uri_handler(stream_httpd, &handlers[i]) != ESP_OK) {
        ESP_LOGE(TAG, "register uri failed for " + String(handlers[i].uri));
        return;
      }
    }
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  // Stop Bluetooth
  btStop();

  // set frequency to 80Mhz
  esp_pm_config_esp32_t pm_config;
  pm_config.max_freq_mhz = RTC_CPU_FREQ_80M;
  pm_config.max_freq_mhz = 80;
  pm_config.min_freq_mhz = RTC_CPU_FREQ_80M;
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
  config.jpeg_quality = 10;
  config.fb_count = 2;                  //if more than one, i2s runs in continuous mode. Use only with JPEG

  // Camera init - Initialise la caméra
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
//    return;
  } else {
    ESP_LOGD(TAG, "Camera correctly initialized ");
    sensor_t * s = esp_camera_sensor_get();
    s->set_vflip(s, FLIP_V);
    s->set_hmirror(s, MIRROR_H);
  }

  pinMode (FLASH_PIN, OUTPUT);//back led

  // Wi-Fi connection - Connecte le module au réseau Wi-Fi
  ESP_LOGD(TAG, "Start Wi-Fi connexion ");
  // attempt to connect; should it fail, fall back to AP
  WiFiManager wm;
  WiFi.mode(WIFI_STA);
  if (wm.connectWifi("", "") != WL_CONNECTED)   {
    wm.startConfigPortal(TAG + ESP_getChipId(), "");
    esp_restart();
  }
  ESP_LOGD(TAG, "Wi-Fi connected ");
  
  // Start streaming web server
  startCameraServer();

  ESP_LOGI(TAG, "Camera Stream Ready");
}

void loop() {
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

  delay(1000);
}
