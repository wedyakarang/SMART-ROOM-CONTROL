#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <ESP32Servo.h>
#include <time.h>
 
// ================= PIN CONFIGURATION =================
#define DHT_PIN     13
#define LDR_PIN     34
#define RELAY_PIN   27
#define SERVO_PIN   18
#define PIR_PIN     14
#define BUZZER_PIN  25
 
// ================= OBJECTS =================
DHTesp dht;
Servo windowServo;
 
// ================= THRESHOLDS =================
float TEMP_LIMIT  = 30.0;  // suhu di atas ini -> relay/kipas ON
int   LIGHT_LIMIT = 2000;  // di atas ini = terang -> jendela CLOSE
                           // di bawah ini = gelap -> jendela OPEN
 
const int WINDOW_OPEN  = 0;
const int WINDOW_CLOSE = 90;
 
const unsigned long MANUAL_HOLD_MS  = 30000;  // perintah MQTT menahan mode manual 30 detik
const unsigned long SENSOR_INTERVAL = 2000;
 
// ================= WIFI & EMQX MQTT =================
const char* WIFI_SSID     = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";
 
const char* MQTT_BROKER    = "c1460dff.ala.asia-southeast1.emqxsl.com";
const int   MQTT_PORT      = 8883;
const char* MQTT_USER      = "mobcom";
const char* MQTT_PASS      = "mobcom";
const char* MQTT_CLIENT_ID = "ESP32_SmartRoom_Client";
 
// Topik MQTT
const char* TOPIC_TELEMETRY  = "smartroom/telemetry";
const char* TOPIC_STATUS     = "smartroom/status";
const char* TOPIC_MOTION     = "smartroom/motion";
const char* TOPIC_RELAY_CMD  = "smartroom/relay/set";
const char* TOPIC_WINDOW_CMD = "smartroom/window/set";
const char* TOPIC_BUZZER_CMD = "smartroom/buzzer/set";
 
// Sertifikat CA EMQX (DigiCert Global Root G2) - dari file emqxsl-ca.crt
const char* EMQX_CA_CERT = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
"MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n" \
"2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n" \
"1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n" \
"q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n" \
"tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n" \
"vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n" \
"BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n" \
"5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n" \
"1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n" \
"NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n" \
"Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n" \
"8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n" \
"pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n" \
"MrY=\n" \
"-----END CERTIFICATE-----\n";
 
WiFiClientSecure tlsClient;
PubSubClient mqttClient(tlsClient);
 
// ================= STATE =================
bool relayState    = false;
bool windowClosed  = false;
bool buzzerState   = false;
bool motionActive  = false;
bool motionPinPrev = false;
 
unsigned long lastSensorRead = 0;
unsigned long lastMqttRetry  = 0;
unsigned long lastWifiRetry  = 0;
bool mqttWasConnected = false;
 
// Waktu berakhirnya mode manual (0 = mode otomatis)
unsigned long relayManualUntil  = 0;
unsigned long windowManualUntil = 0;
unsigned long buzzerManualUntil = 0;
 
bool isManual(unsigned long until) {
  return until != 0 && (long)(until - millis()) > 0;
}
 
// ================= AKTUATOR =================
void setRelay(bool on) {
  relayState = on;
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}
 
void setBuzzer(bool on) {
  buzzerState = on;
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  if (on) {
    ledcAttachChannel(BUZZER_PIN, 2000, 8, 4);
    ledcWriteTone(BUZZER_PIN, 2000);
  } else {
    ledcWrite(BUZZER_PIN, 0);
    ledcDetach(BUZZER_PIN);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  }
#else
  if (on) {
    ledcSetup(4, 2000, 8);
    ledcAttachPin(BUZZER_PIN, 4);
    ledcWriteTone(4, 2000);
  } else {
    ledcWrite(4, 0);
    ledcDetachPin(BUZZER_PIN);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  }
#endif
}
 
void setWindow(bool close) {
  if (close == windowClosed) return;
  windowClosed = close;
  windowServo.write(close ? WINDOW_CLOSE : WINDOW_OPEN);
}
 
// ================= WIFI =================
void connectWiFi() {
  Serial.print("[WiFi] Menghubungkan ke ");
  Serial.print(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {  // maksimal ~30 detik
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" Terhubung (" + WiFi.localIP().toString() + ")");
  } else {
    Serial.println(" Gagal!");
  }
}
 
// ================= NTP (wajib untuk verifikasi sertifikat TLS) =================
void syncTime() {
  if (WiFi.status() != WL_CONNECTED) return;
 
  configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");  // WIB (UTC+7)
  Serial.print("[NTP] Sinkronisasi waktu");
 
  time_t now = time(nullptr);
  int attempts = 0;
  while (now < 1700000000 && attempts < 30) {  // maksimal ~15 detik
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    attempts++;
  }
 
  if (now >= 1700000000) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    Serial.printf(" OK (%04d-%02d-%02d %02d:%02d:%02d)\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    Serial.println(" Gagal! (koneksi TLS mungkin ditolak)");
  }
}
 
// ================= MQTT CALLBACK =================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  msg.toUpperCase();
  Serial.printf("[MQTT] %s -> %s\n", topic, msg.c_str());
 
  String t = String(topic);
 
  if (t == TOPIC_RELAY_CMD) {
    if (msg == "ON" || msg == "1") {
      setRelay(true);
      relayManualUntil = millis() + MANUAL_HOLD_MS;
    } else if (msg == "OFF" || msg == "0") {
      setRelay(false);
      relayManualUntil = millis() + MANUAL_HOLD_MS;
    } else if (msg == "AUTO") {
      relayManualUntil = 0;
    }
  } else if (t == TOPIC_WINDOW_CMD) {
    if (msg == "CLOSE" || msg == "CLOSED") {
      setWindow(true);
      windowManualUntil = millis() + MANUAL_HOLD_MS;
    } else if (msg == "OPEN") {
      setWindow(false);
      windowManualUntil = millis() + MANUAL_HOLD_MS;
    } else if (msg == "AUTO") {
      windowManualUntil = 0;
    }
  } else if (t == TOPIC_BUZZER_CMD) {
    if (msg == "ON" || msg == "1") {
      setBuzzer(true);
      buzzerManualUntil = millis() + MANUAL_HOLD_MS;
    } else if (msg == "OFF" || msg == "0") {
      setBuzzer(false);
      buzzerManualUntil = millis() + MANUAL_HOLD_MS;
    } else if (msg == "AUTO") {
      buzzerManualUntil = 0;
    }
  }
}
 
// ================= MQTT RECONNECT =================
void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastMqttRetry < 5000) return;
  lastMqttRetry = millis();
 
  // Pastikan waktu valid sebelum mencoba TLS
  if (time(nullptr) < 1700000000) {
    syncTime();
  }
 
  Serial.print("[MQTT] Menghubungkan ke broker...");
  String clientId = String(MQTT_CLIENT_ID) + "_" + String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    Serial.println(" Terhubung!");
    mqttClient.subscribe(TOPIC_RELAY_CMD);
    mqttClient.subscribe(TOPIC_WINDOW_CMD);
    mqttClient.subscribe(TOPIC_BUZZER_CMD);
    mqttClient.publish(TOPIC_STATUS, "ONLINE", true);
  } else {
    char errBuf[128] = "";
    int tlsErr = tlsClient.lastError(errBuf, sizeof(errBuf));
    Serial.printf(" Gagal (rc=%d, tls=%d: %s)\n", mqttClient.state(), tlsErr, errBuf);
  }
}
 
// ================= SETUP =================
void setup() {
  Serial.begin(115200);
 
  dht.setup(DHT_PIN, DHTesp::DHT22);
 
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false);
 
  pinMode(BUZZER_PIN, OUTPUT);
  setBuzzer(false);
 
  pinMode(PIR_PIN, INPUT);
 
  windowServo.attach(SERVO_PIN);
  windowServo.write(WINDOW_OPEN);
  windowClosed = false;
 
  Serial.println("SMART ROOM CONTROL");
  connectWiFi();
  syncTime();
 
  tlsClient.setCACert(EMQX_CA_CERT);
  tlsClient.setTimeout(15);
  mqttClient.setBufferSize(512);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(15);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}
 
// ================= LOOP =================
void loop() {
  // Sambungkan ulang WiFi bila terputus
  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiRetry > 10000) {
    lastWifiRetry = millis();
    Serial.println("[WiFi] Terputus, mencoba menyambung ulang...");
    WiFi.reconnect();
  }
 
  if (!mqttClient.connected()) {
    if (mqttWasConnected) {
      Serial.printf("[MQTT] Koneksi terputus (state=%d)\n", mqttClient.state());
      mqttWasConnected = false;
    }
    reconnectMQTT();
    if (mqttClient.connected()) mqttWasConnected = true;
  }
  mqttClient.loop();
 
  // ---------- PIR ----------
  bool pirNow = digitalRead(PIR_PIN) == HIGH;
  motionActive = pirNow;
 
  if (pirNow && !motionPinPrev) {
    Serial.println("[PIR] Gerakan terdeteksi!");
    if (mqttClient.connected()) mqttClient.publish(TOPIC_MOTION, "DETECTED");
  } else if (!pirNow && motionPinPrev) {
    Serial.println("[PIR] Gerakan berhenti");
    if (mqttClient.connected()) mqttClient.publish(TOPIC_MOTION, "CLEAR");
  }
  motionPinPrev = pirNow;
 
  if (!isManual(buzzerManualUntil)) {
    if (buzzerState != motionActive) setBuzzer(motionActive);
  }
 
  // ---------- Sensor lain (tiap 2 detik) ----------
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();
 
    TempAndHumidity data = dht.getTempAndHumidity();
    float temperature = data.temperature;
    float humidity    = data.humidity;
    int lightValue    = analogRead(LDR_PIN);
 
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("[DHT22] Gagal membaca sensor!");
      return;
    }
 
    if (!isManual(relayManualUntil)) {
      setRelay(temperature > TEMP_LIMIT);
    }
 
    if (!isManual(windowManualUntil)) {
      setWindow(lightValue > LIGHT_LIMIT);
    }
 
    bool mqttOk = false;
    if (mqttClient.connected()) {
      char payload[220];
      snprintf(payload, sizeof(payload),
               "{\"temp\":%.1f,\"hum\":%.1f,\"light\":%d,\"motion\":\"%s\","
               "\"relay\":\"%s\",\"window\":\"%s\",\"buzzer\":\"%s\"}",
               temperature, humidity, lightValue,
               motionActive ? "YES" : "NO",
               relayState ? "ON" : "OFF",
               windowClosed ? "CLOSED" : "OPEN",
               buzzerState ? "ON" : "OFF");
      mqttOk = mqttClient.publish(TOPIC_TELEMETRY, payload);
    }
 
    unsigned long sec = millis() / 1000;
    Serial.printf("[%02lu:%02lu] Suhu: %.1f C | Lembap: %.1f%% | Cahaya: %d | Gerak: %s\n",
                  sec / 60, sec % 60, temperature, humidity, lightValue,
                  motionActive ? "YA" : "TIDAK");
    Serial.printf("        Relay: %s | Jendela: %s | Buzzer: %s | MQTT: %s\n",
                  relayState ? "ON" : "OFF",
                  windowClosed ? "TUTUP" : "BUKA",
                  buzzerState ? "ON" : "OFF",
                  mqttOk ? "OK" : "ERR");
  }
}
 