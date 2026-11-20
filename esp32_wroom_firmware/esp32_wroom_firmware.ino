/*
 * ESP32-WROOM-32D  --  Wi-Fi / REST / WebSocket front end for the Zumo
 *
 * Board package: "esp32" by Espressif (2.x or 3.x). Board: "ESP32 Dev Module"
 * Libraries:     "WebSockets" (Markus Sattler), "ArduinoJson" v7
 *                (WiFi, WebServer, ESPmDNS, DNSServer, Preferences ship with the ESP32 core)
 *
 * Wiring (UART2 is used instead of SoftwareSerial):
 *   ESP32 GPIO27 (TX2) -> UNO D2                (3.3 V -> 5 V input, normally reads as HIGH)
 *   UNO D6 -> voltage divider -> ESP32 GPIO25 (RX2)   (5 V -> 3.3 V, e.g. 1k + 2k)
 *   common GND
 *   Do NOT connect the UNO's 5 V TX directly to the ESP32.
 *
 * Wi-Fi setup
 *   - Credentials are stored in NVS (Preferences). On boot the saved network is tried for 15 s.
 *   - If there are none, or the network is not found, the ESP32 opens its own
 *     network "Zumo-Setup" with a captive portal (http://192.168.4.1/wifi).
 *     Pick a network, enter the password, Save: the board restarts and joins it.
 *   - While the portal is open it keeps retrying the saved network every 30 s and
 *     closes the portal as soon as it connects. The robot API also works on
 *     192.168.4.1 while the portal is open.
 *   - Change the network later at http://zumo.local/wifi
 *   - Forget the network: POST /api/wifi/reset, or hold the Zumo button for 5 s.
 *   - Built-in LED (GPIO2): blinking = setup portal, solid on = connected to Wi-Fi.
 *
 * REST (port 80)                         body
 *   POST /api/drive      {"left":150,"right":150}
 *   POST /api/stop
 *   POST /api/led        {"on":true}               LED on D13
 *   POST /api/buzzer     {"freq":880,"ms":200}     freq 0 = silence
 *   POST /api/wifi/reset                           forget Wi-Fi and restart into setup
 *   GET  /api/status                               everything
 *   GET  /api/sensors | /api/battery | /api/button | /api/led | /api/wifi
 *
 * WebSocket (port 81), JSON text frames
 *   client -> {"cmd":"drive","left":150,"right":150}
 *             {"cmd":"stop"} {"cmd":"led","on":true}
 *             {"cmd":"buzzer","freq":880,"ms":200} {"cmd":"status"}
 *   server -> {"ok":true,"cmd":"..."} or {"error":"..."}
 *             {"type":"telemetry", ...}   about every POLL_MS while a client is connected
 *             {"type":"event","name":"button","pressed":true}
 */
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ---------------- config ----------------
const char* AP_NAME = "Zumo-Setup";
const char* AP_PASS = "";                          // empty = open setup network; 8+ chars = WPA2
const unsigned long CONNECT_TIMEOUT_MS = 15000;    // boot-time connect attempt
const unsigned long PORTAL_RETRY_MS = 30000;       // retry saved network while portal is open
const unsigned long RESET_HOLD_MS = 5000;          // hold Zumo button to forget Wi-Fi

const unsigned long LINK_BAUD = 38400;             // must match the UNO
const unsigned long POLL_MS = 100;                 // telemetry poll period
const int NUM_SENSORS = 6;

const int LINK_RX_PIN = 25;                        // UART2 RX  <- UNO D6 (through divider)
const int LINK_TX_PIN = 27;                        // UART2 TX  -> UNO D2
const int LED_PIN = 2;                             // on-board LED on most ESP32 DevKits
const int LED_ON = HIGH;                           // set to LOW if your board's LED is active low
const int LED_OFF = !LED_ON;

HardwareSerial& unoLink = Serial2;
WebServer server(80);
WebSocketsServer ws(81);
DNSServer dns;
Preferences prefs;

struct State {
  int left = 0, right = 0;
  int mv = 0;
  bool button = false;
  bool led = false;
  int sensors[NUM_SENSORS] = {0};
  unsigned long updated = 0;
} st;

unsigned long pausePollUntil = 0;
unsigned long btnPressedAt = 0;
unsigned long restartAt = 0;

// ---------------- Wi-Fi credentials (NVS) ----------------
struct WifiCfg {
  char ssid[33];
  char pass[65];
};

WifiCfg savedCfg;
bool hasSaved = false;
bool portalActive = false;
unsigned long lastPortalRetry = 0;

bool loadCfg(WifiCfg& c) {
  memset(&c, 0, sizeof(c));
  prefs.begin("zumo", true);
  String s = prefs.getString("ssid", "");
  String p = prefs.getString("pass", "");
  prefs.end();
  strncpy(c.ssid, s.c_str(), sizeof(c.ssid) - 1);
  strncpy(c.pass, p.c_str(), sizeof(c.pass) - 1);
  return c.ssid[0] != 0;
}

void saveCfg(const String& ssid, const String& pass) {
  prefs.begin("zumo", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

void clearCfg() {
  prefs.begin("zumo", false);
  prefs.clear();
  prefs.end();
}

// ---------------- commands to the UNO ----------------
void sendDrive(int l, int r) {
  st.left = constrain(l, -400, 400);
  st.right = constrain(r, -400, 400);
  unoLink.printf("M %d %d\n", st.left, st.right);
}
void sendStop() { st.left = st.right = 0; unoLink.print("S\n"); }
void sendLed(bool on) { st.led = on; unoLink.printf("L %d\n", on ? 1 : 0); }
void sendBuzzer(int f, int ms) { unoLink.printf("B %d %d\n", f, ms); }

// Shared by REST and WebSocket.
bool execute(const String& cmd, JsonVariantConst d, String& err) {
  if (cmd == "drive") {
    if (!d["left"].is<int>() || !d["right"].is<int>()) { err = "need integer left,right"; return false; }
    sendDrive(d["left"], d["right"]);
  } else if (cmd == "stop") {
    sendStop();
  } else if (cmd == "led") {
    if (!d["on"].is<bool>()) { err = "need on (bool)"; return false; }
    sendLed(d["on"]);
  } else if (cmd == "buzzer") {
    if (!d["freq"].is<int>()) { err = "need integer freq (0 = off), optional ms"; return false; }
    sendBuzzer(d["freq"], d["ms"] | 200);
  } else {
    err = "unknown cmd";
    return false;
  }
  return true;
}

// ---------------- state -> JSON ----------------
void fillSensors(JsonObject o) {
  JsonArray v = o["values"].to<JsonArray>();   // raw reflectance, left -> right; higher = darker, 0..2500
  for (int i = 0; i < NUM_SENSORS; i++) v.add(st.sensors[i]);
}

void fillState(JsonDocument& doc) {
  doc["left"] = st.left;
  doc["right"] = st.right;
  doc["battery_mv"] = st.mv;
  doc["button"] = st.button;
  doc["led"] = st.led;
  fillSensors(doc["reflectance"].to<JsonObject>());
  doc["rssi"] = WiFi.RSSI();
  doc["uptime_s"] = millis() / 1000;
  doc["age_ms"] = millis() - st.updated;
}

void broadcast(JsonDocument& doc) {
  String out;
  serializeJson(doc, out);
  ws.broadcastTXT(out);
}

// ---------------- UNO -> ESP ----------------
void parseLine(char* line) {
  if (line[0] == 'T') {
    int mv, b, l, s[NUM_SENSORS];
    if (sscanf(line, "T %d %d %d %d %d %d %d %d %d",
               &mv, &b, &l, &s[0], &s[1], &s[2], &s[3], &s[4], &s[5]) == 9) {
      st.mv = mv; st.button = b; st.led = l;
      for (int i = 0; i < NUM_SENSORS; i++) st.sensors[i] = s[i];
      st.updated = millis();
      if (ws.connectedClients() > 0) {
        JsonDocument doc;
        doc["type"] = "telemetry";
        fillState(doc);
        broadcast(doc);
      }
    }
  } else if (line[0] == 'E') {
    int p;
    if (sscanf(line, "E B %d", &p) == 1) {
      st.button = p;
      btnPressedAt = p ? millis() : 0;
      JsonDocument doc;
      doc["type"] = "event";
      doc["name"] = "button";
      doc["pressed"] = (p != 0);
      broadcast(doc);
    }
  }
  // "OK" / "ERR" replies are ignored
}

void readLink() {
  static char buf[96];
  static uint8_t n = 0;
  while (unoLink.available()) {
    char c = unoLink.read();
    if (c == '\n') {
      buf[n] = 0;
      if (n) parseLine(buf);
      n = 0;
    } else if (c != '\r' && n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

// ---------------- Wi-Fi: connect / portal ----------------
bool connectSaved(unsigned long timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(savedCfg.ssid, savedCfg.pass);
  Serial.printf("Connecting to \"%s\"", savedCfg.ssid);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < timeoutMs) {
    delay(250);
    Serial.print('.');
    yield();
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void startPortal() {
  portalActive = true;
  lastPortalRetry = millis();
  WiFi.mode(WIFI_AP_STA);
  IPAddress ip(192, 168, 4, 1);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  if (strlen(AP_PASS) >= 8) WiFi.softAP(AP_NAME, AP_PASS);
  else WiFi.softAP(AP_NAME);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", ip);                  // answer every DNS query with our IP (captive portal)
  Serial.printf("Setup portal: join Wi-Fi \"%s\" and open http://192.168.4.1/wifi\n", AP_NAME);
}

void stopPortal() {
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  portalActive = false;
  MDNS.begin("zumo");
  Serial.print("Connected, portal closed. IP: ");
  Serial.println(WiFi.localIP());
}

void forgetWifiAndRestart() {
  clearCfg();
  sendStop();
  sendBuzzer(880, 300);
  restartAt = millis() + 1500;
}

void serviceWifi() {
  // status LED
  static unsigned long lastBlink = 0;
  if (portalActive) {
    if (millis() - lastBlink > 300) { lastBlink = millis(); digitalWrite(LED_PIN, !digitalRead(LED_PIN)); }
  } else {
    digitalWrite(LED_PIN, WiFi.status() == WL_CONNECTED ? LED_ON : LED_OFF);
  }

  if (portalActive) {
    dns.processNextRequest();
    if (WiFi.status() == WL_CONNECTED) {
      stopPortal();
    } else if (hasSaved && millis() - lastPortalRetry > PORTAL_RETRY_MS) {
      lastPortalRetry = millis();
      WiFi.begin(savedCfg.ssid, savedCfg.pass);
    }
  }
  // (no MDNS.update() needed on ESP32)

  // hold the Zumo button to forget the Wi-Fi network
  if (btnPressedAt && st.button && millis() - btnPressedAt > RESET_HOLD_MS) {
    btnPressedAt = 0;
    forgetWifiAndRestart();
  }
  if (restartAt && millis() > restartAt) ESP.restart();
}

// ---------------- Wi-Fi setup pages ----------------
const char PAGE_HEAD[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Zumo Wi-Fi</title>
<style>body{font-family:sans-serif;max-width:420px;margin:2em auto;padding:0 1em}
input,button{width:100%;padding:.7em;margin:.4em 0;font-size:1em;box-sizing:border-box}
button{background:#08a;color:#fff;border:0;border-radius:4px}small{color:#666}</style></head><body>
<h2>Zumo Wi-Fi setup</h2>)HTML";
const char PAGE_FOOT[] PROGMEM = "</body></html>";

String esc(const String& s) {
  String o;
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += F("&amp;");
    else if (c == '<') o += F("&lt;");
    else if (c == '>') o += F("&gt;");
    else if (c == '"') o += F("&quot;");
    else o += c;
  }
  return o;
}

int scanNetworks() {
  static int lastN = 0;
  static unsigned long lastAt = 0;
  if (lastAt == 0 || millis() - lastAt > 15000) {
    int n = WiFi.scanNetworks();
    lastN = n < 0 ? 0 : n;
    lastAt = millis();
  }
  return lastN;
}

void handleWifiPage() {
  int n = scanNetworks();
  String h = FPSTR(PAGE_HEAD);
  h += F("<p><small>");
  if (WiFi.status() == WL_CONNECTED) h += "Connected to <b>" + esc(WiFi.SSID()) + "</b> (" + WiFi.localIP().toString() + ")";
  else h += F("Not connected to a Wi-Fi network.");
  h += F("</small></p><form method=\"POST\" action=\"/wifi/save\">"
         "<input name=\"ssid\" list=\"nets\" placeholder=\"Network name\" required maxlength=\"32\" autocapitalize=\"none\">"
         "<datalist id=\"nets\">");
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.length()) h += "<option value=\"" + esc(s) + "\">";
  }
  h += F("</datalist><input name=\"pass\" type=\"password\" placeholder=\"Password (empty for open network)\" maxlength=\"64\">"
         "<button type=\"submit\">Save and connect</button></form>"
         "<p><small>Pick a network from the list or type a hidden one. The robot restarts after saving; "
         "if it cannot join, the Zumo-Setup network reappears.</small></p>");
  h += FPSTR(PAGE_FOOT);
  server.send(200, "text/html", h);
}

void handleWifiSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  String h = FPSTR(PAGE_HEAD);
  if (ssid.length() == 0 || ssid.length() > 32 || pass.length() > 64 || (pass.length() > 0 && pass.length() < 8)) {
    h += F("<p>Invalid input: network name 1-32 characters, password empty or 8-64 characters.</p>"
           "<p><a href=\"/wifi\">Back</a></p>");
    h += FPSTR(PAGE_FOOT);
    server.send(400, "text/html", h);
    return;
  }
  saveCfg(ssid, pass);
  h += "<p>Saved <b>" + esc(ssid) + "</b>. The robot is restarting and will join it.</p>"
       "<p>Reconnect your phone or PC to your normal Wi-Fi, then open <b>http://zumo.local</b> "
       "(or check your router for the IP). If the <b>Zumo-Setup</b> network shows up again, "
       "the name or password was wrong.</p>";
  h += FPSTR(PAGE_FOOT);
  server.send(200, "text/html", h);
  restartAt = millis() + 2000;
}

// ---------------- REST ----------------
void cors() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

void sendJson(JsonDocument& doc, int code = 200) {
  cors();
  String out;
  serializeJson(doc, out);
  server.send(code, "application/json", out);
}

void restCommand(const char* cmd) {
  cors();
  JsonDocument doc;
  String body = server.arg("plain");
  if (body.length() && deserializeJson(doc, body)) {
    server.send(400, "application/json", "{\"error\":\"bad json\"}");
    return;
  }
  String err;
  if (!execute(cmd, doc.as<JsonVariantConst>(), err)) {
    server.send(400, "application/json", "{\"error\":\"" + err + "\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void setupRoutes() {
  server.on("/", HTTP_GET, [] {
    if (portalActive) {
      server.sendHeader("Location", "/wifi");
      server.send(302, "text/plain", "");
    } else {
      server.send(200, "text/html",
                  "<p>Zumo robot API. <a href=\"/api/status\">/api/status</a> &middot; "
                  "<a href=\"/wifi\">Wi-Fi settings</a></p>");
    }
  });
  server.on("/wifi", HTTP_GET, handleWifiPage);
  server.on("/wifi/save", HTTP_POST, handleWifiSave);

  server.on("/api/drive", HTTP_POST, [] { restCommand("drive"); });
  server.on("/api/stop", HTTP_POST, [] { restCommand("stop"); });
  server.on("/api/led", HTTP_POST, [] { restCommand("led"); });
  server.on("/api/buzzer", HTTP_POST, [] { restCommand("buzzer"); });
  server.on("/api/wifi/reset", HTTP_POST, [] {
    cors();
    server.send(200, "application/json", "{\"ok\":true,\"note\":\"Wi-Fi forgotten, restarting into setup\"}");
    forgetWifiAndRestart();
  });

  server.on("/api/status", HTTP_GET, [] {
    JsonDocument d; fillState(d); sendJson(d);
  });
  server.on("/api/sensors", HTTP_GET, [] {
    JsonDocument d; fillSensors(d.to<JsonObject>()); sendJson(d);
  });
  server.on("/api/battery", HTTP_GET, [] {
    JsonDocument d; d["mv"] = st.mv; d["volts"] = st.mv / 1000.0; sendJson(d);
  });
  server.on("/api/button", HTTP_GET, [] {
    JsonDocument d; d["pressed"] = st.button; sendJson(d);
  });
  server.on("/api/led", HTTP_GET, [] {
    JsonDocument d; d["on"] = st.led; sendJson(d);
  });
  server.on("/api/wifi", HTTP_GET, [] {
    JsonDocument d;
    d["connected"] = (WiFi.status() == WL_CONNECTED);
    d["ssid"] = WiFi.SSID();
    d["ip"] = WiFi.localIP().toString();
    d["portal"] = portalActive;
    if (portalActive) { d["ap_ssid"] = AP_NAME; d["ap_ip"] = WiFi.softAPIP().toString(); }
    sendJson(d);
  });

  server.onNotFound([] {
    cors();
    if (server.method() == HTTP_OPTIONS) {   // CORS preflight for any path
      server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
      server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
      server.send(204);
    } else if (portalActive && !server.uri().startsWith("/api")) {
      // captive portal: phones probe /generate_204, /hotspot-detect.html, ...
      server.sendHeader("Location", "http://192.168.4.1/wifi");
      server.send(302, "text/plain", "");
    } else {
      server.send(404, "application/json", "{\"error\":\"not found\"}");
    }
  });
}

// ---------------- WebSocket ----------------
void onWs(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_DISCONNECTED) { sendStop(); return; }   // never leave the robot driving

  if (type == WStype_CONNECTED) {
    JsonDocument doc;
    doc["type"] = "telemetry";
    fillState(doc);
    String out;
    serializeJson(doc, out);
    ws.sendTXT(num, out);
    return;
  }
  if (type != WStype_TEXT) return;

  JsonDocument doc;
  if (deserializeJson(doc, (const char*)payload, length)) {
    ws.sendTXT(num, "{\"error\":\"bad json\"}");
    return;
  }
  String cmd = doc["cmd"] | "";
  if (cmd == "status") {
    JsonDocument s;
    s["type"] = "telemetry";
    fillState(s);
    String out;
    serializeJson(s, out);
    ws.sendTXT(num, out);
    return;
  }
  String err;
  String reply;
  if (execute(cmd, doc.as<JsonVariantConst>(), err)) reply = "{\"ok\":true,\"cmd\":\"" + cmd + "\"}";
  else reply = "{\"error\":\"" + err + "\"}";
  ws.sendTXT(num, reply);
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  unoLink.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  WiFi.persistent(false);                // we manage credentials ourselves
  WiFi.setHostname("zumo");              // must be set before WiFi.mode()
  WiFi.mode(WIFI_STA);                   // starts the TCP/IP stack: must happen BEFORE server.begin()/ws.begin()
  WiFi.setSleep(false);                  // lower latency for drive commands
  hasSaved = loadCfg(savedCfg);

  setupRoutes();

  if (hasSaved && connectSaved(CONNECT_TIMEOUT_MS)) {
    Serial.print("Connected, IP: ");
    Serial.println(WiFi.localIP());
    MDNS.begin("zumo");                  // http://zumo.local
  } else {
    startPortal();
  }

  server.begin();
  ws.begin();
  ws.onEvent(onWs);
}

void loop() {
  serviceWifi();
  server.handleClient();
  ws.loop();
  readLink();

  static unsigned long lastPoll = 0;
  unsigned long now = millis();
  if (now >= pausePollUntil && now - lastPoll >= POLL_MS) {
    lastPoll = now;
    unoLink.print("T\n");
  }
}