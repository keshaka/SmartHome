// ============================================================
//  ESP8266 Fan Controller  –  v1.0.2
//
//  SECTIONS (in order):
//    1.  Libraries
//    2.  EEPROM / Configuration
//    3.  Pin definitions & firmware version
//    4.  Global objects
//    5.  State & SSE
//    6.  SSE push helper
//    7.  RELAY CONTROL
//    8.  MQTT helpers  (Discovery, sensors)
//    9.  MQTT callback
//   10.  MQTT reconnect  (non-blocking)
//   11.  AP SETUP HTML   (PROGMEM)
//   12.  DASHBOARD HTML  (PROGMEM)
//   13.  SETTINGS HTML   (PROGMEM)
//   14.  HTTP handlers  (REST API + settings + AP save)
//   15.  setup()
//   16.  loop()
// ============================================================


// ════════════════════════════════════════════════════════════
//  1.  LIBRARIES
// ════════════════════════════════════════════════════════════
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>


// ════════════════════════════════════════════════════════════
//  2.  EEPROM / CONFIGURATION
// ════════════════════════════════════════════════════════════

// ── Magic byte – written when a valid config is saved ────────
//    If EEPROM[0] != CONFIG_MAGIC → no config → AP setup mode
#define EEPROM_SIZE   512
#define CONFIG_MAGIC  0xA5

// ── Config struct – everything stored in EEPROM ──────────────
struct Config {
  uint8_t  magic;            // must be CONFIG_MAGIC for valid config

  char wifiSSID[33];         // WiFi SSID        (max 32 chars + \0)
  char wifiPass[65];         // WiFi password    (max 64 chars + \0)

  char mqttServer[65];       // MQTT broker IP/host
  uint16_t mqttPort;         // MQTT port  (default 1883)
  char mqttUser[33];         // MQTT username
  char mqttPass[65];         // MQTT password

  char mqttPrefix[33];       // MQTT topic prefix  e.g. "home/fancontroller"
                             //   topics become:  <prefix>/relay/state  etc.

  char otaPass[33];          // ArduinoOTA password

  char deviceName[33];       // Human-readable device name (shown in HA)
  char roomName[33];         // Room name (used in HA device area)
};                           // Total: ≈ 393 bytes  (well within 512)

Config cfg;                  // active configuration in RAM

// ── EEPROM helpers ───────────────────────────────────────────

// Load config from EEPROM into cfg struct
void loadConfig()
{
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, cfg);
  EEPROM.end();
}

// Save cfg struct to EEPROM
void saveConfig()
{
  cfg.magic = CONFIG_MAGIC;
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, cfg);
  EEPROM.commit();
  EEPROM.end();
}

// Wipe EEPROM and restart into AP mode
void factoryReset()
{
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < EEPROM_SIZE; i++) EEPROM.write(i, 0);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("Factory reset – restarting into AP mode");
  delay(200);
  ESP.restart();
}

// Returns true if EEPROM contains a valid config
bool configValid()
{
  return cfg.magic == CONFIG_MAGIC;
}


// ════════════════════════════════════════════════════════════
//  3.  PIN DEFINITIONS & FIRMWARE VERSION
// ════════════════════════════════════════════════════════════
#define RELAY_PIN  D2          // GPIO4 – Active LOW relay
#define BOOT_BUTTON_PIN 0      // GPIO0 – FLASH/BOOT button on ESP8266
#define FW_VERSION "1.0.2"
static const char* MASK = "********";


// ════════════════════════════════════════════════════════════
//  4.  GLOBAL OBJECTS
// ════════════════════════════════════════════════════════════
WiFiClient       espClient;
PubSubClient     client(espClient);
ESP8266WebServer server(80);

// Flag set in setup() – true = normal mode, false = AP setup mode
bool normalMode = false;


// ════════════════════════════════════════════════════════════
//  5.  STATE & SSE
// ════════════════════════════════════════════════════════════
bool       relayState = false;   // true = ON, false = OFF
WiFiClient sseClient;            // holds the open SSE connection


// ════════════════════════════════════════════════════════════
//  6.  SSE PUSH HELPER
//  Pushes current state to an open browser EventSource.
//  Called from updateRelay() so HA→MQTT changes appear
//  in the browser within milliseconds (no 1-second poll wait).
// ════════════════════════════════════════════════════════════
void buildUptimeStr(char* buf, size_t len)
{
  unsigned long s   = millis() / 1000;
  unsigned long m   = s / 60;
  unsigned long h   = m / 60;
  unsigned long d   = h / 24;
  h %= 24; m %= 60; s %= 60;
  if (d > 0)       snprintf(buf, len, "%lud %luh %lum", d, h, m);
  else if (h > 0)  snprintf(buf, len, "%luh %lum", h, m);
  else if (m > 0)  snprintf(buf, len, "%lum %lus", m, s);
  else             snprintf(buf, len, "%lus", s);
}

void ssePush()
{
  if (!sseClient || !sseClient.connected()) return;

  char uptime[32];
  buildUptimeStr(uptime, sizeof(uptime));

  String json = "{";
  json += "\"fan\":"        + String(relayState ? "true" : "false") + ",";
  json += "\"wifi\":"       + String(WiFi.RSSI())                   + ",";
  json += "\"heap\":"       + String(ESP.getFreeHeap())             + ",";
  json += "\"ip\":\""       + WiFi.localIP().toString()             + "\",";
  json += "\"mqtt\":"       + String(client.connected() ? "true" : "false") + ",";
  json += "\"uptime\":\""   + String(uptime)                        + "\",";
  json += "\"firmware\":\"" FW_VERSION "\",";
  json += "\"device\":\""   + String(cfg.deviceName)               + "\"";
  json += "}";

  sseClient.print("data: ");
  sseClient.print(json);
  sseClient.print("\n\n");
}


// ════════════════════════════════════════════════════════════
//  7.  RELAY CONTROL  (single source of truth)
//  Both MQTT callback AND HTTP endpoints call this function.
//  Order: GPIO → state → MQTT publish → diagnostics → SSE.
// ════════════════════════════════════════════════════════════
void publishRSSI();   // forward declarations needed by updateRelay
void publishHeap();
void publishIP();

void updateRelay(bool state)
{
  relayState = state;
  digitalWrite(RELAY_PIN, state ? LOW : HIGH);  // Active LOW

  // Build topic from configured prefix
  String stateTopic = String(cfg.mqttPrefix) + "/relay/state";
  client.publish(stateTopic.c_str(), state ? "ON" : "OFF", true);

  // Push fresh diagnostics to HA immediately
  publishRSSI();
  publishHeap();
  publishIP();

  Serial.print("Relay -> ");
  Serial.println(state ? "ON" : "OFF");

  ssePush();
}


// ════════════════════════════════════════════════════════════
//  8.  MQTT HELPERS
// ════════════════════════════════════════════════════════════

// Convenience: build topic string from prefix
String mqttTopic(const char* suffix)
{
  return String(cfg.mqttPrefix) + "/" + suffix;
}

// ── Relay state ───────────────────────────────────────────────
void publishRelayState()
{
  client.publish(
    mqttTopic("relay/state").c_str(),
    relayState ? "ON" : "OFF",
    true
  );
}

// ── Firmware ──────────────────────────────────────────────────
void publishFirmware()
{
  client.publish(mqttTopic("firmware").c_str(), FW_VERSION, true);
}

// ── RSSI ──────────────────────────────────────────────────────
void publishRSSI()
{
  char buf[8];
  sprintf(buf, "%d", WiFi.RSSI());
  client.publish(mqttTopic("rssi").c_str(), buf, true);
}

// ── IP ────────────────────────────────────────────────────────
void publishIP()
{
  client.publish(
    mqttTopic("ip").c_str(),
    WiFi.localIP().toString().c_str(),
    true
  );
}

// ── Heap ──────────────────────────────────────────────────────
void publishHeap()
{
  char buf[12];
  sprintf(buf, "%u", ESP.getFreeHeap());
  client.publish(mqttTopic("heap").c_str(), buf, true);
}

// ── Home Assistant MQTT Discovery ─────────────────────────────
//  All payloads use cfg.deviceName, cfg.roomName, cfg.mqttPrefix
//  and FW_VERSION so HA shows the user-configured device name.

void publishDiscovery()
{
  String name      = String(cfg.deviceName);          // e.g. "Living Room Fan"
  String room      = String(cfg.roomName);            // e.g. "Living Room"
  String prefix    = String(cfg.mqttPrefix);
  String devId     = "esp8266_" + String(cfg.mqttPrefix); // unique per prefix

  // Switch entity
  String sw;
  sw  = "{";
  sw += "\"name\":\"" + name + "\",";
  sw += "\"object_id\":\"fan\",";
  sw += "\"unique_id\":\"" + devId + "_fan\",";
  sw += "\"command_topic\":\"" + prefix + "/relay/set\",";
  sw += "\"state_topic\":\""   + prefix + "/relay/state\",";
  sw += "\"payload_on\":\"ON\",\"payload_off\":\"OFF\",";
  sw += "\"state_on\":\"ON\",\"state_off\":\"OFF\",";
  sw += "\"availability_topic\":\"" + prefix + "/status\",";
  sw += "\"payload_available\":\"ONLINE\",";
  sw += "\"payload_not_available\":\"OFFLINE\",";
  sw += "\"device\":{";
  sw +=   "\"identifiers\":[\"" + devId + "\"],";
  sw +=   "\"name\":\"" + name + "\",";
  sw +=   "\"suggested_area\":\"" + room + "\",";
  sw +=   "\"manufacturer\":\"DIY\",";
  sw +=   "\"model\":\"ESP8266 WeMos D1\",";
  sw +=   "\"sw_version\":\"" FW_VERSION "\"";
  sw += "}}";

  bool ok = client.publish(
    ("homeassistant/switch/" + devId + "_fan/config").c_str(),
    sw.c_str(), true);
  Serial.print("Discovery (switch): ");
  Serial.println(ok ? "OK" : "FAIL");
}

void publishSensorDiscovery(
  const char* objId,
  const char* name,
  const char* stateSuffix,
  const char* unit,
  const char* devClass,
  const char* icon)
{
  String devId  = "esp8266_" + String(cfg.mqttPrefix);
  String prefix = String(cfg.mqttPrefix);

  String p;
  p  = "{";
  p += "\"name\":\"" + String(name) + "\",";
  p += "\"object_id\":\"" + String(objId) + "\",";
  p += "\"unique_id\":\"" + devId + "_" + String(objId) + "\",";
  p += "\"state_topic\":\"" + prefix + "/" + String(stateSuffix) + "\",";
  if (unit && strlen(unit) > 0)
    p += "\"unit_of_measurement\":\"" + String(unit) + "\",";
  if (devClass && strlen(devClass) > 0)
    p += "\"device_class\":\"" + String(devClass) + "\",";
  if (icon && strlen(icon) > 0)
    p += "\"icon\":\"" + String(icon) + "\",";
  p += "\"entity_category\":\"diagnostic\",";
  p += "\"device\":{\"identifiers\":[\"" + devId + "\"]}";
  p += "}";

  String topic = "homeassistant/sensor/" + devId + "_" + String(objId) + "/config";
  client.publish(topic.c_str(), p.c_str(), true);
}

void publishAllDiscovery()
{
  publishDiscovery();
  publishSensorDiscovery("wifi_signal", "WiFi Signal", "rssi",     "dBm",   "signal_strength", "");
  publishSensorDiscovery("ip_address",  "IP Address",  "ip",       "",      "",                "mdi:ip-network");
  publishSensorDiscovery("free_heap",   "Free Heap",   "heap",     "bytes", "",                "mdi:memory");
  publishSensorDiscovery("firmware",    "Firmware",    "firmware", "",      "",                "mdi:chip");
}


// ════════════════════════════════════════════════════════════
//  9.  MQTT CALLBACK
// ════════════════════════════════════════════════════════════
void callback(char* topic, byte* payload, unsigned int length)
{
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("MQTT cmd: "); Serial.println(msg);

  if (msg == "ON") updateRelay(true);
  else             updateRelay(false);
}


// ════════════════════════════════════════════════════════════
//  10.  MQTT RECONNECT  (non-blocking, millis-throttled)
// ════════════════════════════════════════════════════════════
void mqttReconnect()
{
  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt < 5000) return;
  lastAttempt = millis();

  Serial.print("Connecting MQTT...");

  String lwt = mqttTopic("status");
  String cmdTopic = mqttTopic("relay/set");

  if (client.connect(
        cfg.deviceName,          // client ID = device name
        cfg.mqttUser,
        cfg.mqttPass,
        lwt.c_str(), 1, true, "OFFLINE"))
  {
    Serial.println("Connected");
    client.subscribe(cmdTopic.c_str());
    client.publish(lwt.c_str(), "ONLINE", true);

    publishAllDiscovery();
    publishRelayState();
    publishRSSI();
    publishIP();
    publishHeap();
    publishFirmware();
  }
  else
  {
    Serial.print("Failed: "); Serial.println(client.state());
  }
}


// ════════════════════════════════════════════════════════════
//  11.  AP SETUP HTML  (PROGMEM)
//  Served when the device boots without a valid config.
// ════════════════════════════════════════════════════════════
const char AP_SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>ESP8266 Setup</title>
  <link rel="preconnect" href="https://fonts.googleapis.com"/>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet"/>
  <style>
    html{overflow-y:scroll;scrollbar-width:thin;scrollbar-color:rgba(0,180,216,.3) transparent}
    *,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
    :root{
      --bg:#03045e;--s1:rgba(0,119,182,.18);--s2:rgba(0,180,216,.12);
      --surface:rgba(0,180,216,.07);--border:rgba(144,224,239,.15);
      --accent:#00b4d8;--accent2:#90e0ef;--text:#caf0f8;--muted:#90e0ef;
      --green:#22d3a5;--red:#ff6b8a;
    }
    body{
      font-family:'Inter',sans-serif;background:var(--bg);color:var(--text);
      min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px;
      background:linear-gradient(135deg,#03045e 0%,#023e8a 40%,#0077b6 70%,#00b4d8 100%);
      background-attachment:fixed;
    }
    .wrap{width:100%;max-width:480px}
    h1{
      font-size:1.8rem;font-weight:700;text-align:center;margin-bottom:6px;
      background:linear-gradient(135deg,var(--accent2),var(--accent));
      -webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;
    }
    .sub{text-align:center;font-size:.82rem;color:var(--muted);margin-bottom:24px;opacity:.8}
    .card{
      background:rgba(0,119,182,.18);border:1px solid var(--border);border-radius:20px;
      backdrop-filter:blur(20px);-webkit-backdrop-filter:blur(20px);
      padding:22px 24px;margin-bottom:14px;
      box-shadow:0 8px 32px rgba(0,0,0,.25),inset 0 1px 0 rgba(144,224,239,.1);
    }
    .section-title{
      font-size:.68rem;font-weight:700;letter-spacing:2.5px;text-transform:uppercase;
      color:var(--accent);margin-bottom:14px;padding-bottom:8px;
      border-bottom:1px solid rgba(0,180,216,.2);
    }
    .field{margin-bottom:14px}
    label{display:block;font-size:.78rem;font-weight:500;color:var(--muted);margin-bottom:5px}
    input{
      width:100%;padding:11px 14px;
      background:rgba(0,180,216,.08);border:1px solid rgba(144,224,239,.2);
      border-radius:10px;color:var(--text);font-family:'Inter',sans-serif;
      font-size:.9rem;outline:none;transition:border-color .2s,box-shadow .2s;
    }
    input:focus{border-color:var(--accent);box-shadow:0 0 0 3px rgba(0,180,216,.18)}
    input::placeholder{color:rgba(202,240,248,.35)}
    .row{display:grid;grid-template-columns:1fr 100px;gap:10px}
    .btn{
      width:100%;padding:14px;border:none;border-radius:12px;
      font-family:'Inter',sans-serif;font-size:1rem;font-weight:700;
      cursor:pointer;transition:transform .15s,box-shadow .2s;
      background:linear-gradient(135deg,#0077b6,#00b4d8);color:#03045e;
      box-shadow:0 4px 20px rgba(0,180,216,.45);margin-top:8px;letter-spacing:.5px;
    }
    .btn:hover{transform:translateY(-2px);box-shadow:0 6px 28px rgba(0,180,216,.65)}
    .btn:active{transform:scale(.97)}
    #msg{
      text-align:center;padding:12px;border-radius:10px;
      font-weight:600;font-size:.9rem;margin-top:12px;display:none;
    }
    .msg-ok{background:rgba(34,211,165,.15);color:var(--green);border:1px solid rgba(34,211,165,.3)}
    .msg-err{background:rgba(255,107,138,.15);color:var(--red);border:1px solid rgba(255,107,138,.3)}
  </style>
</head>
<body>
<div class="wrap">
  <h1>&#x26A1; Device Setup</h1>
  <p class="sub">Configure your ESP8266 Fan Controller</p>

  <form id="form">
    <div class="card">
      <div class="section-title">WiFi Settings</div>
      <div class="field"><label>SSID</label><input name="wifiSSID" placeholder="Your WiFi name" required maxlength="32"/></div>
      <div class="field"><label>Password</label><input name="wifiPass" type="password" placeholder="WiFi password" maxlength="64"/></div>
    </div>
    <div class="card">
      <div class="section-title">MQTT Settings</div>
      <div class="field"><label>Server</label><input name="mqttServer" placeholder="192.168.1.10" required maxlength="64"/></div>
      <div class="field row">
        <div><label>Username</label><input name="mqttUser" placeholder="mqtt_user" maxlength="32"/></div>
        <div><label>Port</label><input name="mqttPort" type="number" value="1883" min="1" max="65535"/></div>
      </div>
      <div class="field"><label>Password</label><input name="mqttPass" type="password" placeholder="mqtt_password" maxlength="64"/></div>
      <div class="field"><label>Topic Prefix <span style="opacity:.6;font-weight:400">(no trailing slash)</span></label>
        <input name="mqttPrefix" placeholder="home/fancontroller" value="home/fancontroller" maxlength="32" required/>
      </div>
    </div>
    <div class="card">
      <div class="section-title">OTA Settings</div>
      <div class="field"><label>OTA Password</label><input name="otaPass" type="password" placeholder="ota_password" maxlength="32"/></div>
    </div>
    <div class="card">
      <div class="section-title">Device Settings</div>
      <div class="field"><label>Device Name <span style="opacity:.6;font-weight:400">(shown in Home Assistant)</span></label>
        <input name="deviceName" placeholder="Living Room Fan" value="Fan Controller" required maxlength="32"/>
      </div>
      <div class="field"><label>Room Name</label>
        <input name="roomName" placeholder="Living Room" maxlength="32"/>
      </div>
    </div>
    <button type="submit" class="btn">&#x1F4BE; Save &amp; Connect</button>
    <div id="msg"></div>
  </form>
</div>
<script>
  document.getElementById('form').addEventListener('submit', function(e) {
    e.preventDefault();
    var btn = this.querySelector('button');
    btn.disabled = true; btn.textContent = 'Saving...';
    var data = {};
    new FormData(this).forEach(function(v, k){ data[k] = v; });
    data.mqttPort = parseInt(data.mqttPort) || 1883;
    fetch('/save', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify(data)
    })
    .then(function(r){ return r.json(); })
    .then(function(d){
      var msg = document.getElementById('msg');
      if (d.ok) {
        msg.className = 'msg-ok'; msg.style.display = 'block';
        msg.textContent = 'Saved! Restarting in 3 seconds...';
      } else {
        msg.className = 'msg-err'; msg.style.display = 'block';
        msg.textContent = 'Error: ' + (d.error || 'unknown');
        btn.disabled = false; btn.textContent = 'Save & Connect';
      }
    })
    .catch(function(){
      var msg = document.getElementById('msg');
      msg.className='msg-err'; msg.style.display='block';
      msg.textContent='Connection error. Try again.';
      btn.disabled=false; btn.textContent='Save & Connect';
    });
  });
</script>
</body>
</html>
)rawliteral";


// ════════════════════════════════════════════════════════════
//  12.  DASHBOARD HTML  (PROGMEM)
// ════════════════════════════════════════════════════════════
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Fan Controller</title>
  <link rel="preconnect" href="https://fonts.googleapis.com"/>
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin/>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap" rel="stylesheet"/>
  <style>
    /* Fix scrollbar flicker – always reserve space for vertical scrollbar */
    html{overflow-y:scroll;scrollbar-width:thin;scrollbar-color:rgba(0,180,216,.25) transparent}
    *,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
    :root{
      --deep:#03045e;--primary:#0077b6;--accent:#00b4d8;--frost:#90e0ef;--light:#caf0f8;
      --surface:rgba(0,119,182,.14);--border:rgba(144,224,239,.16);
      --on:#22d3a5;--off:#ff6b8a;--blur:18px;--r:18px;
    }
    body{
      font-family:'Inter',sans-serif;color:var(--light);
      min-height:100vh;padding:20px 16px 48px;
      background:var(--deep);
      background-image:
        radial-gradient(ellipse 90% 60% at 50% -10%,rgba(0,119,182,.5) 0%,transparent 65%),
        radial-gradient(ellipse 70% 50% at 100% 110%,rgba(0,180,216,.2) 0%,transparent 60%),
        radial-gradient(ellipse 50% 40% at 0% 80%,rgba(3,4,94,.8) 0%,transparent 60%);
      background-attachment:fixed;
    }
    /* ── Header ───────────────────────────────────────── */
    .header{text-align:center;margin-bottom:24px;padding-top:4px}
    .header h1{
      font-size:clamp(1.5rem,5vw,2.1rem);font-weight:700;letter-spacing:-.5px;
      background:linear-gradient(135deg,var(--frost) 0%,var(--accent) 60%,#00e5ff 100%);
      -webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;
    }
    .header-meta{
      display:flex;align-items:center;justify-content:center;gap:10px;
      margin-top:6px;flex-wrap:wrap;
    }
    .header-meta span{font-size:.75rem;color:var(--frost);opacity:.7}
    #device-name{color:var(--accent);font-weight:600;opacity:1!important}
    #live-clock{font-size:.8rem;font-weight:600;color:var(--accent);letter-spacing:1.5px;font-variant-numeric:tabular-nums;opacity:1!important}
    .conn-dot{
      width:8px;height:8px;border-radius:50%;background:var(--on);
      display:inline-block;flex-shrink:0;
      animation:blink 1.8s ease-in-out infinite;
    }
    .conn-dot.offline{background:var(--off);animation:none}
    @keyframes blink{
      0%,100%{opacity:1;box-shadow:0 0 0 0 rgba(34,211,165,.5)}
      60%{opacity:.5;box-shadow:0 0 0 6px rgba(34,211,165,0)}
    }
    /* ── Nav ──────────────────────────────────────────── */
    .nav{display:flex;justify-content:center;gap:8px;margin-bottom:18px}
    .nav a{
      padding:7px 20px;border-radius:999px;font-size:.8rem;font-weight:600;
      text-decoration:none;color:var(--frost);opacity:.65;
      background:rgba(0,119,182,.15);border:1px solid var(--border);
      transition:all .2s;
    }
    .nav a.active,.nav a:hover{
      opacity:1;border-color:var(--accent);
      color:var(--light);background:rgba(0,180,216,.2);
      box-shadow:0 0 16px rgba(0,180,216,.2);
    }
    /* ── Loading overlay ─────────────────────────────── */
    #loader{
      position:fixed;inset:0;
      background:var(--deep);
      display:flex;flex-direction:column;align-items:center;justify-content:center;
      z-index:999;transition:opacity .5s;
    }
    #loader.hidden{opacity:0;pointer-events:none}
    .spinner{
      width:52px;height:52px;
      border:4px solid rgba(0,180,216,.15);
      border-top-color:var(--accent);
      border-radius:50%;animation:spin-l .75s linear infinite;margin-bottom:18px;
    }
    @keyframes spin-l{to{transform:rotate(360deg)}}
    #loader p{font-size:.85rem;color:var(--frost);opacity:.7}
    /* ── Glass card ──────────────────────────────────── */
    .card{
      background:var(--surface);border:1px solid var(--border);border-radius:var(--r);
      backdrop-filter:blur(var(--blur));-webkit-backdrop-filter:blur(var(--blur));
      padding:20px 22px;margin-bottom:14px;
      box-shadow:0 8px 32px rgba(0,0,0,.3),inset 0 1px 0 rgba(144,224,239,.08);
      transition:border-color .3s;
    }
    /* ── Fan status card ─────────────────────────────── */
    .status-card{
      text-align:center;padding:32px 22px;
      background:linear-gradient(145deg,rgba(0,119,182,.2),rgba(0,180,216,.1));
      border-color:rgba(0,180,216,.25);
    }
    .status-label{
      font-size:.68rem;font-weight:700;letter-spacing:3px;text-transform:uppercase;
      color:var(--frost);opacity:.7;margin-bottom:14px;
    }
    .fan-icon{
      font-size:3.5rem;line-height:1;margin-bottom:12px;display:block;
      filter:drop-shadow(0 0 16px rgba(0,180,216,.5));
    }
    .fan-icon.spinning{animation:spin 1.1s linear infinite}
    @keyframes spin{from{transform:rotate(0deg)}to{transform:rotate(360deg)}}
    .fan-status-text{
      font-size:2rem;font-weight:800;letter-spacing:2px;transition:color .3s;
      text-shadow:0 0 20px currentColor;
    }
    .fan-status-text.on{color:var(--on)}
    .fan-status-text.off{color:var(--off)}
    /* ── Buttons ─────────────────────────────────────── */
    .btn-row{display:flex;gap:12px;margin-top:24px}
    .btn{
      flex:1;padding:16px 10px;border:none;border-radius:14px;
      font-family:'Inter',sans-serif;font-size:1rem;font-weight:700;
      letter-spacing:1px;cursor:pointer;transition:transform .15s,box-shadow .2s;
      position:relative;overflow:hidden;
    }
    .btn::after{
      content:'';position:absolute;inset:0;
      background:rgba(255,255,255,.12);opacity:0;transition:opacity .15s;
    }
    .btn:active::after{opacity:1}
    .btn:active{transform:scale(.96)}
    .btn-on{
      background:linear-gradient(135deg,#05b187,#22d3a5);color:#03045e;
      box-shadow:0 4px 20px rgba(34,211,165,.4);
    }
    .btn-on:hover{box-shadow:0 6px 28px rgba(34,211,165,.65);transform:translateY(-2px)}
    .btn-off{
      background:linear-gradient(135deg,#c9184a,#ff6b8a);color:#fff;
      box-shadow:0 4px 20px rgba(255,107,138,.4);
    }
    .btn-off:hover{box-shadow:0 6px 28px rgba(255,107,138,.65);transform:translateY(-2px)}
    /* ── Stats grid ──────────────────────────────────── */
    .stats-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
    @media(min-width:500px){.stats-grid{grid-template-columns:repeat(3,1fr)}}
    .stat-card{
      background:var(--surface);border:1px solid var(--border);
      border-radius:14px;backdrop-filter:blur(var(--blur));-webkit-backdrop-filter:blur(var(--blur));
      padding:18px 14px;text-align:center;transition:border-color .25s,transform .2s;
    }
    .stat-card:hover{border-color:rgba(0,180,216,.4);transform:translateY(-2px)}
    .stat-icon{font-size:1.5rem;margin-bottom:6px;display:block;filter:drop-shadow(0 0 8px rgba(0,180,216,.4))}
    .stat-value{
      font-size:.95rem;font-weight:700;color:var(--light);
      word-break:break-all;line-height:1.3;
    }
    .rssi-quality{font-size:.68rem;font-weight:600;margin-top:2px;letter-spacing:.5px}
    .rssi-excellent{color:#22d3a5}.rssi-good{color:#84cc16}.rssi-fair{color:#fbbf24}.rssi-poor{color:#ff6b8a}
    .stat-label{
      font-size:.62rem;font-weight:600;letter-spacing:2px;text-transform:uppercase;
      color:var(--frost);opacity:.6;margin-top:6px;
    }
    /* ── MQTT badge ──────────────────────────────────── */
    .badge{
      display:inline-flex;align-items:center;gap:5px;
      padding:4px 11px;border-radius:999px;
      font-size:.68rem;font-weight:700;letter-spacing:1px;text-transform:uppercase;
    }
    .badge-ok{background:rgba(34,211,165,.15);color:#22d3a5;border:1px solid rgba(34,211,165,.3)}
    .badge-err{background:rgba(255,107,138,.15);color:#ff6b8a;border:1px solid rgba(255,107,138,.3)}
    /* ── Footer ──────────────────────────────────────── */
    .footer{
      text-align:center;font-size:.7rem;color:var(--frost);
      opacity:.4;margin-top:10px;
    }
    /* ── Pulse (loading placeholder) ─────────────────── */
    .pulse{animation:pulse 1.8s ease-in-out infinite}
    @keyframes pulse{0%,100%{opacity:.8}50%{opacity:.3}}
  </style>
</head>
<body>

<div id="loader"><div class="spinner"></div><p>Connecting to controller&hellip;</p></div>

<div class="header">
  <h1>&#x1F4A8; <span id="device-name">Fan Controller</span></h1>
  <div class="header-meta">
    <span id="live-clock">--:--:--</span>
    <span id="conn-dot" class="conn-dot offline"></span>
    <span id="fw-label">v&hellip;</span>
  </div>
</div>

<nav class="nav">
  <a href="/" class="active">Dashboard</a>
  <a href="/settings">&#x2699;&#xFE0F; Settings</a>
</nav>

<div class="card status-card">
  <div class="status-label">Fan Status</div>
  <span id="fan-icon" class="fan-icon">&#x1F300;</span>
  <div id="fan-status" class="fan-status-text off">&#x2014;</div>
  <div class="btn-row">
    <button class="btn btn-on"  onclick="fanControl('on')">&#x25B6; ON</button>
    <button class="btn btn-off" onclick="fanControl('off')">&#x25A0; OFF</button>
  </div>
</div>

<div class="stats-grid">
  <div class="stat-card">
    <span class="stat-icon">&#x1F4F6;</span>
    <div id="stat-rssi"   class="stat-value pulse">&#x2026;</div>
    <div id="stat-rssi-q" class="rssi-quality pulse">&#x2026;</div>
    <div class="stat-label">WiFi RSSI</div>
  </div>
  <div class="stat-card">
    <span class="stat-icon">&#x1F310;</span>
    <div id="stat-ip"     class="stat-value pulse">&#x2026;</div>
    <div class="stat-label">IP Address</div>
  </div>
  <div class="stat-card">
    <span class="stat-icon">&#x1F9E0;</span>
    <div id="stat-heap"   class="stat-value pulse">&#x2026;</div>
    <div class="stat-label">Free Heap</div>
  </div>
  <div class="stat-card">
    <span class="stat-icon">&#x1F517;</span>
    <div id="stat-mqtt"   class="stat-value pulse">&#x2026;</div>
    <div class="stat-label">MQTT</div>
  </div>
  <div class="stat-card">
    <span class="stat-icon">&#x23F1;&#xFE0F;</span>
    <div id="stat-uptime" class="stat-value pulse">&#x2026;</div>
    <div class="stat-label">Uptime</div>
  </div>
  <div class="stat-card">
    <span class="stat-icon">&#x1F527;</span>
    <div id="stat-fw"     class="stat-value pulse">&#x2026;</div>
    <div class="stat-label">Firmware</div>
  </div>
</div>

<div class="footer">ESP8266 &middot; WeMos D1 &middot; Auto-refresh 1s + SSE push</div>

<script>
  var firstLoad = true;

  function rssiQuality(dbm) {
    if (dbm >= -50) return {label:'Excellent', cls:'rssi-excellent'};
    if (dbm >= -65) return {label:'Good',      cls:'rssi-good'};
    if (dbm >= -75) return {label:'Fair',       cls:'rssi-fair'};
    return                 {label:'Poor',       cls:'rssi-poor'};
  }

  function formatBytes(b) {
    if (b >= 1024) return (b/1024).toFixed(1)+' KB';
    return b+' B';
  }

  function applyStatus(d) {
    if (firstLoad) {
      firstLoad = false;
      var l = document.getElementById('loader');
      l.classList.add('hidden');
      setTimeout(function(){ l.style.display='none'; }, 500);
    }
    var fanEl = document.getElementById('fan-status');
    var iconEl = document.getElementById('fan-icon');
    if (d.fan) {
      fanEl.textContent='ON'; fanEl.className='fan-status-text on';
      iconEl.className='fan-icon spinning';
    } else {
      fanEl.textContent='OFF'; fanEl.className='fan-status-text off';
      iconEl.className='fan-icon';
    }
    var q = rssiQuality(d.wifi);
    document.getElementById('stat-rssi').textContent = d.wifi+' dBm';
    var qEl = document.getElementById('stat-rssi-q');
    qEl.textContent = q.label; qEl.className = 'rssi-quality '+q.cls;
    document.getElementById('stat-ip').textContent     = d.ip;
    document.getElementById('stat-heap').textContent   = formatBytes(d.heap);
    document.getElementById('stat-uptime').textContent = d.uptime;
    document.getElementById('stat-fw').textContent     = 'v'+d.firmware;
    document.getElementById('fw-label').textContent    = 'v'+d.firmware;
    if (d.device) document.getElementById('device-name').textContent = d.device;
    var mqttEl = document.getElementById('stat-mqtt');
    if (d.mqtt) mqttEl.innerHTML='<span class="badge badge-ok"><span class="conn-dot" style="width:6px;height:6px"></span>Online</span>';
    else        mqttEl.innerHTML='<span class="badge badge-err">Offline</span>';
    document.getElementById('conn-dot').className = 'conn-dot'+(d.mqtt?'':' offline');
    document.querySelectorAll('.pulse').forEach(function(el){ el.classList.remove('pulse'); });
  }

  function refreshStatus() {
    fetch('/status').then(function(r){return r.json();}).then(applyStatus).catch(function(){});
  }

  function fanControl(state) {
    fetch('/fan/'+state).then(function(r){return r.json();}).then(applyStatus).catch(function(){});
  }

  function tickClock() {
    var now = new Date();
    document.getElementById('live-clock').textContent =
      String(now.getHours()).padStart(2,'0')+':'+
      String(now.getMinutes()).padStart(2,'0')+':'+
      String(now.getSeconds()).padStart(2,'0');
  }
  tickClock(); setInterval(tickClock, 1000);

  if (typeof EventSource !== 'undefined') {
    var es = new EventSource('/events');
    es.onmessage = function(e){ try{ applyStatus(JSON.parse(e.data)); }catch(err){} };
  }

  refreshStatus();
  setInterval(refreshStatus, 1000);
</script>
</body>
</html>
)rawliteral";


// ════════════════════════════════════════════════════════════
//  13.  SETTINGS HTML  (PROGMEM)
// ════════════════════════════════════════════════════════════
const char SETTINGS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>Settings &ndash; Fan Controller</title>
  <link rel="preconnect" href="https://fonts.googleapis.com"/>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet"/>
  <style>
    html{overflow-y:scroll;scrollbar-width:thin;scrollbar-color:rgba(0,180,216,.25) transparent}
    *,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
    :root{
      --deep:#03045e;--primary:#0077b6;--accent:#00b4d8;--frost:#90e0ef;--light:#caf0f8;
      --surface:rgba(0,119,182,.14);--border:rgba(144,224,239,.16);
      --on:#22d3a5;--red:#ff6b8a;--orange:#fbbf24;--blur:18px;--r:18px;
    }
    body{
      font-family:'Inter',sans-serif;color:var(--light);
      min-height:100vh;padding:20px 16px 60px;
      background:var(--deep);
      background-image:
        radial-gradient(ellipse 90% 60% at 50% -10%,rgba(0,119,182,.5) 0%,transparent 65%),
        radial-gradient(ellipse 60% 40% at 100% 110%,rgba(0,180,216,.18) 0%,transparent 60%);
      background-attachment:fixed;
    }
    .header{text-align:center;margin-bottom:24px}
    .header h1{
      font-size:clamp(1.4rem,5vw,2rem);font-weight:700;
      background:linear-gradient(135deg,var(--frost),var(--accent));
      -webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;
    }
    /* Nav */
    .nav{display:flex;justify-content:center;gap:8px;margin-bottom:20px}
    .nav a{
      padding:7px 20px;border-radius:999px;font-size:.8rem;font-weight:600;
      text-decoration:none;color:var(--frost);opacity:.65;
      background:rgba(0,119,182,.15);border:1px solid var(--border);transition:all .2s;
    }
    .nav a.active,.nav a:hover{
      opacity:1;border-color:var(--accent);color:var(--light);
      background:rgba(0,180,216,.2);box-shadow:0 0 16px rgba(0,180,216,.2);
    }
    /* Card */
    .card{
      background:var(--surface);border:1px solid var(--border);border-radius:var(--r);
      backdrop-filter:blur(var(--blur));-webkit-backdrop-filter:blur(var(--blur));
      padding:22px 24px;margin-bottom:14px;
      box-shadow:0 8px 32px rgba(0,0,0,.25),inset 0 1px 0 rgba(144,224,239,.08);
    }
    .section-title{
      font-size:.68rem;font-weight:700;letter-spacing:2.5px;text-transform:uppercase;
      color:var(--accent);margin-bottom:16px;padding-bottom:8px;
      border-bottom:1px solid rgba(0,180,216,.2);
    }
    .field{margin-bottom:14px}
    label{display:block;font-size:.78rem;font-weight:500;color:var(--frost);margin-bottom:5px;opacity:.8}
    .pw-wrap{position:relative}
    input{
      width:100%;padding:11px 14px;
      background:rgba(0,180,216,.07);border:1px solid rgba(144,224,239,.2);
      border-radius:10px;color:var(--light);font-family:'Inter',sans-serif;
      font-size:.9rem;outline:none;transition:border-color .2s,box-shadow .2s;
    }
    input:focus{border-color:var(--accent);box-shadow:0 0 0 3px rgba(0,180,216,.18)}
    input::placeholder{color:rgba(202,240,248,.3)}
    .pw-wrap input{padding-right:44px}
    .pw-toggle{
      position:absolute;right:12px;top:50%;transform:translateY(-50%);
      background:none;border:none;color:var(--frost);cursor:pointer;
      font-size:1rem;padding:4px;line-height:1;opacity:.6;transition:opacity .2s;
    }
    .pw-toggle:hover{opacity:1}
    .row2{display:grid;grid-template-columns:1fr 110px;gap:10px}
    /* Action buttons */
    .action-bar{display:flex;flex-direction:column;gap:10px;margin-top:4px}
    .btn{
      width:100%;padding:14px;border:none;border-radius:12px;
      font-family:'Inter',sans-serif;font-size:.95rem;font-weight:700;
      cursor:pointer;transition:transform .15s,box-shadow .2s;
    }
    .btn:active{transform:scale(.97)}
    .btn-save{
      background:linear-gradient(135deg,var(--primary),var(--accent));color:#03045e;
      box-shadow:0 4px 20px rgba(0,180,216,.4);
    }
    .btn-save:hover{transform:translateY(-2px);box-shadow:0 6px 28px rgba(0,180,216,.6)}
    .btn-restart{
      background:rgba(251,191,36,.1);color:var(--orange);
      border:1px solid rgba(251,191,36,.3);
    }
    .btn-restart:hover{background:rgba(251,191,36,.2);transform:translateY(-2px)}
    .btn-reset{
      background:rgba(255,107,138,.08);color:var(--red);
      border:1px solid rgba(255,107,138,.28);
    }
    .btn-reset:hover{background:rgba(255,107,138,.18);transform:translateY(-2px)}
    /* Toast */
    #toast{
      position:fixed;bottom:24px;left:50%;transform:translateX(-50%) translateY(80px);
      padding:12px 26px;border-radius:12px;font-weight:600;font-size:.9rem;
      opacity:0;transition:all .35s;white-space:nowrap;z-index:999;pointer-events:none;
      backdrop-filter:blur(12px);
    }
    #toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
    .toast-ok{background:rgba(34,211,165,.18);color:var(--on);border:1px solid rgba(34,211,165,.35)}
    .toast-err{background:rgba(255,107,138,.18);color:var(--red);border:1px solid rgba(255,107,138,.35)}
    /* Confirm overlay */
    #confirm-overlay{
      display:none;position:fixed;inset:0;
      background:rgba(3,4,94,.75);z-index:998;
      align-items:center;justify-content:center;
      backdrop-filter:blur(6px);
    }
    #confirm-overlay.show{display:flex}
    .confirm-box{
      background:rgba(0,36,80,.9);border:1px solid var(--border);border-radius:20px;
      padding:28px 28px 22px;max-width:340px;width:90%;text-align:center;
      box-shadow:0 20px 60px rgba(0,0,0,.5);
    }
    .confirm-box h3{font-size:1.1rem;margin-bottom:8px;color:var(--red)}
    .confirm-box p{font-size:.85rem;color:var(--frost);opacity:.8;margin-bottom:20px;line-height:1.6}
    .confirm-btns{display:flex;gap:10px}
    .confirm-btns button{
      flex:1;padding:11px;border-radius:10px;
      font-family:'Inter',sans-serif;font-weight:600;cursor:pointer;
      border:none;font-size:.9rem;
    }
    #confirm-cancel{background:rgba(0,180,216,.12);color:var(--frost);border:1px solid var(--border)}
    #confirm-ok{background:var(--red);color:#fff}
  </style>
</head>
<body>

<div class="header">
  <h1>&#x2699;&#xFE0F; Settings</h1>
</div>

<nav class="nav">
  <a href="/">Dashboard</a>
  <a href="/settings" class="active">&#x2699;&#xFE0F; Settings</a>
</nav>

<form id="settings-form">

  <div class="card">
    <div class="section-title">WiFi Settings</div>
    <div class="field"><label>SSID</label><input id="wifiSSID" name="wifiSSID" placeholder="Your WiFi name" maxlength="32"/></div>
    <div class="field"><label>Password</label>
      <div class="pw-wrap">
        <input id="wifiPass" name="wifiPass" type="password" placeholder="Leave blank to keep current" maxlength="64"/>
        <button type="button" class="pw-toggle" onclick="togglePw('wifiPass',this)">&#x1F441;</button>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="section-title">MQTT Settings</div>
    <div class="field"><label>Server</label><input id="mqttServer" name="mqttServer" placeholder="192.168.1.10" maxlength="64"/></div>
    <div class="field row2">
      <div><label>Username</label><input id="mqttUser" name="mqttUser" maxlength="32"/></div>
      <div><label>Port</label><input id="mqttPort" name="mqttPort" type="number" min="1" max="65535"/></div>
    </div>
    <div class="field"><label>Password</label>
      <div class="pw-wrap">
        <input id="mqttPass" name="mqttPass" type="password" placeholder="Leave blank to keep current" maxlength="64"/>
        <button type="button" class="pw-toggle" onclick="togglePw('mqttPass',this)">&#x1F441;</button>
      </div>
    </div>
    <div class="field"><label>Topic Prefix <span style="opacity:.6;font-weight:400">(no trailing slash)</span></label>
      <input id="mqttPrefix" name="mqttPrefix" placeholder="home/fancontroller" maxlength="32"/>
    </div>
  </div>

  <div class="card">
    <div class="section-title">OTA Settings</div>
    <div class="field"><label>OTA Password</label>
      <div class="pw-wrap">
        <input id="otaPass" name="otaPass" type="password" placeholder="Leave blank to keep current" maxlength="32"/>
        <button type="button" class="pw-toggle" onclick="togglePw('otaPass',this)">&#x1F441;</button>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="section-title">Device Settings</div>
    <div class="field"><label>Device Name <span style="opacity:.6;font-weight:400">(shown in Home Assistant)</span></label>
      <input id="deviceName" name="deviceName" maxlength="32"/>
    </div>
    <div class="field"><label>Room Name</label>
      <input id="roomName" name="roomName" maxlength="32"/>
    </div>
  </div>

  <div class="action-bar">
    <button type="submit" class="btn btn-save">&#x1F4BE; Save Settings</button>
    <button type="button" class="btn btn-restart" onclick="doRestart()">&#x1F504; Restart ESP</button>
    <button type="button" class="btn btn-reset"   onclick="showConfirm()">&#x26A0;&#xFE0F; Factory Reset</button>
  </div>

</form>

<div id="confirm-overlay">
  <div class="confirm-box">
    <h3>&#x26A0; Factory Reset</h3>
    <p>This will clear all saved settings and return the device to AP setup mode. Are you sure?</p>
    <div class="confirm-btns">
      <button id="confirm-cancel" onclick="hideConfirm()">Cancel</button>
      <button id="confirm-ok"     onclick="doReset()">Reset</button>
    </div>
  </div>
</div>

<div id="toast"></div>

<script>
  var MASK = '********';

  function showToast(msg, cls, dur) {
    var t = document.getElementById('toast');
    t.textContent = msg;
    t.className = 'show ' + cls;
    setTimeout(function(){ t.className = ''; }, dur || 3000);
  }

  function togglePw(id, btn) {
    var inp = document.getElementById(id);
    inp.type = inp.type === 'password' ? 'text' : 'password';
    btn.textContent = inp.type === 'password' ? '\uD83D\uDC41' : '\uD83D\uDE48';
  }

  fetch('/api/settings')
    .then(function(r){ return r.json(); })
    .then(function(d){
      document.getElementById('wifiSSID').value    = d.wifiSSID    || '';
      document.getElementById('wifiPass').value    = d.wifiPass    || '';
      document.getElementById('mqttServer').value  = d.mqttServer  || '';
      document.getElementById('mqttPort').value    = d.mqttPort    || 1883;
      document.getElementById('mqttUser').value    = d.mqttUser    || '';
      document.getElementById('mqttPass').value    = d.mqttPass    || '';
      document.getElementById('mqttPrefix').value  = d.mqttPrefix  || '';
      document.getElementById('otaPass').value     = d.otaPass     || '';
      document.getElementById('deviceName').value  = d.deviceName  || '';
      document.getElementById('roomName').value    = d.roomName    || '';
    })
    .catch(function(){ showToast('Could not load settings', 'toast-err'); });

  document.getElementById('settings-form').addEventListener('submit', function(e){
    e.preventDefault();
    var btn = this.querySelector('.btn-save');
    btn.disabled = true; btn.textContent = 'Saving...';
    var data = {
      wifiSSID:   document.getElementById('wifiSSID').value.trim(),
      wifiPass:   document.getElementById('wifiPass').value,
      mqttServer: document.getElementById('mqttServer').value.trim(),
      mqttPort:   parseInt(document.getElementById('mqttPort').value) || 1883,
      mqttUser:   document.getElementById('mqttUser').value.trim(),
      mqttPass:   document.getElementById('mqttPass').value,
      mqttPrefix: document.getElementById('mqttPrefix').value.trim(),
      otaPass:    document.getElementById('otaPass').value,
      deviceName: document.getElementById('deviceName').value.trim(),
      roomName:   document.getElementById('roomName').value.trim()
    };
    if (!data.wifiSSID)   { showToast('WiFi SSID is required',   'toast-err'); btn.disabled=false; btn.textContent='Save Settings'; return; }
    if (!data.mqttServer) { showToast('MQTT Server is required',  'toast-err'); btn.disabled=false; btn.textContent='Save Settings'; return; }
    if (!data.deviceName) { showToast('Device Name is required',  'toast-err'); btn.disabled=false; btn.textContent='Save Settings'; return; }
    if (!data.mqttPrefix) { showToast('Topic Prefix is required', 'toast-err'); btn.disabled=false; btn.textContent='Save Settings'; return; }
    fetch('/api/settings', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify(data)
    })
    .then(function(r){ return r.json(); })
    .then(function(d){
      if (d.ok) { showToast('Saved! Restarting...', 'toast-ok', 5000); }
      else { showToast('Error: '+(d.error||'unknown'), 'toast-err'); btn.disabled=false; btn.textContent='Save Settings'; }
    })
    .catch(function(){
      showToast('Connection error', 'toast-err');
      btn.disabled=false; btn.textContent='Save Settings';
    });
  });

  function doRestart() {
    fetch('/api/restart', {method:'POST'})
      .then(function(){ showToast('Restarting...', 'toast-ok', 5000); })
      .catch(function(){});
  }

  function showConfirm() { document.getElementById('confirm-overlay').classList.add('show'); }
  function hideConfirm() { document.getElementById('confirm-overlay').classList.remove('show'); }
  function doReset() {
    hideConfirm();
    fetch('/api/reset', {method:'POST'})
      .then(function(){ showToast('Factory reset! Device restarting...', 'toast-ok', 6000); })
      .catch(function(){});
  }
</script>
</body>
</html>
)rawliteral";


// ════════════════════════════════════════════════════════════
//  14.  HTTP HANDLERS
// ════════════════════════════════════════════════════════════

// ── Helper: build uptime + status JSON ───────────────────────
String buildStatusJSON()
{
  char uptime[32];
  buildUptimeStr(uptime, sizeof(uptime));

  String json = "{";
  json += "\"fan\":"        + String(relayState ? "true" : "false") + ",";
  json += "\"wifi\":"       + String(WiFi.RSSI())                   + ",";
  json += "\"heap\":"       + String(ESP.getFreeHeap())             + ",";
  json += "\"ip\":\""       + WiFi.localIP().toString()             + "\",";
  json += "\"mqtt\":"       + String(client.connected() ? "true" : "false") + ",";
  json += "\"uptime\":\""   + String(uptime)                        + "\",";
  json += "\"firmware\":\"" FW_VERSION "\",";
  json += "\"device\":\""   + String(cfg.deviceName)               + "\"";
  json += "}";
  return json;
}

// GET /
void handleRoot()    { server.send_P(200, "text/html", DASHBOARD_HTML); }

// GET /settings
void handleSettings(){ server.send_P(200, "text/html", SETTINGS_HTML); }

// GET /status  – returns JSON
void handleStatus()  { server.send(200, "application/json", buildStatusJSON()); }

// GET /fan/on  /fan/off
void handleFanOn()   { updateRelay(true);  server.send(200, "application/json", buildStatusJSON()); }
void handleFanOff()  { updateRelay(false); server.send(200, "application/json", buildStatusJSON()); }

// GET /events  – Server-Sent Events
void handleEvents()
{
  sseClient = server.client();
  sseClient.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/event-stream\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: keep-alive\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "\r\n"
  );
  ssePush();
}

// GET /api/settings  – return current config (passwords masked)
void handleApiSettingsGet()
{
  // Passwords are always returned as "********" for security.
  // The settings page JS sends "********" back unchanged;
  // handleApiSettingsPost() skips overwriting those fields.
  String json = "{";
  json += "\"wifiSSID\":\""   + String(cfg.wifiSSID)   + "\",";
  json += "\"wifiPass\":\""   + String(MASK)            + "\",";
  json += "\"mqttServer\":\"" + String(cfg.mqttServer) + "\",";
  json += "\"mqttPort\":"     + String(cfg.mqttPort)   + ",";
  json += "\"mqttUser\":\""   + String(cfg.mqttUser)   + "\",";
  json += "\"mqttPass\":\""   + String(MASK)            + "\",";
  json += "\"mqttPrefix\":\"" + String(cfg.mqttPrefix) + "\",";
  json += "\"otaPass\":\""    + String(MASK)            + "\",";
  json += "\"deviceName\":\"" + String(cfg.deviceName) + "\",";
  json += "\"roomName\":\""   + String(cfg.roomName)   + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// ── Simple JSON value extractor (avoids heavy JSON library) ──
static String jsonStr(const String& body, const String& key)
{
  String search = "\"" + key + "\":\"";
  int start = body.indexOf(search);
  if (start < 0) return "";
  start += search.length();
  int end = body.indexOf("\"", start);
  if (end < 0) return "";
  return body.substring(start, end);
}

static int jsonInt(const String& body, const String& key)
{
  String search = "\"" + key + "\":";
  int start = body.indexOf(search);
  if (start < 0) return -1;
  start += search.length();
  int end = start;
  while (end < (int)body.length() && (isdigit(body[end]) || body[end]=='-')) end++;
  return body.substring(start, end).toInt();
}

// POST /api/settings  – save config, restart
void handleApiSettingsPost()
{
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"no body\"}"); return; }
  String body = server.arg("plain");

  // ── Validate required fields ─────────────────────────────
  String wifiSSID   = jsonStr(body, "wifiSSID");
  String mqttServer = jsonStr(body, "mqttServer");
  String deviceName = jsonStr(body, "deviceName");
  String mqttPrefix = jsonStr(body, "mqttPrefix");

  if (wifiSSID.isEmpty())   { server.send(400, "application/json", "{\"ok\":false,\"error\":\"wifiSSID required\"}");   return; }
  if (mqttServer.isEmpty()) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"mqttServer required\"}"); return; }
  if (deviceName.isEmpty()) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"deviceName required\"}"); return; }
  if (mqttPrefix.isEmpty()) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"mqttPrefix required\"}"); return; }

  // ── Apply non-password fields ────────────────────────────
  strlcpy(cfg.wifiSSID,   wifiSSID.c_str(),   sizeof(cfg.wifiSSID));
  strlcpy(cfg.mqttServer, mqttServer.c_str(), sizeof(cfg.mqttServer));
  strlcpy(cfg.mqttUser,   jsonStr(body,"mqttUser").c_str(),   sizeof(cfg.mqttUser));
  strlcpy(cfg.mqttPrefix, mqttPrefix.c_str(), sizeof(cfg.mqttPrefix));
  strlcpy(cfg.deviceName, deviceName.c_str(), sizeof(cfg.deviceName));
  strlcpy(cfg.roomName,   jsonStr(body,"roomName").c_str(),   sizeof(cfg.roomName));

  int port = jsonInt(body, "mqttPort");
  if (port > 0 && port <= 65535) cfg.mqttPort = (uint16_t)port;

  // ── Only overwrite passwords if not the mask sentinel ────
  String wifiPass = jsonStr(body, "wifiPass");
  if (wifiPass != MASK && !wifiPass.isEmpty())
    strlcpy(cfg.wifiPass, wifiPass.c_str(), sizeof(cfg.wifiPass));

  String mqttPass = jsonStr(body, "mqttPass");
  if (mqttPass != MASK && !mqttPass.isEmpty())
    strlcpy(cfg.mqttPass, mqttPass.c_str(), sizeof(cfg.mqttPass));

  String otaPass = jsonStr(body, "otaPass");
  if (otaPass != MASK && !otaPass.isEmpty())
    strlcpy(cfg.otaPass, otaPass.c_str(), sizeof(cfg.otaPass));

  saveConfig();

  server.send(200, "application/json", "{\"ok\":true}");
  Serial.println("Settings saved – restarting");
  delay(500);
  ESP.restart();
}

// POST /api/restart
void handleRestart()
{
  server.send(200, "application/json", "{\"ok\":true}");
  delay(300);
  ESP.restart();
}

// POST /api/reset
void handleReset()
{
  server.send(200, "application/json", "{\"ok\":true}");
  delay(300);
  factoryReset();   // wipes EEPROM and restarts
}

// ── AP mode handlers ──────────────────────────────────────────

void handleApRoot()  { server.send_P(200, "text/html", AP_SETUP_HTML); }

// POST /save  (AP mode – initial config form submit)
void handleApSave()
{
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"no body\"}"); return; }
  String body = server.arg("plain");

  String wifiSSID   = jsonStr(body, "wifiSSID");
  String mqttServer = jsonStr(body, "mqttServer");
  String deviceName = jsonStr(body, "deviceName");
  String mqttPrefix = jsonStr(body, "mqttPrefix");

  if (wifiSSID.isEmpty())   { server.send(400, "application/json", "{\"ok\":false,\"error\":\"wifiSSID required\"}");   return; }
  if (mqttServer.isEmpty()) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"mqttServer required\"}"); return; }
  if (deviceName.isEmpty()) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"deviceName required\"}"); return; }
  if (mqttPrefix.isEmpty()) { server.send(400, "application/json", "{\"ok\":false,\"error\":\"mqttPrefix required\"}"); return; }

  strlcpy(cfg.wifiSSID,   wifiSSID.c_str(),               sizeof(cfg.wifiSSID));
  strlcpy(cfg.wifiPass,   jsonStr(body,"wifiPass").c_str(),sizeof(cfg.wifiPass));
  strlcpy(cfg.mqttServer, mqttServer.c_str(),             sizeof(cfg.mqttServer));
  strlcpy(cfg.mqttUser,   jsonStr(body,"mqttUser").c_str(),sizeof(cfg.mqttUser));
  strlcpy(cfg.mqttPass,   jsonStr(body,"mqttPass").c_str(),sizeof(cfg.mqttPass));
  strlcpy(cfg.mqttPrefix, mqttPrefix.c_str(),             sizeof(cfg.mqttPrefix));
  strlcpy(cfg.otaPass,    jsonStr(body,"otaPass").c_str(), sizeof(cfg.otaPass));
  strlcpy(cfg.deviceName, deviceName.c_str(),             sizeof(cfg.deviceName));
  strlcpy(cfg.roomName,   jsonStr(body,"roomName").c_str(),sizeof(cfg.roomName));

  int port = jsonInt(body, "mqttPort");
  cfg.mqttPort = (port > 0 && port <= 65535) ? (uint16_t)port : 1883;

  saveConfig();
  server.send(200, "application/json", "{\"ok\":true}");
  Serial.println("Initial config saved – restarting");
  delay(1000);
  ESP.restart();
}


// ════════════════════════════════════════════════════════════
//  15.  setup()
// ════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  Serial.println("\n\nESP8266 Fan Controller  " FW_VERSION);

  // ── RELAY ──────────────────────────────────────────────────
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);  // OFF at boot (Active LOW)

  // ── FLASH/BOOT Button ──────────────────────────────────────
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  // ── EEPROM – load saved config ─────────────────────────────
  loadConfig();

  if (!configValid())
  {
    // ╔══════════════════════════════════════════════════╗
    // ║  AP SETUP MODE  –  no valid config in EEPROM    ║
    // ╚══════════════════════════════════════════════════╝
    Serial.println("No config found – starting AP setup mode");
    normalMode = false;

    WiFi.mode(WIFI_AP);
    char apSSID[32];
    snprintf(apSSID, sizeof(apSSID), "ESP8266-%04X", ESP.getChipId() & 0xFFFF);
    WiFi.softAP(apSSID, "12345678");
    Serial.print("AP SSID: ");
    Serial.println(apSSID);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/",     HTTP_GET,  handleApRoot);
    server.on("/save", HTTP_POST, handleApSave);
    server.begin();
    Serial.println("AP web server started");
    return;  // setup() exits; loop() handles AP clients only
  }

  // ╔══════════════════════════════════════════════════════╗
  // ║  NORMAL MODE  –  valid config loaded from EEPROM    ║
  // ╚══════════════════════════════════════════════════════╝
  normalMode = true;
  Serial.print("Device: "); Serial.println(cfg.deviceName);
  Serial.print("Room:   "); Serial.println(cfg.roomName);
  Serial.print("Prefix: "); Serial.println(cfg.mqttPrefix);

  // ── WIFI ───────────────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  WiFi.begin(cfg.wifiSSID, cfg.wifiPass);

  Serial.print("Connecting WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000)
  {
    delay(500); Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("WiFi connected: "); Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("WiFi failed – continuing anyway (MQTT will retry)");
  }

  // ── OTA ────────────────────────────────────────────────────
  ArduinoOTA.setHostname(cfg.deviceName);
  if (strlen(cfg.otaPass) > 0) ArduinoOTA.setPassword(cfg.otaPass);
  ArduinoOTA.setRebootOnSuccess(true);

  ArduinoOTA.onStart([]()   { Serial.println("OTA Start"); });
  ArduinoOTA.onEnd([]()     { Serial.println("OTA End"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%\n", (p*100)/t);
  });
  ArduinoOTA.onError([](ota_error_t err) {
    Serial.printf("OTA Error[%u]\n", err);
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  // ── mDNS ───────────────────────────────────────────────────
  if (MDNS.begin("fancontroller"))
  {
    Serial.println("mDNS responder started: http://fancontroller.local");
  }
  else
  {
    Serial.println("Error setting up MDNS responder!");
  }

  // ── WEB SERVER ─────────────────────────────────────────────
  server.on("/",            HTTP_GET,  handleRoot);
  server.on("/settings",    HTTP_GET,  handleSettings);
  server.on("/status",      HTTP_GET,  handleStatus);
  server.on("/fan/on",      HTTP_GET,  handleFanOn);
  server.on("/fan/off",     HTTP_GET,  handleFanOff);
  server.on("/events",      HTTP_GET,  handleEvents);
  server.on("/api/settings",HTTP_GET,  handleApiSettingsGet);
  server.on("/api/settings",HTTP_POST, handleApiSettingsPost);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/reset",   HTTP_POST, handleReset);
  server.begin();
  Serial.println("Web server started on port 80");

  // ── MQTT ───────────────────────────────────────────────────
  client.setServer(cfg.mqttServer, cfg.mqttPort);
  client.setBufferSize(2048);   // larger buffer for dynamic Discovery payloads
  client.setCallback(callback);
}


// ════════════════════════════════════════════════════════════
//  16.  loop()
// ════════════════════════════════════════════════════════════
void loop()
{
  // ── Physical button monitoring for Factory Reset ───────────
  static unsigned long buttonPressStart = 0;
  if (digitalRead(BOOT_BUTTON_PIN) == LOW)
  {
    if (buttonPressStart == 0)
    {
      buttonPressStart = millis();
      Serial.println("Reset button pressed. Hold for 10s to Factory Reset...");
    }
    else if (millis() - buttonPressStart >= 10000)
    {
      Serial.println("Button held for 10s! Performing factory reset...");
      factoryReset();
    }
  }
  else
  {
    if (buttonPressStart > 0)
    {
      Serial.println("Reset button released.");
      buttonPressStart = 0;
    }
  }

  if (!normalMode)
  {
    // AP setup mode – only serve the config web page
    server.handleClient();
    return;
  }

  // ── NORMAL MODE ────────────────────────────────────────────

  // MQTT (non-blocking reconnect)
  if (!client.connected()) mqttReconnect();
  client.loop();

  // OTA
  ArduinoOTA.handle();

  // Web server
  server.handleClient();

  // mDNS update
  MDNS.update();

  // Periodic diagnostics every 30 seconds
  static unsigned long lastDiag = 0;
  if (millis() - lastDiag > 30000)
  {
    lastDiag = millis();
    if (client.connected())
    {
      publishRSSI();
      publishHeap();
      publishIP();
      publishFirmware();
    }
  }
}
