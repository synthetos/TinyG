// TinyG HIL switchbox firmware: Feather HUZZAH ESP8266 platform layer around lib/sbcore.
//
// Everything runs in loop(): serial lines, HTTP (ESP8266WebServer) and WebSocket messages
// (WebSockets4WebServer on the same port) all go through sb::core().handle*, so commands are
// serialized without locks. Timed edges run from the timer1 interrupt (sb::Core::on_timer), so a
// slow HTTP client or a long serial reply never delays a pulse edge.
#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <LittleFS.h>
#include <WebSockets4WebServer.h>
#include <uri/UriBraces.h>

#include <sbcore.h>

#include "generated/assets.h"

using sb::Source;

namespace {

constexpr char HOSTNAME[] = "tinyg-switchbox";
constexpr char AP_PASSWORD[] = "tinyg-switchbox";   // documented default; provision STA for real use
constexpr char CONFIG_FILE[] = "/config.json";
constexpr char WIFI_FILE[] = "/wifi.json";
constexpr uint32_t STA_TIMEOUT_MS = 20000;
constexpr uint32_t POLL_MS = 10;            // line-level watch
constexpr uint32_t WS_PUMP_MS = 20;
constexpr int WS_BURST = 6;                 // events per client per pump
constexpr uint32_t WS_MAX_BACKLOG = 24;     // further behind than this: skip ahead, report the drop
constexpr uint32_t WS_KEEP = 4;             // events still sent after skipping ahead
constexpr uint32_t WS_MIN_HEAP = 8192;      // below this, WebSocket pushes wait (and later coalesce)
constexpr uint32_t WS_HISTORY = 16;         // recent events a new WebSocket client gets
constexpr int SERIAL_BURST = 2;             // subscribed events per loop pass (~13 ms each at 115200)
constexpr size_t LINE_MAX = 1024;

class Ws : public WebSockets4WebServer {
  public:
    bool connected(uint8_t num) { return num < WEBSOCKETS_SERVER_CLIENT_MAX && clientIsConnected(&_clients[num]); }
    // True if the client's TCP send buffer takes `len` bytes now, so sendTXT will not block.
    bool writable(uint8_t num, size_t len) {
        if (!connected(num) || !_clients[num].tcp) return false;
        return (size_t)_clients[num].tcp->availableForWrite() >= len + 16;
    }
};

ESP8266WebServer server(80);
Ws ws;
sb::Reply reply;

char line[LINE_MAX + 1];
size_t line_len = 0;
bool line_over = false;
bool serial_was_subscribed = false;
uint32_t serial_cursor = 0;

uint32_t ws_cursor[WEBSOCKETS_SERVER_CLIENT_MAX];
bool ws_live[WEBSOCKETS_SERVER_CLIENT_MAX];

struct WifiCreds {
    char ssid[33];
    char password[64];
};
WifiCreds creds;
bool have_creds = false;
sb::Wifi wifi_state = sb::Wifi::off;
uint32_t wifi_deadline = 0;
bool wifi_restart_pending = false;
uint32_t wifi_restart_at = 0;
bool mdns_started = false;
char ap_ssid[24];

uint32_t last_poll = 0, last_ws = 0;

// Buffers small writes to a WiFiClient (each write is otherwise its own TCP segment).
class BufferedClient : public Print {
  public:
    explicit BufferedClient(WiFiClient &c) : c_(c) {}
    size_t write(uint8_t b) override {
        buf_[n_++] = b;
        if (n_ == sizeof buf_) flush();
        return 1;
    }
    void flush() override {
        if (n_) c_.write(buf_, n_);
        n_ = 0;
    }

  private:
    WiFiClient &c_;
    uint8_t buf_[512];
    size_t n_ = 0;
};

void write_spec(Print &out) {
    char buf[256];
    for (size_t off = 0; off < SPEC_JSON_LEN;) {
        size_t n = SPEC_JSON_LEN - off < sizeof buf ? SPEC_JSON_LEN - off : sizeof buf;
        memcpy_P(buf, SPEC_JSON + off, n);
        out.write(reinterpret_cast<const uint8_t *>(buf), n);
        off += n;
        yield();                            // ~1.5 s at 115200: keep WiFi and the watchdog fed
    }
}

}  // namespace

// ---- sb::hal ----------------------------------------------------------------------------------

namespace sb {
namespace hal {

IRAM_ATTR uint64_t now_us() { return micros64(); }

// Open-drain contact: the output latch stays low and only the output enable changes, so the pin
// can sink the line or float, never source current. GPIO0-15 also have the pad's open-drain
// driver set (pin_setup), so even a stray latch-high write could not drive the line high.
IRAM_ATTR void pin_drive(uint8_t gpio, bool closed) {
    if (gpio < 16) {
        uint32_t bit = 1u << gpio;
        if (closed) {
            GPOC = bit;
            GPES = bit;
        } else {
            GPEC = bit;
        }
    } else if (gpio == 16) {
        if (closed) {
            GP16O &= ~1u;
            GP16E |= 1u;
        } else {
            GP16E &= ~1u;
        }
    }
}

IRAM_ATTR uint8_t pin_read(uint8_t gpio) {
    if (gpio < 16) return (GPI >> gpio) & 1u;
    if (gpio == 16) return GP16I & 1u;
    return 0;
}

IRAM_ATTR uint32_t irq_save() { return xt_rsil(15); }
IRAM_ATTR void irq_restore(uint32_t state) { xt_wsr_ps(state); }

IRAM_ATTR void timer_arm(uint64_t at_us) {
    if (!at_us) {
        timer1_disable();
        return;
    }
    uint64_t now = micros64();
    uint64_t d = at_us > now ? at_us - now : 1;
    if (d > 1600000) d = 1600000;           // timer1 at 5 MHz holds 1.67 s; re-evaluated on expiry
    uint32_t ticks = (uint32_t)d * 5;
    if (ticks < 10) ticks = 10;
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
    timer1_write(ticks);
}

IRAM_ATTR void spin_until(uint64_t at_us) {
    while (micros64() < at_us) {
    }
}

void pin_setup(uint8_t gpio) {
    pin_drive(gpio, false);
    pinMode(gpio, INPUT);                   // GPIO function, no pull-up, output disabled
    if (gpio < 16) {
        GPOC = 1u << gpio;
        GPC(gpio) |= (1u << GPCD);          // open-drain pad driver
    } else {
        GP16O &= ~1u;
    }
}

void pin_reset(uint8_t gpio) {
    pin_drive(gpio, false);
    pinMode(gpio, INPUT);
}

void info(JsonObject out) {
    char buf[12];
    out["api_version"] = SB_API_VERSION;
    out["spec_sha256"] = SB_SPEC_SHA256;
    out["heap_free"] = ESP.getFreeHeap();
    out["heap_max_block"] = ESP.getMaxFreeBlockSize();
    out["heap_fragmentation"] = ESP.getHeapFragmentation();
    out["cpu_mhz"] = ESP.getCpuFreqMHz();
    snprintf(buf, sizeof buf, "%06x", ESP.getChipId());
    out["chip_id"] = buf;
    out["core_version"] = ESP.getCoreVersion();
    out["sdk_version"] = ESP.getSdkVersion();
    out["mdns"] = "tinyg-switchbox.local";
    out["ws_clients"] = ws.connectedClients();
    JsonObject w = out["wifi"].to<JsonObject>();
    w["state"] = wifi_name(wifi_state);
    w["credentials_stored"] = have_creds;
    w["mac"] = WiFi.macAddress();
    if (wifi_state == Wifi::ap) {
        w["mode"] = "ap";
        w["ssid"] = (const char *)ap_ssid;
        w["ip"] = WiFi.softAPIP().toString();
        w["ap_clients"] = WiFi.softAPgetStationNum();
    } else if (WiFi.status() == WL_CONNECTED) {
        w["mode"] = "sta";
        w["ssid"] = WiFi.SSID();
        w["ip"] = WiFi.localIP().toString();
        w["rssi"] = WiFi.RSSI();
    } else {
        w["mode"] = wifi_state == Wifi::connecting ? "sta" : "off";
        w["ssid"] = have_creds ? (const char *)creds.ssid : nullptr;
        w["ip"] = nullptr;
    }
}

bool save_config(const Config &cfg) {
    JsonDocument doc;
    doc["version"] = 1;
    JsonArray arr = doc["channels"].to<JsonArray>();
    for (int i = 0; i < cfg.count; i++) {
        JsonObject c = arr.add<JsonObject>();
        c["name"] = (const char *)cfg.ch[i].name;
        c["gpio"] = cfg.ch[i].gpio;
        c["type"] = cfg.ch[i].type == Type::nc ? "nc" : "no";
    }
    File f = LittleFS.open("/config.tmp", "w");
    if (!f) return false;
    bool ok = serializeJson(doc, f) > 0;
    f.close();
    return ok && LittleFS.rename("/config.tmp", CONFIG_FILE);
}

bool save_wifi(const char *ssid, const char *password) {
    if (!*ssid) {
        LittleFS.remove(WIFI_FILE);
        have_creds = false;
    } else {
        JsonDocument doc;
        doc["ssid"] = ssid;
        doc["password"] = password;
        File f = LittleFS.open(WIFI_FILE, "w");
        if (!f) return false;
        bool ok = serializeJson(doc, f) > 0;
        f.close();
        if (!ok) return false;
        snprintf(creds.ssid, sizeof creds.ssid, "%s", ssid);
        snprintf(creds.password, sizeof creds.password, "%s", password);
        have_creds = true;
    }
    wifi_restart_pending = true;            // after the reply has gone out
    wifi_restart_at = millis() + 300;
    return true;
}

}  // namespace hal
}  // namespace sb

// ---- storage ----------------------------------------------------------------------------------

namespace {

IRAM_ATTR void timer_isr() { sb::core().on_timer(); }

// "stored", "default" (no file) or "invalid: <why>" (file ignored, defaults used).
String load_config(sb::Config &cfg) {
    sb::default_config(cfg);
    File f = LittleFS.open(CONFIG_FILE, "r");
    if (!f) return "default";
    JsonDocument doc;
    DeserializationError e = deserializeJson(doc, f);
    f.close();
    if (e) return String("invalid: ") + e.c_str();
    char why[160];
    sb::Config loaded;
    if (!sb::parse_config(doc, loaded, why, sizeof why)) return String("invalid: ") + why;
    cfg = loaded;
    return "stored";
}

void load_wifi() {
    have_creds = false;
    File f = LittleFS.open(WIFI_FILE, "r");
    if (!f) return;
    JsonDocument doc;
    DeserializationError e = deserializeJson(doc, f);
    f.close();
    const char *ssid = doc["ssid"] | "";
    if (e || !*ssid) return;
    snprintf(creds.ssid, sizeof creds.ssid, "%s", ssid);
    snprintf(creds.password, sizeof creds.password, "%s", doc["password"] | "");
    have_creds = true;
}

// ---- WiFi -------------------------------------------------------------------------------------

void set_wifi_state(sb::Wifi s) {
    if (s != wifi_state) sb::core().log_wifi(s);
    wifi_state = s;
}

void start_ap() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, AP_PASSWORD);
    set_wifi_state(sb::Wifi::ap);
    if (mdns_started) MDNS.notifyAPChange();
}

void wifi_start() {
    WiFi.persistent(false);                 // credentials live in LittleFS, not the SDK's flash area
    WiFi.hostname(HOSTNAME);
    WiFi.setAutoReconnect(true);
    if (!have_creds) {
        start_ap();
        return;
    }
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(creds.ssid, creds.password);
    wifi_deadline = millis() + STA_TIMEOUT_MS;
    set_wifi_state(sb::Wifi::connecting);
}

void wifi_loop() {
    if (wifi_restart_pending && (int32_t)(millis() - wifi_restart_at) >= 0) {
        wifi_restart_pending = false;
        wifi_start();
    }
    if (wifi_state == sb::Wifi::connecting) {
        if (WiFi.status() == WL_CONNECTED) {
            set_wifi_state(sb::Wifi::sta);
        } else if ((int32_t)(millis() - wifi_deadline) >= 0) {
            sb::core().log_wifi(sb::Wifi::sta_failed);
            start_ap();
        }
    }
    if (!mdns_started && (wifi_state == sb::Wifi::sta || wifi_state == sb::Wifi::ap)) {
        mdns_started = MDNS.begin(HOSTNAME);
        if (mdns_started) MDNS.addService("http", "tcp", 80);
    }
    if (mdns_started) MDNS.update();
}

// ---- serial -----------------------------------------------------------------------------------

void serial_reply(sb::Reply &r) {
    if (r.stream_spec) {
        Serial.print(F("{\"id\":"));
        serializeJson(r.doc["id"], Serial);
        Serial.print(F(",\"ok\":true,\"result\":"));
        write_spec(Serial);
        Serial.print(F("}\n"));
        return;
    }
    serializeJson(r.doc, Serial);
    Serial.write('\n');
}

void serial_line(const char *s, size_t n) {
    bool blank = true;
    for (size_t i = 0; i < n && blank; i++) blank = s[i] == ' ' || s[i] == '\t';
    if (blank) return;
    sb::core().handle_line(Source::serial, s, n, reply);
    serial_reply(reply);
    bool sub = sb::core().serial_subscribed();
    if (sub && !serial_was_subscribed) serial_cursor = reply.doc["result"]["next"] | sb::core().events().head();
    serial_was_subscribed = sub;
}

void serial_loop() {
    for (int budget = 256; budget > 0 && Serial.available(); budget--) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            bool over = line_over;
            size_t n = line_len;
            line_len = 0;
            line_over = false;
            if (over) {
                sb::core().line_too_long(Source::serial, reply);
                serial_reply(reply);
                return;
            }
            if (n) {
                serial_line(line, n);
                return;                     // one request per pass keeps the loop fair
            }
        } else if (line_len < LINE_MAX) {
            line[line_len++] = c;
        } else {
            line_over = true;
        }
    }
}

void serial_pump() {
    if (!sb::core().serial_subscribed()) return;
    const sb::EventRing &ring = sb::core().events();
    uint32_t oldest = ring.oldest(), head = ring.head();
    if (serial_cursor < oldest) {
        Serial.printf("{\"event\":\"dropped\",\"count\":%lu,\"next\":%lu}\n",
                      (unsigned long)(oldest - serial_cursor), (unsigned long)oldest);
        serial_cursor = oldest;
    }
    sb::Event e;
    for (int k = 0; k < SERIAL_BURST && serial_cursor < head; k++, serial_cursor++) {
        if (!ring.get(serial_cursor, e)) continue;
        JsonDocument doc;
        sb::core().event_json(e, doc.to<JsonObject>());
        serializeJson(doc, Serial);
        Serial.write('\n');
    }
}

// ---- HTTP -------------------------------------------------------------------------------------

void http_json(int status, JsonVariantConst body) {
    server.setContentLength(measureJson(body));
    server.send(status, "application/json", "");
    BufferedClient out(server.client());
    serializeJson(body, out);
    out.flush();
}

void send_spec() { server.send_P(200, "application/json", SPEC_JSON, SPEC_JSON_LEN); }
void send_ui() { server.send_P(200, "text/html; charset=utf-8", UI_HTML, UI_HTML_LEN); }

void http_reply(sb::Reply &r) {
    if (r.stream_spec) {
        send_spec();
        return;
    }
    if (r.err == sb::Err::none) {
        http_json(200, r.doc["result"]);
        return;
    }
    JsonDocument e;
    e["error"] = r.doc["error"];
    http_json(sb::http_status(r.err), e);
}

// A REST route: the body (JSON object), the path's {name} and query parameters become the op's
// parameters, then it runs through the same executor as serial and WebSocket requests.
void http_op(const char *op, bool with_name) {
    JsonDocument req;
    const String &body = server.arg("plain");
    if (body.length()) {
        if (deserializeJson(req, body)) {
            sb::core().handle_line(Source::http, body.c_str(), body.length(), reply);   // bad_json reply
            http_reply(reply);
            return;
        }
    } else {
        req.to<JsonObject>();
    }
    if (req.is<JsonObject>()) {
        req["op"] = op;
        if (with_name) req["name"] = server.pathArg(0);
        for (int i = 0; i < server.args(); i++) {
            const String &k = server.argName(i);
            if (k == "plain") continue;
            const String &v = server.arg(i);
            bool digits = v.length() > 0 && v.length() <= 10;
            for (size_t j = 0; digits && j < v.length(); j++) digits = isdigit((unsigned char)v[j]);
            if (digits) req[k] = strtoull(v.c_str(), nullptr, 10);
            else req[k] = v;
        }
    }
    sb::core().handle(Source::http, req, reply);
    http_reply(reply);
}

void not_found() {
    if (server.method() == HTTP_OPTIONS) {              // CORS preflight
        server.sendHeader("Access-Control-Allow-Methods", "GET, PUT, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        server.send(204);
        return;
    }
    JsonDocument e;
    e["error"]["code"] = "not_found";
    e["error"]["msg"] = "no such route; see /openapi.json";
    http_json(404, e);
}

void http_setup() {
    server.enableCORS(true);
    server.on("/", HTTP_GET, send_ui);
    server.on("/openapi.json", HTTP_GET, send_spec);
    server.on("/api/info", HTTP_GET, [] { http_op("getInfo", false); });
    server.on("/api/state", HTTP_GET, [] { http_op("getState", false); });
    server.on(UriBraces("/api/channels/{}/pulse"), HTTP_POST, [] { http_op("pulseChannel", true); });
    server.on(UriBraces("/api/channels/{}/bounce"), HTTP_POST, [] { http_op("bounceChannel", true); });
    server.on(UriBraces("/api/channels/{}"), HTTP_PUT, [] { http_op("setChannel", true); });
    server.on("/api/release_all", HTTP_POST, [] { http_op("releaseAll", false); });
    server.on("/api/idle_all", HTTP_POST, [] { http_op("idleAll", false); });
    server.on("/api/config", HTTP_GET, [] { http_op("getConfig", false); });
    server.on("/api/config", HTTP_PUT, [] { http_op("setConfig", false); });
    server.on("/api/events", HTTP_GET, [] { http_op("getEvents", false); });
    server.on("/api/wifi", HTTP_PUT, [] { http_op("setWifi", false); });
    server.onNotFound(not_found);
}

// ---- WebSocket --------------------------------------------------------------------------------

bool ws_send(uint8_t num, JsonVariantConst doc, bool must) {
    size_t n = measureJson(doc);
    if (!must && !ws.writable(num, n)) return false;
    char *buf = static_cast<char *>(malloc(n + 1));
    if (!buf) return false;
    serializeJson(doc, buf, n + 1);
    bool ok = ws.sendTXT(num, buf, n);
    free(buf);
    return ok;
}

void ws_state(uint8_t num, const char *event, bool full) {
    JsonDocument doc;
    doc["event"] = event;
    if (full) {
        sb::core().info_json(doc["info"].to<JsonObject>());
        sb::core().config_json(doc["config"].to<JsonObject>());
    }
    sb::core().state_json(doc["state"].to<JsonObject>());
    ws_send(num, doc, full);
}

void ws_event(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
    if (num >= WEBSOCKETS_SERVER_CLIENT_MAX) return;
    switch (type) {
    case WStype_CONNECTED: {
        const sb::EventRing &ring = sb::core().events();
        uint32_t head = ring.head(), from = head > WS_HISTORY ? head - WS_HISTORY : 1;
        ws_cursor[num] = from > ring.oldest() ? from : ring.oldest();
        ws_live[num] = true;
        ws_state(num, "snapshot", true);
        break;
    }
    case WStype_DISCONNECTED:
        ws_live[num] = false;
        break;
    case WStype_TEXT:
        sb::core().handle_line(Source::ws, reinterpret_cast<const char *>(payload), len, reply);
        ws_send(num, reply.doc, true);
        break;
    default:
        break;
    }
}

// Pushes new events to each client without ever blocking on a slow one: a client whose TCP
// buffer is full is skipped; one that falls too far behind skips ahead and is told how many
// events it missed (getEvents still has them while they are in the ring).
void ws_pump() {
    if (ESP.getFreeHeap() < WS_MIN_HEAP) return;
    const sb::EventRing &ring = sb::core().events();
    for (uint8_t num = 0; num < WEBSOCKETS_SERVER_CLIENT_MAX; num++) {
        if (!ws_live[num] || !ws.connected(num)) continue;
        uint32_t oldest = ring.oldest(), head = ring.head(), c = ws_cursor[num];
        if (c < oldest || head - c > WS_MAX_BACKLOG) {
            uint32_t to = head - WS_KEEP > oldest ? head - WS_KEEP : oldest;
            JsonDocument d;
            d["event"] = "dropped";
            d["count"] = to - c;
            d["next"] = to;
            if (!ws_send(num, d, false)) continue;
            ws_state(num, "state", false);
            c = to;
        }
        sb::Event e;
        for (int k = 0; k < WS_BURST && c < head; k++) {
            if (!ring.get(c, e)) {
                c++;
                continue;
            }
            JsonDocument d;
            sb::core().event_json(e, d.to<JsonObject>());
            if (!ws_send(num, d, false)) break;
            c++;
        }
        ws_cursor[num] = c;
    }
}

void print_boot_line(const String &config_source, bool fs_ok) {
    sb::Event e;
    JsonDocument doc;
    if (sb::core().events().get(1, e)) sb::core().event_json(e, doc.to<JsonObject>());
    doc["api_version"] = SB_API_VERSION;
    doc["config"] = config_source;
    doc["fs"] = fs_ok;
    Serial.write('\n');                     // end whatever the ROM printed at 74880 baud
    serializeJson(doc, Serial);
    Serial.write('\n');
}

}  // namespace

void setup() {
    Serial.setRxBufferSize(2048);
    Serial.begin(115200);
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(ap_ssid, sizeof ap_ssid, "tinyg-switchbox-%02X%02X", mac[4], mac[5]);

    sb::Config cfg;
    bool fs_ok = LittleFS.begin();
    String config_source = fs_ok ? load_config(cfg) : String("default");
    if (!fs_ok) sb::default_config(cfg);
    sb::core().begin(cfg, ESP.getResetReason().c_str(), RANDOM_REG32);   // all channels released
    timer1_isr_init();
    timer1_attachInterrupt(timer_isr);
    print_boot_line(config_source, fs_ok);

    if (fs_ok) load_wifi();
    wifi_start();
    http_setup();
    server.addHook(ws.hookForWebserver("/ws", ws_event));
    ws.enableHeartbeat(15000, 3000, 2);
    server.begin();
}

void loop() {
    serial_loop();
    serial_pump();
    server.handleClient();
    ws.loop();
    uint32_t ms = millis();
    if (ms - last_poll >= POLL_MS) {
        last_poll = ms;
        sb::core().poll_lines();
    }
    if (ms - last_ws >= WS_PUMP_MS) {
        last_ws = ms;
        ws_pump();
    }
    wifi_loop();
}
