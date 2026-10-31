// Switchbox core; see sbcore.h for the model and the concurrency rules.
//
// ArduinoJson note: string literals are stored by pointer, every other string is copied. Member
// char arrays are cast to const char* before they go into a document, because a const char[N]
// would be taken for a literal of length N-1.
#include "sbcore.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace sb {

#define TRY(expr)                                   \
    do {                                            \
        Err e_ = (expr);                            \
        if (e_ != Err::none) return e_;             \
    } while (0)

// ---- names ----------------------------------------------------------------------------------

static const char *const OP_NAMES[] = {
    "getInfo", "getState", "setChannel", "pulseChannel", "bounceChannel", "releaseAll", "idleAll",
    "getConfig", "setConfig", "getEvents", "setWifi", "scanWifi", "getSpec", "subscribe", "invalid"};
static const char *const ERR_NAMES[] = {
    "none", "bad_json", "bad_request", "unknown_op", "unknown_param", "missing_param", "bad_param",
    "conflicting_params", "unknown_channel", "no_change", "busy", "invalid_config", "line_too_long",
    "unsupported_transport", "storage"};
static const char *const SOURCE_NAMES[] = {"boot", "serial", "http", "ws", "ttl", "poll", "sys"};
static const char *const KIND_NAMES[] = {"boot", "command", "edge", "line", "config", "wifi"};
static const char *const CAUSE_NAMES[] = {"set", "pulse", "bounce", "ttl", "release_all", "idle_all", "config"};
static const char *const WIFI_NAMES[] = {"off", "connecting", "sta", "ap", "sta_failed"};

#define COUNT_OF(a) (sizeof(a) / sizeof((a)[0]))
static_assert(COUNT_OF(OP_NAMES) == (size_t)Op::COUNT, "op names");
static_assert(COUNT_OF(ERR_NAMES) == (size_t)Err::COUNT, "error names");
static_assert(COUNT_OF(SOURCE_NAMES) == (size_t)Source::COUNT, "source names");
static_assert(COUNT_OF(KIND_NAMES) == (size_t)Kind::COUNT, "kind names");
static_assert(COUNT_OF(CAUSE_NAMES) == (size_t)Cause::COUNT, "cause names");
static_assert(COUNT_OF(WIFI_NAMES) == (size_t)Wifi::COUNT, "wifi names");
static_assert((EVENT_RING & (EVENT_RING - 1)) == 0, "EVENT_RING must be a power of two");
static_assert(MAX_CHANNELS <= 8, "levels/driven are 8-bit masks");

static const char *name_of(const char *const *table, size_t n, unsigned i) { return i < n ? table[i] : "?"; }
const char *op_name(Op op) { return name_of(OP_NAMES, COUNT_OF(OP_NAMES), (unsigned)op); }
const char *err_name(Err e) { return name_of(ERR_NAMES, COUNT_OF(ERR_NAMES), (unsigned)e); }
const char *source_name(Source s) { return name_of(SOURCE_NAMES, COUNT_OF(SOURCE_NAMES), (unsigned)s); }
const char *kind_name(Kind k) { return name_of(KIND_NAMES, COUNT_OF(KIND_NAMES), (unsigned)k); }
const char *cause_name(Cause c) { return name_of(CAUSE_NAMES, COUNT_OF(CAUSE_NAMES), (unsigned)c); }
const char *wifi_name(Wifi w) { return name_of(WIFI_NAMES, COUNT_OF(WIFI_NAMES), (unsigned)w); }

Op op_from_name(const char *name) {
    for (unsigned i = 0; i < (unsigned)Op::invalid; i++)
        if (name && strcmp(name, OP_NAMES[i]) == 0) return (Op)i;
    return Op::invalid;
}

int http_status(Err e) {
    switch (e) {
    case Err::none: return 200;
    case Err::unknown_op:
    case Err::unknown_channel: return 404;
    case Err::busy:
    case Err::no_change: return 409;
    case Err::storage: return 500;
    default: return 400;
    }
}

// ---- configuration --------------------------------------------------------------------------

void default_config(Config &c) {
    // Safe pins first; GPIO0/GPIO2 last because they are boot-strap pins (see gpio_caveat).
    static const struct { const char *name; uint8_t gpio; } map[] = {
        {"xmin", 4}, {"xmax", 5}, {"ymin", 12}, {"ymax", 13},
        {"zmin", 14}, {"zmax", 16}, {"amin", 0}, {"amax", 2}};
    memset(&c, 0, sizeof c);
    c.count = COUNT_OF(map);
    for (unsigned i = 0; i < COUNT_OF(map); i++) {
        snprintf(c.ch[i].name, sizeof c.ch[i].name, "%s", map[i].name);
        c.ch[i].gpio = map[i].gpio;
        c.ch[i].type = Type::no;
    }
}

const char *gpio_forbidden(long gpio) {
    if (gpio == 1 || gpio == 3) return "is the USB serial port (TX/RX)";
    if (gpio >= 6 && gpio <= 11) return "is the ESP-12 module's SPI flash (no pad on the Feather)";
    if (gpio == 15) return "has the Feather's 4.7k boot-strap pull-down: TinyG's input would never read open, and its pull-up can stop the boot";
    if (gpio < 0 || gpio > 16) return "is not an ESP8266 GPIO (A0 is analog-only)";
    return nullptr;
}

const char *gpio_caveat(long gpio) {
    if (gpio == 0) return "boot-strap pin (must be high at boot) with the red LED; a closed contact or real switch during ESP reset stops it booting, and the USB-serial auto-reset circuit can pull it low";
    if (gpio == 2) return "boot-strap pin (must be high at boot) with a 10k pull-up and the module's blue LED; a closed contact or real switch during ESP reset stops it booting";
    return nullptr;
}

static bool valid_name(const char *s) {
    size_t n = strlen(s);
    if (n < 1 || n > (size_t)NAME_LEN) return false;
    for (size_t i = 0; i < n; i++) {
        char c = s[i];
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                  c == '_' || c == '-' || c == '.';
        if (!ok) return false;
    }
    return true;
}

bool parse_config(JsonVariantConst v, Config &out, char *why, size_t why_len) {
    memset(&out, 0, sizeof out);
    JsonVariantConst list = v["channels"];
    if (!list.is<JsonArrayConst>()) {
        snprintf(why, why_len, "channels must be an array");
        return false;
    }
    JsonArrayConst arr = list.as<JsonArrayConst>();
    if (arr.size() < 1 || arr.size() > (size_t)MAX_CHANNELS) {
        snprintf(why, why_len, "channels must hold 1 to %d entries", MAX_CHANNELS);
        return false;
    }
    int i = 0;
    for (JsonVariantConst item : arr) {
        if (!item.is<JsonObjectConst>()) {
            snprintf(why, why_len, "channels[%d] must be an object", i);
            return false;
        }
        for (JsonPairConst kv : item.as<JsonObjectConst>()) {
            const char *k = kv.key().c_str();
            if (strcmp(k, "name") && strcmp(k, "gpio") && strcmp(k, "type") && strcmp(k, "caveat")) {
                snprintf(why, why_len, "channels[%d]: unknown key '%.16s'", i, k);
                return false;
            }
        }
        JsonVariantConst name = item["name"], gpio = item["gpio"], type = item["type"];
        if (!name.is<const char *>() || !valid_name(name.as<const char *>())) {
            snprintf(why, why_len, "channels[%d]: name must be 1-%d characters of A-Z a-z 0-9 _ - .", i, NAME_LEN);
            return false;
        }
        if (!gpio.is<long>()) {
            snprintf(why, why_len, "channels[%d] (%s): gpio must be an integer", i, name.as<const char *>());
            return false;
        }
        long g = gpio.as<long>();
        if (const char *bad = gpio_forbidden(g)) {
            snprintf(why, why_len, "channels[%d] (%s): GPIO%ld %s", i, name.as<const char *>(), g, bad);
            return false;
        }
        Type t = Type::no;
        if (!type.isNull()) {
            const char *ts = type.is<const char *>() ? type.as<const char *>() : "";
            if (strcmp(ts, "no") == 0) t = Type::no;
            else if (strcmp(ts, "nc") == 0) t = Type::nc;
            else {
                snprintf(why, why_len, "channels[%d] (%s): type must be \"no\" or \"nc\"", i, name.as<const char *>());
                return false;
            }
        }
        for (int j = 0; j < i; j++) {
            if (strcmp(out.ch[j].name, name.as<const char *>()) == 0) {
                snprintf(why, why_len, "channels[%d]: duplicate name '%s'", i, name.as<const char *>());
                return false;
            }
            if (out.ch[j].gpio == g) {
                snprintf(why, why_len, "channels[%d] (%s): GPIO%ld already used by %s", i, name.as<const char *>(), g, out.ch[j].name);
                return false;
            }
        }
        snprintf(out.ch[i].name, sizeof out.ch[i].name, "%s", name.as<const char *>());
        out.ch[i].gpio = (uint8_t)g;
        out.ch[i].type = t;
        i++;
    }
    out.count = (uint8_t)i;
    return true;
}

// ---- event ring -----------------------------------------------------------------------------

SB_IRAM uint32_t EventRing::push(uint8_t kind, uint8_t src, uint8_t code, int8_t ch, uint8_t value,
                                 uint8_t err, uint32_t rid, uint32_t arg, uint64_t t_us,
                                 uint8_t levels, uint8_t driven) {
    uint32_t s = hal::irq_save();
    uint32_t seq = next_++;
    Event &e = buf_[seq & (EVENT_RING - 1)];   // fields one by one: no memcpy in the interrupt
    e.t_us = t_us;
    e.seq = seq;
    e.rid = rid;
    e.arg = arg;
    e.kind = kind;
    e.src = src;
    e.code = code;
    e.ch = ch;
    e.value = value;
    e.err = err;
    e.levels = levels;
    e.driven = driven;
    hal::irq_restore(s);
    return seq;
}

bool EventRing::get(uint32_t seq, Event &out) const {
    uint32_t s = hal::irq_save();
    bool ok = seq >= oldest() && seq < next_;
    if (ok) out = buf_[seq & (EVENT_RING - 1)];
    hal::irq_restore(s);
    return ok;
}

uint32_t EventRing::head() const { return next_; }
uint32_t EventRing::oldest() const { return next_ > (uint32_t)EVENT_RING ? next_ - EVENT_RING : 1; }

void EventRing::clear() {
    uint32_t s = hal::irq_save();
    memset(buf_, 0, sizeof buf_);
    next_ = 1;
    hal::irq_restore(s);
}

// ---- core: channel state and the edge sequencer ---------------------------------------------

static Core g_core;
SB_IRAM Core &core() { return g_core; }   // reached from the timer interrupt

void Core::begin(const Config &cfg, const char *reset_reason, uint32_t boot_id) {
    cfg_ = cfg;
    memset(rt_, 0, sizeof rt_);
    ring_.clear();
    serial_subscribed_ = false;
    boot_id_ = boot_id;
    snprintf(reset_reason_, sizeof reset_reason_, "%s", reset_reason ? reset_reason : "");
    for (int i = 0; i < cfg_.count; i++) {
        hal::pin_setup(cfg_.ch[i].gpio);     // every channel starts released: open, nothing driven
        rt_[i].last_level = hal::pin_read(cfg_.ch[i].gpio);
    }
    ring_.push((uint8_t)Kind::boot, (uint8_t)Source::boot, 0, -1, 1, 0, 0, 0, hal::now_us(), levels(), 0);
}

SB_IRAM uint8_t Core::levels() const {
    uint8_t m = 0;
    for (int i = 0; i < cfg_.count; i++)
        if (hal::pin_read(cfg_.ch[i].gpio)) m |= (uint8_t)(1u << i);
    return m;
}

SB_IRAM uint8_t Core::driven() const {
    uint8_t m = 0;
    for (int i = 0; i < cfg_.count; i++)
        if (rt_[i].closed) m |= (uint8_t)(1u << i);
    return m;
}

// The only place a pin changes. Callers hold the interrupt lock (or are the interrupt).
SB_IRAM void Core::edge(int i, bool closed, Source src, Cause cause, uint32_t rid, uint64_t now) {
    hal::pin_drive(cfg_.ch[i].gpio, closed);
    rt_[i].closed = closed;
    rt_[i].since_us = now;
    ring_.push((uint8_t)Kind::edge, (uint8_t)src, (uint8_t)cause, (int8_t)i, closed, 0, rid, 0, now,
               levels(), driven());
}

// Applies every timed action that is due; returns the next deadline (0 = nothing pending).
SB_IRAM uint64_t Core::service(uint64_t now) {
    uint64_t next = 0;
    for (int i = 0; i < cfg_.count; i++) {
        Runtime &r = rt_[i];
        if (r.ttl_at_us && r.ttl_at_us <= now) {        // ttl is a hard limit: it also ends a sequence
            r.ttl_at_us = 0;
            r.seq_kind = 0;
            if (r.closed) edge(i, false, Source::ttl, Cause::ttl, r.ttl_rid, now);
        }
        if (r.seq_kind && r.seq_next_us <= now) {
            if (r.seq_kind == SEQ_PULSE) {
                edge(i, r.final_closed, (Source)r.src, Cause::pulse, r.rid, now);
                r.seq_kind = 0;
            } else if (r.toggles_left) {
                edge(i, !r.closed, (Source)r.src, Cause::bounce, r.rid, now);
                r.toggles_left--;
                r.seq_next_us += r.interval_us;
            } else {
                if (r.closed != r.final_closed) edge(i, r.final_closed, (Source)r.src, Cause::bounce, r.rid, now);
                r.seq_kind = 0;
            }
        }
        if (r.ttl_at_us && (!next || r.ttl_at_us < next)) next = r.ttl_at_us;
        if (r.seq_kind && (!next || r.seq_next_us < next)) next = r.seq_next_us;
    }
    return next;
}

// Runs due actions, spins through edges closer than INLINE_US, then arms the timer for the rest.
// Interrupts are masked by the caller (loop critical section or the timer interrupt itself).
SB_IRAM void Core::service_and_rearm() {
    for (int guard = 0; guard < 4096; guard++) {
        uint64_t now = hal::now_us();
        uint64_t next = service(now);
        if (!next) {
            hal::timer_arm(0);
            return;
        }
        if (next > now + INLINE_US) {
            hal::timer_arm(next);
            return;
        }
        hal::spin_until(next);
    }
    hal::timer_arm(hal::now_us() + INLINE_US);
}

SB_IRAM void Core::on_timer() { service_and_rearm(); }

int Core::find(const char *name) const {
    for (int i = 0; i < cfg_.count; i++)
        if (name && strcmp(cfg_.ch[i].name, name) == 0) return i;
    return -1;
}

const char *Core::warning(uint8_t level, const Runtime &r) const {
    uint64_t age = hal::now_us() - r.since_us;
    if (r.closed && level) return "contact closed but the line reads high: something else drives it high, or the pin or wire is faulty";
    if (!r.closed && r.flaps + r.flaps_prev >= 4) return "open line toggles on its own: input floating (TinyG unpowered or wire disconnected)";
    if (!r.closed && !level && age > 1000) return "open but the line reads low: a real switch or another device holds it low, a wiring fault, or TinyG is unpowered/disconnected";
    return nullptr;
}

void Core::poll_lines() {
    uint64_t now = hal::now_us();
    for (int i = 0; i < cfg_.count; i++) {
        Runtime &r = rt_[i];
        uint8_t level = hal::pin_read(cfg_.ch[i].gpio);
        uint32_t s = hal::irq_save();
        bool closed = r.closed;
        uint64_t since = r.since_us;
        hal::irq_restore(s);
        if (now - r.flap_window_us >= 1000000) {
            r.flaps_prev = r.flaps;
            r.flaps = 0;
            r.flap_window_us = now;
        }
        if (level == r.last_level) continue;
        r.last_level = level;
        bool ours = since && now - since < 20000;       // the line following our own edge
        bool floating = r.flaps + r.flaps_prev >= 4;    // toggling by itself: stop logging it
        if (!closed && !ours && r.flaps < 255) r.flaps++;
        if ((!closed && !floating) || (closed && level))  // released line moved, or a driven line is high
            ring_.push((uint8_t)Kind::line, (uint8_t)Source::poll, 0, (int8_t)i, level, 0, 0, 0, now,
                       levels(), driven());
    }
}

void Core::log_wifi(Wifi state) {
    ring_.push((uint8_t)Kind::wifi, (uint8_t)Source::sys, 0, -1, 1, 0, 0, (uint32_t)state, hal::now_us(),
               levels(), driven());
}

void Core::log_command(Source src, Op op, int ch, Err err, uint32_t rid, uint32_t arg) {
    ring_.push((uint8_t)Kind::command, (uint8_t)src, (uint8_t)op, (int8_t)ch, err == Err::none, (uint8_t)err,
               rid, arg, hal::now_us(), levels(), driven());
}

void Core::apply_config(const Config &next, Source src, uint32_t rid) {
    uint32_t s = hal::irq_save();
    uint64_t now = hal::now_us();
    for (int i = 0; i < cfg_.count; i++) {
        rt_[i].seq_kind = 0;
        rt_[i].ttl_at_us = 0;
        if (rt_[i].closed) edge(i, false, src, Cause::config, rid, now);
    }
    service_and_rearm();                                // nothing pending: cancels the timer
    hal::irq_restore(s);
    for (int i = 0; i < cfg_.count; i++) {
        bool kept = false;
        for (int j = 0; j < next.count; j++) kept = kept || next.ch[j].gpio == cfg_.ch[i].gpio;
        if (!kept) hal::pin_reset(cfg_.ch[i].gpio);
    }
    s = hal::irq_save();
    cfg_ = next;
    memset(rt_, 0, sizeof rt_);
    hal::irq_restore(s);
    for (int i = 0; i < cfg_.count; i++) {
        hal::pin_setup(cfg_.ch[i].gpio);
        rt_[i].last_level = hal::pin_read(cfg_.ch[i].gpio);
        rt_[i].since_us = now;
    }
    ring_.push((uint8_t)Kind::config, (uint8_t)src, 0, -1, 1, 0, rid, cfg_.count, hal::now_us(), levels(), 0);
}

// ---- JSON views -----------------------------------------------------------------------------

static void mask_string(uint8_t mask, int count, char *buf) {
    for (int i = 0; i < count; i++) buf[i] = (mask >> i) & 1 ? '1' : '0';
    buf[count] = 0;
}

void Core::event_json(const Event &e, JsonObject out) const {
    char buf[MAX_CHANNELS + 1];
    out["event"] = kind_name((Kind)e.kind);
    out["seq"] = e.seq;
    out["t_us"] = e.t_us;
    out["src"] = source_name((Source)e.src);
    if (e.rid) out["rid"] = e.rid;
    if (e.ch >= 0 && e.ch < cfg_.count) out["ch"] = (const char *)cfg_.ch[e.ch].name;
    switch ((Kind)e.kind) {
    case Kind::command:
        out["op"] = op_name((Op)e.code);
        out["ok"] = e.value != 0;
        if (!e.value) out["error"] = err_name((Err)e.err);
        if (e.arg) out["arg"] = e.arg;
        break;
    case Kind::edge:
        out["state"] = e.value ? "closed" : "open";
        out["cause"] = cause_name((Cause)e.code);
        break;
    case Kind::line:
        out["level"] = e.value;
        if (e.ch >= 0) out["state"] = (e.driven >> e.ch) & 1 ? "closed" : "open";
        break;
    case Kind::boot:
        out["fw"] = FW_VERSION;
        out["reason"] = (const char *)reset_reason_;
        snprintf(buf, sizeof buf, "%08lx", (unsigned long)boot_id_);
        out["boot_id"] = buf;
        break;
    case Kind::config:
        out["channels"] = e.arg;
        break;
    case Kind::wifi:
        out["state"] = wifi_name((Wifi)e.arg);
        break;
    default:
        break;
    }
    mask_string(e.levels, cfg_.count, buf);
    out["levels"] = buf;
    mask_string(e.driven, cfg_.count, buf);
    out["driven"] = buf;
}

void Core::channel_json(int i, JsonObject out) {
    uint32_t s = hal::irq_save();
    Runtime r = rt_[i];
    hal::irq_restore(s);
    const ChannelConfig &c = cfg_.ch[i];
    uint8_t level = hal::pin_read(c.gpio);
    out["name"] = (const char *)c.name;
    out["gpio"] = c.gpio;
    out["type"] = c.type == Type::nc ? "nc" : "no";
    out["state"] = r.closed ? "closed" : "open";
    out["tripped"] = c.type == Type::nc ? !r.closed : (bool)r.closed;
    out["level"] = level;
    out["since_us"] = r.since_us;
    if (r.seq_kind) {
        JsonObject p = out["pending"].to<JsonObject>();
        p["kind"] = r.seq_kind == SEQ_PULSE ? "pulse" : "bounce";
        p["until_us"] = r.seq_next_us + (uint64_t)r.toggles_left * r.interval_us;
    } else {
        out["pending"] = nullptr;
    }
    if (r.ttl_at_us) out["ttl_until_us"] = r.ttl_at_us;
    else out["ttl_until_us"] = nullptr;
    const char *w = warning(level, r);
    if (w) out["warning"] = w;
    else out["warning"] = nullptr;
}

void Core::state_json(JsonObject out) {
    char buf[MAX_CHANNELS + 1];
    out["t_us"] = hal::now_us();
    mask_string(levels(), cfg_.count, buf);
    out["levels"] = buf;
    mask_string(driven(), cfg_.count, buf);
    out["driven"] = buf;
    JsonArray arr = out["channels"].to<JsonArray>();
    for (int i = 0; i < cfg_.count; i++) channel_json(i, arr.add<JsonObject>());
}

void Core::config_json(JsonObject out) const {
    Config d;
    default_config(d);
    JsonArray arr = out["channels"].to<JsonArray>();
    for (int i = 0; i < cfg_.count; i++) {
        JsonObject c = arr.add<JsonObject>();
        c["name"] = (const char *)cfg_.ch[i].name;
        c["gpio"] = cfg_.ch[i].gpio;
        c["type"] = cfg_.ch[i].type == Type::nc ? "nc" : "no";
        if (const char *caveat = gpio_caveat(cfg_.ch[i].gpio)) c["caveat"] = caveat;
    }
    out["is_default"] = memcmp(&d, &cfg_, sizeof d) == 0;
}

// ---- request parsing helpers ----------------------------------------------------------------

Err Core::fail(Err e, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg_, sizeof msg_, fmt, ap);
    va_end(ap);
    return e;
}

static bool present(JsonVariantConst q, const char *key) { return !q[key].isNull(); }

static void note_clamped(JsonObject res, const char *key) {
    JsonVariant c = res["clamped"];
    JsonArray arr = c.is<JsonArray>() ? c.as<JsonArray>() : res["clamped"].to<JsonArray>();
    arr.add(key);
}

namespace {
// Parameter reader bound to one request; errors go through Core::fail via the callback.
struct Params {
    JsonVariantConst q;
    JsonObject res;
    char *msg;
    size_t msg_len;

    Err failf(Err e, const char *fmt, const char *a, const char *b = "") {
        snprintf(msg, msg_len, fmt, a, b);
        return e;
    }

    Err keys(const char *const *allowed) {
        for (JsonPairConst kv : q.as<JsonObjectConst>()) {
            const char *k = kv.key().c_str();
            if (!strcmp(k, "id") || !strcmp(k, "op")) continue;
            bool ok = false;
            for (const char *const *p = allowed; *p && !ok; p++) ok = !strcmp(k, *p);
            if (!ok) return failf(Err::unknown_param, "unknown parameter '%.24s'", k);
        }
        return Err::none;
    }

    // Unsigned integer in [lo, hi]; larger values (and 0 when lo > 0) are clamped and reported.
    Err uint(const char *key, uint32_t lo, uint32_t hi, uint32_t &out) {
        JsonVariantConst v = q[key];
        if (v.isNull()) return failf(Err::missing_param, "missing parameter '%s'", key);
        if (v.is<bool>() || !v.is<long long>()) return failf(Err::bad_param, "'%s' must be an integer", key);
        long long n = v.as<long long>();
        if (n < 0) return failf(Err::bad_param, "'%s' must not be negative", key);
        uint64_t u = (uint64_t)n;
        if (u < lo || u > hi) {
            u = u < lo ? lo : hi;
            note_clamped(res, key);
        }
        out = (uint32_t)u;
        return Err::none;
    }

    Err str(const char *key, const char *&out) {
        JsonVariantConst v = q[key];
        if (v.isNull()) return failf(Err::missing_param, "missing parameter '%s'", key);
        if (!v.is<const char *>()) return failf(Err::bad_param, "'%s' must be a string", key);
        out = v.as<const char *>();
        return Err::none;
    }
};
}  // namespace

// ---- request dispatch -----------------------------------------------------------------------

void Core::line_too_long(Source src, Reply &r) {
    r.doc.clear();
    r.stream_spec = false;
    r.err = Err::line_too_long;
    JsonObject top = r.doc.to<JsonObject>();
    top["id"] = nullptr;
    top["ok"] = false;
    JsonObject e = top["error"].to<JsonObject>();
    e["code"] = err_name(r.err);
    e["msg"] = "request line too long (max 1024 bytes)";
    log_command(src, Op::invalid, -1, r.err, 0, 0);
}

void Core::handle_line(Source src, const char *line, size_t len, Reply &r) {
    JsonDocument req;
    DeserializationError de = deserializeJson(req, line, len, DeserializationOption::NestingLimit(4));
    if (de) {
        r.doc.clear();
        r.stream_spec = false;
        r.err = Err::bad_json;
        JsonObject top = r.doc.to<JsonObject>();
        top["id"] = nullptr;
        top["ok"] = false;
        JsonObject e = top["error"].to<JsonObject>();
        e["code"] = err_name(r.err);
        snprintf(msg_, sizeof msg_, "not JSON: %s", de.c_str());
        e["msg"] = msg_;
        log_command(src, Op::invalid, -1, r.err, 0, 0);
        return;
    }
    handle(src, req, r);
}

static bool mutating(Op op) {
    switch (op) {
    case Op::getInfo:
    case Op::getState:
    case Op::getConfig:
    case Op::getEvents:
    case Op::scanWifi:
    case Op::getSpec: return false;
    default: return true;
    }
}

void Core::handle(Source src, JsonVariantConst req, Reply &r) {
    r.doc.clear();
    r.stream_spec = false;
    r.err = Err::none;
    msg_[0] = 0;
    JsonObject top = r.doc.to<JsonObject>();
    Op op = Op::invalid;
    int ch = -1;
    uint32_t rid = 0, arg = 0;
    Err err = Err::none;

    JsonVariantConst id = req["id"];
    if (!req.is<JsonObjectConst>()) {
        top["id"] = nullptr;
        err = fail(Err::bad_request, "a request is a JSON object");
    } else if (!id.isNull() && !id.is<long long>() && !id.is<const char *>()) {
        top["id"] = nullptr;
        err = fail(Err::bad_request, "id must be an integer or a string");
    } else {
        top["id"] = id;
        if (id.is<long long>() && id.as<long long>() > 0 && id.as<long long>() <= 0xFFFFFFFFLL)
            rid = (uint32_t)id.as<long long>();
        JsonVariantConst opv = req["op"];
        if (opv.isNull()) err = fail(Err::missing_param, "missing 'op'");
        else if ((op = op_from_name(opv.as<const char *>())) == Op::invalid)
            err = fail(Err::unknown_op, "unknown op '%.32s'", opv.is<const char *>() ? opv.as<const char *>() : "?");
    }
    top["ok"] = true;
    JsonObject res = top["result"].to<JsonObject>();
    static const char *const no_params[] = {nullptr};
    Params p{req, res, msg_, sizeof msg_};
    bool takes_params = op == Op::setChannel || op == Op::pulseChannel || op == Op::bounceChannel ||
                        op == Op::setConfig || op == Op::getEvents || op == Op::setWifi || op == Op::subscribe;
    if (err == Err::none && !takes_params) err = p.keys(no_params);
    if (err == Err::none) {
        switch (op) {
        case Op::getInfo: err = op_info(res); break;
        case Op::getState: err = op_state(res); break;
        case Op::setChannel: err = op_set(src, req, rid, res, ch, arg); break;
        case Op::pulseChannel: err = op_pulse(src, req, rid, res, ch, arg); break;
        case Op::bounceChannel: err = op_bounce(src, req, rid, res, ch, arg); break;
        case Op::releaseAll: err = op_release_all(src, rid, false, res); break;
        case Op::idleAll: err = op_release_all(src, rid, true, res); break;
        case Op::getConfig: err = op_get_config(res); break;
        case Op::setConfig: err = op_set_config(src, req, rid, res); break;
        case Op::getEvents: err = op_events(req, res); break;
        case Op::setWifi: err = op_wifi(req, res); break;
        case Op::scanWifi: err = op_scan(res); break;
        case Op::subscribe: err = op_subscribe(src, req, res); break;
        case Op::getSpec:
            if (src == Source::ws)
                err = fail(Err::unsupported_transport, "the spec is too large for one WebSocket message; GET /openapi.json");
            r.stream_spec = err == Err::none;
            break;
        default: err = fail(Err::unknown_op, "unknown op"); break;
        }
    }
    r.err = err;
    if (err != Err::none) {
        r.stream_spec = false;
        top.remove("result");
        top["ok"] = false;
        JsonObject e = top["error"].to<JsonObject>();
        e["code"] = err_name(err);
        e["msg"] = msg_;
    }
    if (err != Err::none || mutating(op)) log_command(src, op, ch, err, rid, arg);
}

// Resolves "name" to a channel index.
static Err channel_param(Params &p, const Core &c, int &out) {
    const char *name;
    TRY(p.str("name", name));
    for (int i = 0; i < c.config().count; i++) {
        if (strcmp(c.config().ch[i].name, name) == 0) {
            out = i;
            return Err::none;
        }
    }
    return p.failf(Err::unknown_channel, "unknown channel '%.16s'", name);
}

// Target contact state from a contact-level key ("closed"/"open") or a semantic one
// ("trip"/"release"); exactly one may be given. `dflt` < 0 means one is required.
static Err target_param(Params &p, const ChannelConfig &c, const char *state_key, const char *action_key,
                        int dflt, bool &closed) {
    bool has_state = present(p.q, state_key), has_action = present(p.q, action_key);
    if (has_state && has_action)
        return p.failf(Err::conflicting_params, "give '%s' or '%s', not both", state_key, action_key);
    if (has_state) {
        const char *s;
        TRY(p.str(state_key, s));
        if (!strcmp(s, "closed")) closed = true;
        else if (!strcmp(s, "open")) closed = false;
        else return p.failf(Err::bad_param, "'%s' must be \"closed\" or \"open\"", state_key);
        return Err::none;
    }
    if (has_action) {
        const char *a;
        TRY(p.str(action_key, a));
        bool trip;
        if (!strcmp(a, "trip")) trip = true;
        else if (!strcmp(a, "release")) trip = false;
        else return p.failf(Err::bad_param, "'%s' must be \"trip\" or \"release\"", action_key);
        closed = c.type == Type::nc ? !trip : trip;
        return Err::none;
    }
    if (dflt < 0) return p.failf(Err::missing_param, "give '%s' or '%s'", state_key, action_key);
    closed = dflt != 0;
    return Err::none;
}

static Err ttl_param(Params &p, uint32_t &ttl) {
    ttl = 0;
    if (!present(p.q, "ttl_ms")) return Err::none;
    if (p.q["ttl_ms"].is<long long>() && p.q["ttl_ms"].as<long long>() == 0) return Err::none;
    return p.uint("ttl_ms", 1, TTL_MS_MAX, ttl);
}

Err Core::op_info(JsonObject res) {
    info_json(res);
    return Err::none;
}

void Core::info_json(JsonObject res) {
    char buf[12];
    uint64_t now = hal::now_us();
    res["fw_version"] = FW_VERSION;
    res["uptime_ms"] = now / 1000;
    res["uptime_us"] = now;
    snprintf(buf, sizeof buf, "%08lx", (unsigned long)boot_id_);
    res["boot_id"] = buf;
    res["reset_reason"] = (const char *)reset_reason_;
    res["channels"] = cfg_.count;
    JsonObject ev = res["events"].to<JsonObject>();
    ev["head"] = ring_.head();
    ev["oldest"] = ring_.oldest();
    ev["capacity"] = EVENT_RING;
    res["serial_subscribed"] = serial_subscribed_;
    hal::info(res);
}

Err Core::op_state(JsonObject res) {
    state_json(res);
    return Err::none;
}

Err Core::op_set(Source src, JsonVariantConst q, uint32_t rid, JsonObject res, int &ch, uint32_t &arg) {
    static const char *const keys[] = {"name", "state", "action", "ttl_ms", nullptr};
    Params p{q, res, msg_, sizeof msg_};
    TRY(p.keys(keys));
    int i;
    TRY(channel_param(p, *this, i));
    ch = i;
    bool target;
    TRY(target_param(p, cfg_.ch[i], "state", "action", -1, target));
    uint32_t ttl;
    TRY(ttl_param(p, ttl));
    arg = ttl;

    uint32_t s = hal::irq_save();
    uint64_t now = hal::now_us();
    Runtime &r = rt_[i];
    r.seq_kind = 0;                                     // a direct set ends any pulse or bounce
    bool changed = r.closed != target;
    if (changed) edge(i, target, src, Cause::set, rid, now);
    if (target && ttl) {
        r.ttl_at_us = now + (uint64_t)ttl * 1000;
        r.ttl_rid = rid;
    } else {
        r.ttl_at_us = 0;
    }
    service_and_rearm();
    hal::irq_restore(s);

    res["changed"] = changed;
    if (target && ttl) res["ttl_ms"] = ttl;
    else res["ttl_ms"] = nullptr;
    channel_json(i, res["channel"].to<JsonObject>());
    return Err::none;
}

Err Core::op_pulse(Source src, JsonVariantConst q, uint32_t rid, JsonObject res, int &ch, uint32_t &arg) {
    static const char *const keys[] = {"name", "ms", "us", "action", "state", nullptr};
    Params p{q, res, msg_, sizeof msg_};
    TRY(p.keys(keys));
    int i;
    TRY(channel_param(p, *this, i));
    ch = i;
    const ChannelConfig &c = cfg_.ch[i];
    bool target;
    TRY(target_param(p, c, "state", "action", c.type == Type::nc ? 0 : 1, target));   // default: trip
    bool has_ms = present(q, "ms"), has_us = present(q, "us");
    if (has_ms == has_us) return fail(has_ms ? Err::conflicting_params : Err::missing_param, "give 'ms' or 'us'%s",
                                      has_ms ? ", not both" : "");
    uint32_t dur;
    if (has_ms) {
        TRY(p.uint("ms", 1, PULSE_MS_MAX, dur));
        dur *= 1000;
    } else {
        TRY(p.uint("us", 1, PULSE_US_MAX, dur));
    }
    arg = dur;

    uint32_t s = hal::irq_save();
    Runtime &r = rt_[i];
    if (r.seq_kind) {
        hal::irq_restore(s);
        return fail(Err::busy, "%s is already running a %s", c.name, r.seq_kind == SEQ_PULSE ? "pulse" : "bounce");
    }
    if ((bool)r.closed == target) {
        hal::irq_restore(s);
        return fail(Err::no_change, "%s is already %s", c.name, target ? "closed" : "open");
    }
    uint64_t now = hal::now_us();
    edge(i, target, src, Cause::pulse, rid, now);
    r.seq_kind = SEQ_PULSE;
    r.final_closed = !target;
    r.seq_next_us = now + dur;
    r.src = (uint8_t)src;
    r.rid = rid;
    service_and_rearm();                                // pulses under INLINE_US finish right here
    hal::irq_restore(s);

    res["start_us"] = now;
    res["end_us"] = now + dur;
    res["duration_us"] = dur;
    channel_json(i, res["channel"].to<JsonObject>());
    return Err::none;
}

Err Core::op_bounce(Source src, JsonVariantConst q, uint32_t rid, JsonObject res, int &ch, uint32_t &arg) {
    static const char *const keys[] = {"name", "edges", "interval_us", "final_state", "final_action", "ttl_ms", nullptr};
    Params p{q, res, msg_, sizeof msg_};
    TRY(p.keys(keys));
    int i;
    TRY(channel_param(p, *this, i));
    ch = i;
    const ChannelConfig &c = cfg_.ch[i];
    uint32_t edges, interval, ttl;
    TRY(p.uint("edges", 1, BOUNCE_EDGES_MAX, edges));
    TRY(p.uint("interval_us", BOUNCE_INTERVAL_US_MIN, BOUNCE_INTERVAL_US_MAX, interval));
    bool final_closed;
    TRY(target_param(p, c, "final_state", "final_action", -1, final_closed));
    TRY(ttl_param(p, ttl));
    arg = edges;

    uint32_t s = hal::irq_save();
    Runtime &r = rt_[i];
    if (r.seq_kind) {
        hal::irq_restore(s);
        return fail(Err::busy, "%s is already running a %s", c.name, r.seq_kind == SEQ_PULSE ? "pulse" : "bounce");
    }
    uint64_t now = hal::now_us();
    uint64_t end = now + (uint64_t)edges * interval;
    edge(i, !r.closed, src, Cause::bounce, rid, now);
    r.seq_kind = SEQ_BOUNCE;
    r.toggles_left = (uint16_t)(edges - 1);
    r.interval_us = interval;
    r.final_closed = final_closed;
    r.seq_next_us = now + interval;
    r.src = (uint8_t)src;
    r.rid = rid;
    if (final_closed && ttl) {
        r.ttl_at_us = end + (uint64_t)ttl * 1000;
        r.ttl_rid = rid;
    } else {
        r.ttl_at_us = 0;
    }
    service_and_rearm();
    hal::irq_restore(s);

    res["start_us"] = now;
    res["end_us"] = end;
    res["edges"] = edges;
    res["interval_us"] = interval;
    res["final_state"] = final_closed ? "closed" : "open";
    if (final_closed && ttl) res["ttl_ms"] = ttl;
    else res["ttl_ms"] = nullptr;
    channel_json(i, res["channel"].to<JsonObject>());
    return Err::none;
}

// releaseAll: every contact open, nothing driven (the power-on state; NC channels read tripped).
// idleAll: every channel untripped for its type (NO open, NC closed).
Err Core::op_release_all(Source src, uint32_t rid, bool idle, JsonObject res) {
    int changed = 0;
    uint32_t s = hal::irq_save();
    uint64_t now = hal::now_us();
    for (int i = 0; i < cfg_.count; i++) {
        Runtime &r = rt_[i];
        r.seq_kind = 0;
        r.ttl_at_us = 0;
        bool target = idle && cfg_.ch[i].type == Type::nc;
        if ((bool)r.closed != target) {
            edge(i, target, src, idle ? Cause::idle_all : Cause::release_all, rid, now);
            changed++;
        }
    }
    service_and_rearm();
    hal::irq_restore(s);
    res["changed"] = changed;
    return Err::none;
}

Err Core::op_get_config(JsonObject res) {
    config_json(res);
    return Err::none;
}

Err Core::op_set_config(Source src, JsonVariantConst q, uint32_t rid, JsonObject res) {
    static const char *const keys[] = {"channels", "defaults", nullptr};
    Params p{q, res, msg_, sizeof msg_};
    TRY(p.keys(keys));
    bool has_defaults = present(q, "defaults"), has_channels = present(q, "channels");
    if (has_defaults && has_channels) return fail(Err::conflicting_params, "give 'channels' or 'defaults', not both");
    if (!has_defaults && !has_channels) return fail(Err::missing_param, "give 'channels' or 'defaults'");
    Config next;
    if (has_defaults) {
        if (!q["defaults"].is<bool>() || !q["defaults"].as<bool>()) return fail(Err::bad_param, "'defaults' must be true");
        default_config(next);
    } else if (!parse_config(q, next, msg_, sizeof msg_)) {
        return Err::invalid_config;
    }
    apply_config(next, src, rid);
    bool persisted = hal::save_config(cfg_);
    config_json(res);
    res["persisted"] = persisted;
    return Err::none;
}

Err Core::op_events(JsonVariantConst q, JsonObject res) {
    static const char *const keys[] = {"since", "limit", nullptr};
    Params p{q, res, msg_, sizeof msg_};
    TRY(p.keys(keys));
    uint32_t since = 0, limit = EVENTS_PAGE_DEFAULT;
    if (present(q, "since")) TRY(p.uint("since", 0, 0xFFFFFFFFu, since));
    if (present(q, "limit")) TRY(p.uint("limit", 1, EVENTS_PAGE_MAX, limit));
    uint32_t oldest = ring_.oldest(), head = ring_.head();
    uint32_t seq = since > oldest ? since : oldest;
    JsonArray arr = res["events"].to<JsonArray>();
    uint32_t n = 0;
    Event e;
    for (; seq < head && n < limit; seq++) {
        if (!ring_.get(seq, e)) continue;               // overwritten while paging
        event_json(e, arr.add<JsonObject>());
        n++;
    }
    res["next"] = seq;
    res["oldest"] = oldest;
    res["head"] = head;
    res["lost"] = since && since < oldest ? oldest - since : 0;
    res["more"] = seq < head;
    return Err::none;
}

Err Core::op_wifi(JsonVariantConst q, JsonObject res) {
    static const char *const keys[] = {"ssid", "password", nullptr};
    Params p{q, res, msg_, sizeof msg_};
    TRY(p.keys(keys));
    const char *ssid, *password = "";
    TRY(p.str("ssid", ssid));
    if (present(q, "password")) TRY(p.str("password", password));
    size_t sl = strlen(ssid), pl = strlen(password);
    if (sl > 32) return fail(Err::bad_param, "'ssid' is at most 32 bytes");
    if (sl == 0 && pl) return fail(Err::bad_param, "an empty ssid clears the credentials; give no password");
    if (pl && (pl < 8 || pl > 63)) return fail(Err::bad_param, "'password' is empty (open network) or 8-63 characters");
    if (!hal::save_wifi(ssid, password)) return fail(Err::storage, "could not store the credentials");
    res["ssid"] = ssid;                                 // the password is never echoed or logged
    res["cleared"] = sl == 0;
    res["reconnecting"] = true;
    return Err::none;
}

Err Core::op_scan(JsonObject res) {
    JsonArray nets = res["networks"].to<JsonArray>();
    if (!hal::wifi_scan(nets, msg_, sizeof msg_)) return Err::busy;
    res["count"] = nets.size();
    return Err::none;
}

Err Core::op_subscribe(Source src, JsonVariantConst q, JsonObject res) {
    static const char *const keys[] = {"on", nullptr};
    Params p{q, res, msg_, sizeof msg_};
    TRY(p.keys(keys));
    bool on = true;
    if (present(q, "on")) {
        if (!q["on"].is<bool>()) return fail(Err::bad_param, "'on' must be true or false");
        on = q["on"].as<bool>();
    }
    if (src == Source::http) return fail(Err::unsupported_transport, "HTTP has no event stream; use /ws or GET /api/events");
    if (src == Source::ws && !on) return fail(Err::unsupported_transport, "WebSocket clients always receive events");
    if (src == Source::serial) serial_subscribed_ = on;
    res["subscribed"] = on;
    res["next"] = ring_.head();
    return Err::none;
}

}  // namespace sb
