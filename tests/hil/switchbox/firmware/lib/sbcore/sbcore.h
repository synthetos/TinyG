// Switchbox core: channel model, request validation and execution, edge sequencer, event ring.
//
// Plain C++ with no Arduino dependency, so tests/host compiles it on the host under ASan/UBSan.
// The platform (src/main.cpp on the ESP8266, test/host/sb_host.cpp on the host) supplies the
// sb::hal functions declared at the bottom.
//
// Contact model: a channel emulates a switch contact wired between a TinyG input and GND.
//   closed = the pin sinks the line to GND (output enabled, output latch low)
//   open   = the pin is high-Z (input, no pull-up); TinyG's own pull-up raises the line
// The pin is never driven high. A channel's type (no/nc) only maps the semantic actions:
//   NO: trip = closed, release = open      NC: trip = open, release = closed
//
// Concurrency: requests run in the main loop. Timed edges (pulse ends, bounce edges, ttl expiry)
// run from the timer interrupt via Core::on_timer(). Loop code touches channel runtime state and
// the event ring only inside hal::irq_save()/irq_restore(); everything the interrupt calls is
// SB_IRAM and avoids library calls.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <ArduinoJson.h>

#ifndef SB_IRAM
#if defined(ESP8266)
#include <c_types.h>
#define SB_IRAM IRAM_ATTR
#else
#define SB_IRAM
#endif
#endif

namespace sb {

constexpr const char FW_VERSION[] = "0.1.0";

constexpr int MAX_CHANNELS = 8;
constexpr int NAME_LEN = 15;
constexpr int EVENT_RING = 128;             // power of two: the interrupt indexes it with a mask
constexpr uint32_t EVENTS_PAGE_DEFAULT = 16;
constexpr uint32_t EVENTS_PAGE_MAX = 32;

// Parameter limits; out-of-range values are clamped and the reply lists them in "clamped".
constexpr uint32_t TTL_MS_MAX = 3600000;
constexpr uint32_t PULSE_US_MAX = 1000000;
constexpr uint32_t PULSE_MS_MAX = 60000;
constexpr uint32_t BOUNCE_EDGES_MAX = 64;
constexpr uint32_t BOUNCE_INTERVAL_US_MIN = 2;
constexpr uint32_t BOUNCE_INTERVAL_US_MAX = 100000;
// Edges closer together than this are timed by spinning with interrupts masked instead of by the
// timer interrupt (whose entry latency is a few us). Worst case masked: 64 edges x 50 us = 3.2 ms.
constexpr uint32_t INLINE_US = 50;

enum class Type : uint8_t { no, nc };
enum class Source : uint8_t { boot, serial, http, ws, ttl, poll, sys, COUNT };
enum class Op : uint8_t {
    getInfo, getState, setChannel, pulseChannel, bounceChannel, releaseAll, idleAll,
    getConfig, setConfig, getEvents, setWifi, scanWifi, getSpec, subscribe, invalid, COUNT
};
enum class Kind : uint8_t { boot, command, edge, line, config, wifi, COUNT };
enum class Cause : uint8_t { set, pulse, bounce, ttl, release_all, idle_all, config, COUNT };
enum class Err : uint8_t {
    none, bad_json, bad_request, unknown_op, unknown_param, missing_param, bad_param,
    conflicting_params, unknown_channel, no_change, busy, invalid_config, line_too_long,
    unsupported_transport, storage, COUNT
};
enum class Wifi : uint8_t { off, connecting, sta, ap, sta_failed, COUNT };

const char *op_name(Op op);
const char *err_name(Err e);
const char *source_name(Source s);
const char *kind_name(Kind k);
const char *cause_name(Cause c);
const char *wifi_name(Wifi w);
Op op_from_name(const char *name);
int http_status(Err e);

struct ChannelConfig {
    char name[NAME_LEN + 1];
    uint8_t gpio;
    Type type;
};

struct Config {
    uint8_t count;
    ChannelConfig ch[MAX_CHANNELS];
};

void default_config(Config &c);
// Why a GPIO cannot carry a channel, or nullptr if it can.
const char *gpio_forbidden(long gpio);
// Caveat for a usable GPIO (boot-strap pins), or nullptr.
const char *gpio_caveat(long gpio);
// Validates a {"channels":[...]} object into `out`; on failure writes a reason into `why`.
bool parse_config(JsonVariantConst v, Config &out, char *why, size_t why_len);

struct Event {
    uint64_t t_us;
    uint32_t seq;
    uint32_t rid;       // request id when it was a positive integer, else 0
    uint32_t arg;       // command: ttl_ms / duration_us / edges; boot: unused; wifi: Wifi
    uint8_t kind;       // Kind
    uint8_t src;        // Source
    uint8_t code;       // command: Op; edge: Cause
    int8_t ch;          // channel index or -1
    uint8_t value;      // command: ok; edge: closed; line: level
    uint8_t err;        // command: Err
    uint8_t levels;     // bit i = line level of channel i when the event was recorded
    uint8_t driven;     // bit i = channel i contact closed
};

class EventRing {
  public:
    // Appends an event, assigning the next seq; safe from the interrupt and from the loop.
    uint32_t push(uint8_t kind, uint8_t src, uint8_t code, int8_t ch, uint8_t value, uint8_t err,
                  uint32_t rid, uint32_t arg, uint64_t t_us, uint8_t levels, uint8_t driven);
    bool get(uint32_t seq, Event &out) const;   // false if not yet written or already overwritten
    uint32_t head() const;                      // seq the next event will get
    uint32_t oldest() const;                    // oldest seq still held
    void clear();

  private:
    Event buf_[EVENT_RING] = {};
    uint32_t next_ = 1;
};

// One request's reply. The platform serializes doc; for getSpec it streams the embedded spec
// as the result instead.
struct Reply {
    JsonDocument doc;
    Err err = Err::none;
    bool stream_spec = false;
};

class Core {
  public:
    void begin(const Config &cfg, const char *reset_reason, uint32_t boot_id);
    const Config &config() const { return cfg_; }
    const EventRing &events() const { return ring_; }

    // Parses one line of the serial/WebSocket framing and executes it.
    void handle_line(Source src, const char *line, size_t len, Reply &r);
    // Executes an already parsed request object ({"id":..,"op":..,...params}).
    void handle(Source src, JsonVariantConst req, Reply &r);
    // The reply to a line that overflowed the input buffer.
    void line_too_long(Source src, Reply &r);

    void poll_lines();                  // from the loop, every ~10 ms: line-level watch
    void on_timer();                    // from the timer interrupt
    void log_wifi(Wifi state);
    bool serial_subscribed() const { return serial_subscribed_; }

    void event_json(const Event &e, JsonObject out) const;
    void info_json(JsonObject out);
    void state_json(JsonObject out);
    void config_json(JsonObject out) const;
    void channel_json(int i, JsonObject out);
    uint32_t boot_id() const { return boot_id_; }
    const char *reset_reason() const { return reset_reason_; }

  private:
    struct Runtime {
        uint8_t closed;
        uint8_t seq_kind;               // 0 none, SEQ_PULSE, SEQ_BOUNCE
        uint8_t final_closed;
        uint8_t src;                    // Source of the running pulse/bounce
        uint16_t toggles_left;
        uint32_t interval_us;
        uint32_t rid, ttl_rid;
        uint64_t seq_next_us;
        uint64_t ttl_at_us;             // 0 = no ttl
        uint64_t since_us;              // last contact change
        // loop-only line watch
        uint8_t last_level;
        uint8_t flaps, flaps_prev;
        uint64_t flap_window_us;
    };
    static constexpr uint8_t SEQ_PULSE = 1, SEQ_BOUNCE = 2;

    void edge(int i, bool closed, Source src, Cause cause, uint32_t rid, uint64_t now);
    uint64_t service(uint64_t now);
    void service_and_rearm();
    uint8_t levels() const;
    uint8_t driven() const;
    int find(const char *name) const;
    const char *warning(uint8_t level, const Runtime &r) const;
    void apply_config(const Config &cfg, Source src, uint32_t rid);
    void log_command(Source src, Op op, int ch, Err err, uint32_t rid, uint32_t arg);

    // op handlers: fill r.doc["result"] or return an error (message in msg_)
    Err op_info(JsonObject res);
    Err op_state(JsonObject res);
    Err op_set(Source src, JsonVariantConst q, uint32_t rid, JsonObject res, int &ch, uint32_t &arg);
    Err op_pulse(Source src, JsonVariantConst q, uint32_t rid, JsonObject res, int &ch, uint32_t &arg);
    Err op_bounce(Source src, JsonVariantConst q, uint32_t rid, JsonObject res, int &ch, uint32_t &arg);
    Err op_release_all(Source src, uint32_t rid, bool idle, JsonObject res);
    Err op_get_config(JsonObject res);
    Err op_set_config(Source src, JsonVariantConst q, uint32_t rid, JsonObject res);
    Err op_events(JsonVariantConst q, JsonObject res);
    Err op_wifi(JsonVariantConst q, JsonObject res);
    Err op_scan(JsonObject res);
    Err op_subscribe(Source src, JsonVariantConst q, JsonObject res);
    Err fail(Err e, const char *fmt, ...);

    Config cfg_ = {};
    Runtime rt_[MAX_CHANNELS] = {};
    EventRing ring_;
    bool serial_subscribed_ = false;
    uint32_t boot_id_ = 0;
    char reset_reason_[32] = "";
    char msg_[160] = "";
};

Core &core();

// ---- platform hooks -------------------------------------------------------------------------
// Those marked [isr] are called from the timer interrupt and must be in IRAM on the ESP8266.
namespace hal {
uint64_t now_us();                          // [isr] monotonic microseconds since boot, 64-bit
void pin_drive(uint8_t gpio, bool closed);  // [isr] closed: output enabled, latch low; open: high-Z
uint8_t pin_read(uint8_t gpio);             // [isr] actual line level, 0 or 1
uint32_t irq_save();                        // [isr] mask interrupts, return the previous state
void irq_restore(uint32_t state);           // [isr]
void timer_arm(uint64_t at_us);             // [isr] fire Core::on_timer() at at_us; 0 cancels
void spin_until(uint64_t at_us);            // [isr] busy-wait (interrupts are already masked)
void pin_setup(uint8_t gpio);               // pin becomes an open contact (input, no pull, latch low)
void pin_reset(uint8_t gpio);               // pin no longer used by a channel: plain input
void info(JsonObject out);                  // platform fields for getInfo (heap, WiFi, versions)
bool save_config(const Config &cfg);        // persist; false on storage failure
bool save_wifi(const char *ssid, const char *password);   // persist and reconnect
bool wifi_scan(JsonArray out, char *why, size_t why_len);  // blocking scan; false (why) if busy/failed
}  // namespace hal

}  // namespace sb
