// Host build of the switchbox core (tests/host/test_switchbox_core.py): fake pins, clock and timer,
// driven by one command per stdin line. Every output line is one JSON object.
//
//   {...}                 a request, as on the serial port; prints the reply
//   @boot                 Core::begin with the default config (fake clock keeps running)
//   @advance <us>         move the clock, firing the timer interrupt whenever it is due
//   @poll                 Core::poll_lines()
//   @ext <gpio> <0|1>     level an undriven line floats to (TinyG pull-up = 1)
//   @pins                 {"pins":{"<gpio>":"low"|"z"}} for every configured pin
//   @src serial|http|ws   source for the following requests
//   @long                 the reply to an overlong line
//   @savefail <0|1>       make save_config fail
//   @scanfail <0|1>       make wifi_scan fail
//   @wifi                 what save_wifi stored (password length only)
//   @names                the core's op/error/source/kind/cause names
//   @gpios                gpio_forbidden/gpio_caveat for GPIO -1..17
//   @status               clock, armed timer, spins
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <string>

#include "sbcore.h"

namespace {
uint64_t clock_us = 1000;
uint64_t armed_at = 0;
int irq_depth = 0;
unsigned spins = 0;
bool save_ok = true;
bool scan_ok = true;
unsigned saves = 0;
std::string wifi_ssid, wifi_password;
struct Pin {
    bool setup;
    bool driven;
    int ext;
};
Pin pins[17];
}  // namespace

namespace sb {
namespace hal {
uint64_t now_us() { return clock_us; }
void pin_drive(uint8_t gpio, bool closed) {
    assert(gpio <= 16 && pins[gpio].setup && !gpio_forbidden(gpio));
    pins[gpio].driven = closed;
}
uint8_t pin_read(uint8_t gpio) {
    assert(gpio <= 16);
    return pins[gpio].driven ? 0 : (uint8_t)pins[gpio].ext;
}
uint32_t irq_save() { return (uint32_t)irq_depth++; }
void irq_restore(uint32_t state) {
    irq_depth--;
    assert((uint32_t)irq_depth == state);
}
void timer_arm(uint64_t at_us) {
    assert(irq_depth > 0);              // the core only arms with interrupts masked
    armed_at = at_us;
}
void spin_until(uint64_t at_us) {
    assert(irq_depth > 0);
    spins++;
    if (at_us > clock_us) clock_us = at_us;
}
void pin_setup(uint8_t gpio) {
    assert(gpio <= 16 && !gpio_forbidden(gpio));
    pins[gpio].setup = true;
    pins[gpio].driven = false;
}
void pin_reset(uint8_t gpio) {
    assert(gpio <= 16);
    pins[gpio].setup = false;
    pins[gpio].driven = false;
}
void info(JsonObject out) { out["platform"] = "host"; }
bool save_config(const Config &) {
    saves++;
    return save_ok;
}
bool save_wifi(const char *ssid, const char *password) {
    wifi_ssid = ssid;
    wifi_password = password;
    return true;
}
bool wifi_scan(JsonArray out, char *why, size_t why_len) {
    if (!scan_ok) {
        snprintf(why, why_len, "scan refused by the test");
        return false;
    }
    static const struct { const char *ssid; int rssi, channel; } nets[] = {{"ExampleNet", -48, 6}, {"other", -80, 11}};
    for (const auto &n : nets) {
        JsonObject o = out.add<JsonObject>();
        o["ssid"] = n.ssid;
        o["rssi"] = n.rssi;
        o["channel"] = n.channel;
        o["encryption"] = "wpa2";
        o["hidden"] = false;
    }
    return true;
}
}  // namespace hal
}  // namespace sb

static void print(JsonDocument &doc) {
    std::string s;
    serializeJson(doc, s);
    fputs(s.c_str(), stdout);
    fputc('\n', stdout);
}

static void advance(uint64_t to) {
    while (armed_at && armed_at <= to) {
        if (armed_at > clock_us) clock_us = armed_at;
        armed_at = 0;
        irq_depth++;                        // the interrupt runs with interrupts masked
        sb::core().on_timer();
        irq_depth--;
    }
    if (to > clock_us) clock_us = to;
}

static void names(JsonDocument &doc) {
    JsonArray ops = doc["ops"].to<JsonArray>();
    for (int i = 0; i < (int)sb::Op::invalid; i++) ops.add(sb::op_name((sb::Op)i));
    JsonArray errs = doc["errors"].to<JsonArray>();
    for (int i = 1; i < (int)sb::Err::COUNT; i++) errs.add(sb::err_name((sb::Err)i));
    JsonArray src = doc["sources"].to<JsonArray>();
    for (int i = 0; i < (int)sb::Source::COUNT; i++) src.add(sb::source_name((sb::Source)i));
    JsonArray kinds = doc["kinds"].to<JsonArray>();
    for (int i = 0; i < (int)sb::Kind::COUNT; i++) kinds.add(sb::kind_name((sb::Kind)i));
    JsonArray causes = doc["causes"].to<JsonArray>();
    for (int i = 0; i < (int)sb::Cause::COUNT; i++) causes.add(sb::cause_name((sb::Cause)i));
    JsonArray wifi = doc["wifi"].to<JsonArray>();
    for (int i = 0; i < (int)sb::Wifi::COUNT; i++) wifi.add(sb::wifi_name((sb::Wifi)i));
    doc["event_ring"] = sb::EVENT_RING;
}

int main() {
    for (auto &p : pins) p = Pin{false, false, 1};
    sb::Source src = sb::Source::serial;
    sb::Reply reply;
    std::string line;
    while (std::getline(std::cin, line)) {
        JsonDocument out;
        if (line.empty()) continue;
        if (line[0] != '@') {
            sb::core().handle_line(src, line.data(), line.size(), reply);
            if (reply.stream_spec) reply.doc["stream_spec"] = true;
            if (reply.stream_board) reply.doc["stream_board"] = true;
            print(reply.doc);
        } else if (line == "@boot") {
            sb::Config c;
            sb::default_config(c);
            sb::core().begin(c, "host", 0x1234abcd);
            out["booted"] = true;
            print(out);
        } else if (line.rfind("@advance ", 0) == 0) {
            advance(clock_us + strtoull(line.c_str() + 9, nullptr, 10));
            out["t_us"] = clock_us;
            print(out);
        } else if (line == "@poll") {
            sb::core().poll_lines();
            out["polled"] = true;
            print(out);
        } else if (line.rfind("@ext ", 0) == 0) {
            int gpio = -1, level = 0;
            int got = sscanf(line.c_str() + 5, "%d %d", &gpio, &level);
            assert(got == 2 && gpio >= 0 && gpio <= 16);
            (void)got;
            pins[gpio].ext = level;
            out["ext"] = gpio;
            print(out);
        } else if (line == "@pins") {
            JsonObject p = out["pins"].to<JsonObject>();
            char key[4];
            for (int g = 0; g <= 16; g++) {
                if (!pins[g].setup) continue;
                snprintf(key, sizeof key, "%d", g);
                p[key] = pins[g].driven ? "low" : "z";
            }
            print(out);
        } else if (line.rfind("@src ", 0) == 0) {
            std::string s = line.substr(5);
            src = s == "http" ? sb::Source::http : s == "ws" ? sb::Source::ws : sb::Source::serial;
            out["src"] = sb::source_name(src);
            print(out);
        } else if (line == "@long") {
            sb::core().line_too_long(src, reply);
            print(reply.doc);
        } else if (line.rfind("@scanfail ", 0) == 0) {
            scan_ok = line[10] == '0';
            out["scan_ok"] = scan_ok;
            print(out);
        } else if (line.rfind("@savefail ", 0) == 0) {
            save_ok = line[10] == '0';
            out["save_ok"] = save_ok;
            print(out);
        } else if (line == "@wifi") {
            out["ssid"] = wifi_ssid;
            out["password_len"] = wifi_password.size();
            print(out);
        } else if (line == "@names") {
            names(out);
            print(out);
        } else if (line == "@gpios") {
            JsonArray arr = out["gpios"].to<JsonArray>();
            for (long g = -1; g <= 17; g++) {
                JsonObject o = arr.add<JsonObject>();
                o["gpio"] = g;
                const char *bad = sb::gpio_forbidden(g), *caveat = sb::gpio_caveat(g);
                if (bad) o["forbidden"] = bad;
                else o["forbidden"] = nullptr;
                if (caveat) o["caveat"] = caveat;
                else o["caveat"] = nullptr;
            }
            print(out);
        } else if (line == "@status") {
            out["t_us"] = clock_us;
            out["armed_at"] = armed_at;
            out["spins"] = spins;
            out["saves"] = saves;
            print(out);
        } else {
            fprintf(stderr, "unknown command: %s\n", line.c_str());
            return 2;
        }
        assert(irq_depth == 0);
        fflush(stdout);
    }
    return 0;
}
