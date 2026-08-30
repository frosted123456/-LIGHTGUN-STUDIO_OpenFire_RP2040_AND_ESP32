// Serial command surface and persistence for the recoil engine. Split from
// the engine so the waveform stays a pure, dependency-free unit; this file
// owns parsing, replies and the NVS blob.
#include "recoil_fx.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_RP2040)
#define FX_HAVE_STORE 1
#include "nvs_flash.h"
#include "nvs.h"
#define FX_NVS_NS  "aim"
#define FX_NVS_KEY "fx0"
#endif

// Blob version: bump when fx_params_t changes shape, so a stale stored blob
// is ignored instead of misread.
#define FX_BLOB_VER 2
typedef struct { int ver; fx_params_t p; } fx_blob_t;

static void (*s_sink)(const char* line) = 0;
static int s_temp = -1;

void fx_temp_note(int state) { s_temp = state; }
int  fx_temp_get(void)       { return s_temp; }

static void reply(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void reply(const char* fmt, ...)
{
    char b[192];
    va_list ap; va_start(ap, fmt);
    const int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (s_sink) s_sink(b); else fputs(b, stdout);
}

void fx_set_reply(void (*sink)(const char* line)) { s_sink = sink; }

static void report(void)
{
    const fx_params_t* p = fx_get();
    reply("FX: on=%d drive=%d hold=%d duty=%d pulse=%d gap=%d jit=%d "
          "rumoff=%d rumms=%d space=%d auto=%d\n",
          p->enabled, p->drive_ms, p->hold_ms, p->duty_pct, p->pulses,
          p->gap_ms, p->jit_pct, p->rum_off_ms, p->rum_ms, p->space_ms,
          p->auto_ms);
}

// Parses one signed integer, advancing the cursor.
static int parse_int(const char** s)
{
    int sgn = 1, v = 0;
    if (**s == '-') { sgn = -1; ++(*s); }
    // Digits past the cap are consumed but not accumulated: signed overflow is
    // undefined behaviour, and a long digit string on a serial line is a typo
    // or a garbled byte. Every key clamps to its own range afterwards.
    while (**s >= '0' && **s <= '9') {
        const int d = *(*s)++ - '0';
        if (v < 100000000) v = v * 10 + d;
    }
    return v * sgn;
}

int fx_command(const char* line, uint64_t now_us)
{
    if (!line) return 0;
    if (!strncmp(line, "fxsave", 6)) {
        reply(fx_store() ? "FX: saved\n" : "FX: SAVE FAILED\n");
        return 1;
    }
    if (!strncmp(line, "fx?", 3)) {
        report();
        // quiet= is what a tool tests to learn this firmware HAS quiet mode:
        // an older build simply does not print the key, and the tools fall
        // back rather than believing a silence they never got.
        reply("FX: ab=%d left=%d busy=%d temp=%d quiet=%d qleft=%d\n",
              fx_ab_active(now_us), fx_ab_left_s(now_us), fx_busy(now_us),
              s_temp, fx_quiet_active(now_us), fx_quiet_left_s(now_us));
        return 1;
    }
    if (strncmp(line, "fx=", 3) != 0) return 0;

    fx_params_t p = *fx_get();
    int changed = 0;
    const char* s = line + 3;
    while (*s) {
        char key[8] = {0}; int ki = 0;
        while (*s && *s != ':' && *s != ',' && ki < 7) key[ki++] = *s++;
        if (*s != ':') { while (*s && *s != ',') ++s; if (*s) ++s; continue; }
        ++s;
        const int v = parse_int(&s);
        if (*s == ',') ++s;
        if      (!strcmp(key, "on"))     { p.enabled = v;    changed = 1; }
        else if (!strcmp(key, "drive"))  { p.drive_ms = v;   changed = 1; }
        else if (!strcmp(key, "hold"))   { p.hold_ms = v;    changed = 1; }
        else if (!strcmp(key, "duty"))   { p.duty_pct = v;   changed = 1; }
        else if (!strcmp(key, "pulse"))  { p.pulses = v;     changed = 1; }
        else if (!strcmp(key, "gap"))    { p.gap_ms = v;     changed = 1; }
        else if (!strcmp(key, "jit"))    { p.jit_pct = v;    changed = 1; }
        else if (!strcmp(key, "rumoff")) { p.rum_off_ms = v; changed = 1; }
        else if (!strcmp(key, "rumms"))  { p.rum_ms = v;     changed = 1; }
        else if (!strcmp(key, "space"))  { p.space_ms = v;   changed = 1; }
        else if (!strcmp(key, "auto"))   { p.auto_ms = v;    changed = 1; }
        else if (!strcmp(key, "ab")) {
            // Dry-fire mode: the trigger fires the solenoid without an IR
            // lock, and it disarms by itself. Answered explicitly so the
            // tools can show its true state, not their hope.
            fx_ab_set(v ? 1 : 0, now_us);
            reply("FX: dry-fire %s ab=%d left=%d\n",
                  v ? "ON" : "off", v ? 1 : 0, fx_ab_left_s(now_us));
        }
        else if (!strcmp(key, "quiet")) {
            // The calibration tools' silence switch: nothing fires and both
            // outputs belong to the engine until it is turned off or lapses.
            fx_quiet_set(v ? 1 : 0, now_us);
            // qleft, never left: the tools key their state off the token name,
            // and "left" already belongs to the dry-fire countdown. Sharing it
            // would make arming quiet claim dry-fire was armed.
            reply("FX: quiet %s quiet=%d qleft=%d\n", v ? "ON" : "off",
                  fx_quiet_active(now_us), fx_quiet_left_s(now_us));
        }
        else if (!strcmp(key, "test")) {
            // One full sequence with the current parameters, no trigger and
            // no IR involved. The refusal is reported: a silently swallowed
            // test fire looks exactly like a dead solenoid -- and "quiet mode"
            // is named as the cause, or a tool that armed quiet and forgot
            // looks like a gun that stopped working.
            if (v) {
                if (fx_quiet_active(now_us))
                    reply("FX: quiet mode is ON -- not fired\n");
                else
                    reply(fx_fire_forced(now_us) ? "FX: test fired\n"
                                                 : "FX: busy, not fired\n");
            }
        }
    }
    if (changed) {
        fx_set(&p);
        report();                 // echo what was ACCEPTED, clamps included
    }
    return 1;
}

int fx_store(void)
{
#if defined(FX_HAVE_STORE)
    fx_blob_t b; b.ver = FX_BLOB_VER; b.p = *fx_get();
    nvs_handle_t h;
    if (nvs_open(FX_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return 0;
    const int ok = (nvs_set_blob(h, FX_NVS_KEY, &b, sizeof(b)) == ESP_OK);
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
#else
    return 1;
#endif
}

int fx_load(void)
{
#if defined(FX_HAVE_STORE)
    fx_blob_t b; size_t len = sizeof(b);
    nvs_handle_t h;
    if (nvs_open(FX_NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
    const esp_err_t e = nvs_get_blob(h, FX_NVS_KEY, &b, &len);
    nvs_close(h);
    if (e != ESP_OK || len != sizeof(b) || b.ver != FX_BLOB_VER) return 0;
    fx_set(&b.p);                 // clamps, so a corrupt blob cannot inject
    return 1;
#else
    return 0;
#endif
}
