// Wiicam connection diagnostic implementation. Open-drain bit-bang: a line is
// never driven high, only pulled low or released, exactly like real I2C, so
// probing cannot fight a live device.
#include "wiicam_diag.h"

#define HALF_US   5          // ~100 kHz probe clock
#define SETTLE_US 200

static const wd_io_t* IO;

static void say(const char* line) { if (IO->report) IO->report(line); }

// Majority of five spaced reads, so a floating line's noise cannot pass as a
// stable level.
static int read_stable(int pin)
{
    int hi = 0;
    for (int i = 0; i < 5; ++i) {
        IO->delay_us(20);
        hi += IO->pin_read(pin) ? 1 : 0;
    }
    return hi >= 3;
}

enum { LINE_EXT_PULLUP, LINE_FLOATING, LINE_STUCK_LOW };

// Classifies one line: an external pull-up beats the chip's own pull-down; a
// floating line follows whichever internal pull is applied; a stuck line
// reads low even against the internal pull-up.
static int line_class(int pin)
{
    IO->pin_mode(pin, WD_IN_PD);
    IO->delay_us(SETTLE_US);
    const int with_pd = read_stable(pin);
    IO->pin_mode(pin, WD_IN_PU);
    IO->delay_us(SETTLE_US);
    const int with_pu = read_stable(pin);
    IO->pin_mode(pin, WD_IN);
    IO->delay_us(SETTLE_US);
    if (with_pd) return LINE_EXT_PULLUP;
    if (!with_pu) return LINE_STUCK_LOW;
    return LINE_FLOATING;
}

static void drive_low(int pin) { IO->pin_mode(pin, WD_OUT_LOW); }
static void release(int pin)   { IO->pin_mode(pin, WD_IN); }

// Releases SCL and waits for it to actually rise (clock stretching / stuck).
static int scl_up(int scl)
{
    release(scl);
    for (int i = 0; i < 400; ++i) {       // 2 ms budget
        if (IO->pin_read(scl)) return 1;
        IO->delay_us(5);
    }
    return 0;
}

// One address probe: START, 8 address bits, read the ACK, STOP.
// Returns 1 on ACK, 0 on NACK, -1 if SCL never rose.
static int probe(int sda, int scl)
{
    release(sda); release(scl);
    IO->delay_us(SETTLE_US);
    if (!IO->pin_read(scl) || !IO->pin_read(sda)) return -1;
    // START: SDA falls while SCL is high
    drive_low(sda);  IO->delay_us(HALF_US);
    drive_low(scl);  IO->delay_us(HALF_US);
    const int byte = WD_SENSOR_ADDR << 1;         // write direction
    for (int bit = 7; bit >= 0; --bit) {
        if ((byte >> bit) & 1) release(sda); else drive_low(sda);
        IO->delay_us(HALF_US);
        if (!scl_up(scl)) return -1;
        IO->delay_us(HALF_US);
        drive_low(scl);
        IO->delay_us(HALF_US);
    }
    release(sda);                                  // let the device answer
    IO->delay_us(HALF_US);
    if (!scl_up(scl)) return -1;
    IO->delay_us(HALF_US);
    const int ack = !IO->pin_read(sda);            // low = ACK
    drive_low(scl); IO->delay_us(HALF_US);
    // STOP: SDA rises while SCL is high
    drive_low(sda); IO->delay_us(HALF_US);
    release(scl);   IO->delay_us(HALF_US);
    release(sda);   IO->delay_us(HALF_US);
    return ack;
}

int wd_run(const wd_io_t* io, int sda, int scl)
{
    IO = io;
    say("CAM: diag -- sensor connection test");

    const int csda = line_class(sda);
    const int cscl = line_class(scl);
    say(csda == LINE_EXT_PULLUP ? "CAM: diag SDA pull-up present"
        : csda == LINE_STUCK_LOW ? "CAM: diag SDA STUCK LOW"
        : "CAM: diag SDA floating (no pull-up)");
    say(cscl == LINE_EXT_PULLUP ? "CAM: diag SCL pull-up present"
        : cscl == LINE_STUCK_LOW ? "CAM: diag SCL STUCK LOW"
        : "CAM: diag SCL floating (no pull-up)");

    if (csda == LINE_STUCK_LOW || cscl == LINE_STUCK_LOW) {
        say("CAM: diag VERDICT: a line is held low -- shorted or broken wire, "
            "or a wedged sensor. Power-cycle; if it persists, check that wire.");
        return WD_STUCK;
    }
    if (csda != LINE_EXT_PULLUP && cscl != LINE_EXT_PULLUP) {
        say("CAM: diag VERDICT: no pull-ups on either line. The pull-ups live "
            "on the sensor board, so the sensor has NO POWER or the cable is "
            "unplugged. Check VCC and GND first.");
        return WD_NO_PULLUPS;
    }
    if (csda != LINE_EXT_PULLUP || cscl != LINE_EXT_PULLUP) {
        // One pulled up, one floating: the pulled line proves the sensor has
        // power and that wire conducts; the floating one is the break. The
        // probe cannot run without both lines, so this IS the verdict --
        // falling through mislabelled exactly this fault as a clock problem.
        say(csda != LINE_EXT_PULLUP
            ? "CAM: diag VERDICT: power and SCL are good; the SDA wire is "
              "BROKEN (wire or connector), sensor side to board side."
            : "CAM: diag VERDICT: power and SDA are good; the SCL wire is "
              "BROKEN (wire or connector), sensor side to board side.");
        return WD_BROKEN_WIRE;
    }

    const int normal = probe(sda, scl);
    if (normal == 1) {
        say("CAM: diag VERDICT: sensor ANSWERS at its address -- the wiring "
            "and power are good.");
        return WD_ACK;
    }
    const int swapped = probe(scl, sda);
    if (swapped == 1) {
        say("CAM: diag VERDICT: sensor answers only with the lines CROSSED -- "
            "SDA and SCL are swapped somewhere in the harness.");
        return WD_SWAPPED;
    }
    if (normal == -1 || swapped == -1) {
        say("CAM: diag VERDICT: the clock line will not rise during a probe -- "
            "intermittent short or a device clamping the bus.");
        return WD_STUCK;
    }
    say("CAM: diag VERDICT: bus is powered and clean but NOTHING answers at "
        "0x58 -- a data wire is broken at one end, or the sensor is dead.");
    return WD_NO_ACK;
}
