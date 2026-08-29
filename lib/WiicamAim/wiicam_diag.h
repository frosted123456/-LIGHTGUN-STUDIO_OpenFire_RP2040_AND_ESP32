// Wiicam (SEN0158) connection diagnostic: tells WHICH part of the sensor link
// is broken, from software alone. Pure logic over injected pin callbacks, so
// the whole protocol is host-testable against a simulated bus.
//
// What it can distinguish, and how:
//   - no pull-ups on SDA/SCL  -> the sensor board carries the pull-ups, so
//     this reads as "sensor has no power or the cable is off"
//   - a line stuck low        -> short, broken wire to ground, or a wedged
//     sensor holding the bus
//   - bus healthy, no ACK     -> power path fine, data path broken or the
//     sensor is dead
//   - ACK only when the two lines are treated as swapped -> SDA and SCL
//     wires are crossed
//   - ACK at the sensor's address -> the electrical link is GOOD
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

enum {                       // pin modes the prober asks for
    WD_IN       = 0,         // input, floating
    WD_IN_PU    = 1,         // input with the chip's own pull-up
    WD_OUT_LOW  = 2,         // driven low (open-drain "low")
    WD_IN_PD    = 3,         // input with the chip's own pull-down
};

typedef struct {
    void (*pin_mode)(int pin, int mode);
    int  (*pin_read)(int pin);
    void (*delay_us)(int us);
    void (*report)(const char* line);   // one human-readable line at a time
} wd_io_t;

enum {                       // verdicts, worst first
    WD_STUCK      = 1,       // a line is held low no matter what
    WD_NO_PULLUPS = 2,       // both lines float: sensor unpowered / unplugged
    WD_NO_ACK     = 3,       // bus is electrically fine, nothing answers
    WD_SWAPPED    = 4,       // the sensor answers with SDA and SCL crossed
    WD_ACK        = 5,       // the sensor answers at its address
    WD_BROKEN_WIRE= 6,       // one line pulled up, the other floating
};

#define WD_SENSOR_ADDR 0x58  // DFRobot IR cam, 0xB0 >> 1

int wd_run(const wd_io_t* io, int sda, int scl);

// Board entry point: runs the probe on real pins, restoring their mux after.
// Returns the verdict; 0 when pins are unset or the build has no diag.
int wiicam_diag_arduino(int sda, int scl, void (*report)(const char*));

#ifdef __cplusplus
}
#endif
