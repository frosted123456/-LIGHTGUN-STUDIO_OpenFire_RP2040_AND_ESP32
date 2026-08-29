// RP2040 pin glue for the wiicam connection diagnostic: maps the prober's pin
// requests onto real GPIO, saving and restoring each pin's mux so the live
// I2C block gets its pins back exactly as they were.
#include "wiicam_diag.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <Arduino.h>
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>

static void (*s_rep)(const char*) = 0;

static void d_mode(int pin, int mode)
{
    switch (mode) {
    case WD_IN:      gpio_set_dir(pin, false); gpio_set_pulls(pin, false, false); break;
    case WD_IN_PU:   gpio_set_dir(pin, false); gpio_set_pulls(pin, true,  false); break;
    case WD_IN_PD:   gpio_set_dir(pin, false); gpio_set_pulls(pin, false, true);  break;
    case WD_OUT_LOW: gpio_put(pin, 0);         gpio_set_dir(pin, true);           break;
    }
}
static int  d_read(int pin) { return gpio_get(pin); }
static void d_delay(int us) { delayMicroseconds(us); }
static void d_report(const char* l)
{
    if (!s_rep) return;
    char b[160];
    snprintf(b, sizeof(b), "%s\n", l);
    s_rep(b);
}

int wiicam_diag_arduino(int sda, int scl, void (*rep)(const char*))
{
    s_rep = rep;
    if (sda < 0 || scl < 0) {
        d_report("CAM: diag camera pins are not configured in the profile");
        return 0;
    }
    const gpio_function_t fs = gpio_get_function(sda);
    const gpio_function_t fc = gpio_get_function(scl);
    gpio_set_function(sda, GPIO_FUNC_SIO);
    gpio_set_function(scl, GPIO_FUNC_SIO);
    static const wd_io_t io = { d_mode, d_read, d_delay, d_report };
    const int v = wd_run(&io, sda, scl);
    // hand the pins back to the I2C block untouched
    gpio_set_dir(sda, false); gpio_set_pulls(sda, false, false);
    gpio_set_dir(scl, false); gpio_set_pulls(scl, false, false);
    gpio_set_function(sda, fs);
    gpio_set_function(scl, fc);
    return v;
}

#else   // other boards and the host: report honestly instead of pretending

int wiicam_diag_arduino(int sda, int scl, void (*rep)(const char*))
{
    (void)sda; (void)scl;
    if (rep) rep("CAM: diag is only available on the RP2040 wiicam build\n");
    return 0;
}

#endif
