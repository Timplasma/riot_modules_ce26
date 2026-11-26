/*
 Thingsat project

 GPS over I2C (eg GlobalTop/MediaTek XA1110 at address 0x10)
 Copyright (c) 2021-2026 UGA CSUG LIG

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */

/*
 * The GNSS module streams the same raw NMEA bytes over I2C as over UART.
 * This file polls the module over I2C and feeds every byte to parse_nmea(),
 * so the whole NMEA parsing (fix, position, time, RTC ...) is shared with
 * the UART transport.
 *
 * Enabled with GPS_I2C_ENABLE=1 (see apps/basic_mission/Makefile.gnss.mk).
 */

#ifndef GPS_I2C_ENABLE
#define GPS_I2C_ENABLE 0
#endif

#if GPS_I2C_ENABLE == 1

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "periph/i2c.h"
#include "thread.h"
#include "ztimer.h"

#include "parse_nmea.h"
#include "gps_uart.h"

/* I2C bus number of the GNSS module */
#ifndef GPS_I2C_DEV
#define GPS_I2C_DEV             0
#endif

/* 7-bit I2C address of the GNSS module (0x10 for the XA1110) */
#ifndef GPS_I2C_ADDR
#define GPS_I2C_ADDR            (0x10)
#endif

/* Delay between two reads when the module has no data to send */
#ifndef GPS_I2C_POLL_MS
#define GPS_I2C_POLL_MS         (100U)
#endif

/* Delay between two reads while NMEA data is flowing (leaves the bus to other devices) */
#ifndef GPS_I2C_BUSY_POLL_MS
#define GPS_I2C_BUSY_POLL_MS    (5U)
#endif

/* Number of bytes read per I2C transaction */
#ifndef GPS_I2C_CHUNK_SIZE
#define GPS_I2C_CHUNK_SIZE      (128U)
#endif

/* Lower priority than main: the I2C driver busy-waits and parsing is not time critical */
#ifndef GPS_I2C_THREAD_PRIO
#define GPS_I2C_THREAD_PRIO     (THREAD_PRIORITY_MAIN + 1)
#endif

static char _gps_i2c_stack[THREAD_STACKSIZE_MAIN];
static kernel_pid_t _gps_i2c_pid = KERNEL_PID_UNDEF;

/* Statistics (written by the GNSS I2C thread, read by the shell) */
static volatile uint32_t _gps_i2c_reads = 0;   /* successful I2C read transactions */
static volatile uint32_t _gps_i2c_bytes = 0;   /* useful NMEA bytes given to the parser */
static volatile uint32_t _gps_i2c_errors = 0;  /* failed I2C read transactions */

static void *_gps_i2c_thread(void *arg)
{
    (void)arg;
    uint8_t buf[GPS_I2C_CHUNK_SIZE];
    uint8_t last = 0;

    while (1) {
        /* The bus is shared with the other I2C sensors */
        i2c_acquire(I2C_DEV(GPS_I2C_DEV));
        int res = i2c_read_bytes(I2C_DEV(GPS_I2C_DEV), GPS_I2C_ADDR, buf, sizeof(buf), 0);
        i2c_release(I2C_DEV(GPS_I2C_DEV));

        if (res != 0) {
            _gps_i2c_errors++;
            ztimer_sleep(ZTIMER_MSEC, GPS_I2C_POLL_MS);
            continue;
        }
        _gps_i2c_reads++;

        bool got_data = false;
        for (unsigned i = 0; i < sizeof(buf); i++) {
            const uint8_t c = buf[i];
            /*
             * When its output buffer is empty, the module pads the read with
             * 0x0A ('\n') bytes. A genuine NMEA line ends with "\r\n", so a '\n'
             * that does not follow a '\r' is padding and must be dropped
             * (otherwise each one would be parsed as an empty bad-checksum line).
             */
            if (c == '\n' && last != '\r') {
                continue;
            }
            got_data = true;
            _gps_i2c_bytes++;
            parse_nmea(c);
            last = c;
        }

        ztimer_sleep(ZTIMER_MSEC, got_data ? GPS_I2C_BUSY_POLL_MS : GPS_I2C_POLL_MS);
    }

    /* this should never be reached */
    return NULL;
}

int gps_i2c_start(void)
{
    if (_gps_i2c_pid != KERNEL_PID_UNDEF) {
        /* never run two readers on the same stack and parser */
        puts("WARN: GNSS I2C reader already started");
        return 0;
    }

    _gps_i2c_pid = thread_create(_gps_i2c_stack, sizeof(_gps_i2c_stack),
                                 GPS_I2C_THREAD_PRIO, 0,
                                 _gps_i2c_thread, NULL, "gps_i2c");
    if (_gps_i2c_pid < 0) {
        puts("ERROR: Unable to start the GNSS I2C reader thread");
        _gps_i2c_pid = KERNEL_PID_UNDEF;
        return 1;
    }

    printf("INFO: GNSS over I2C_DEV(%d) at address 0x%02x\n", GPS_I2C_DEV, GPS_I2C_ADDR);
    return 0;
}

void gps_i2c_print_stats(void)
{
    printf("INFO: GNSS I2C dev=%d addr=0x%02x reads=%lu bytes=%lu errors=%lu\n",
           GPS_I2C_DEV, GPS_I2C_ADDR,
           (unsigned long)_gps_i2c_reads, (unsigned long)_gps_i2c_bytes,
           (unsigned long)_gps_i2c_errors);
}

#endif /* GPS_I2C_ENABLE == 1 */
