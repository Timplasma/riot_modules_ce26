/*
 Thingsat project

 GPS over UART
 Copyright (c) 2021-2023 UGA CSUG LIG

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */

/*
 * Author: Didier Donsez, Université Grenoble Alpes
 */

#ifndef _GPS_UART_H
#define _GPS_UART_H

#include <stdint.h>

/* 1: the GNSS module is read over I2C instead of UART (see gps_i2c.c) */
#ifndef GPS_I2C_ENABLE
#define GPS_I2C_ENABLE 0
#endif

int gps_start(const unsigned dev, const uint32_t baud);
void gps_restart(void);
void gps_power_off_on(void);
int gps_cmd(int argc, char **argv);

#if GPS_I2C_ENABLE == 1
/**
 * @brief Start the thread polling the GNSS module over I2C
 *        (GPS_I2C_DEV, GPS_I2C_ADDR) and feeding parse_nmea()
 */
int gps_i2c_start(void);

/**
 * @brief Print the I2C transport statistics (reads, bytes, errors)
 */
void gps_i2c_print_stats(void);
#endif

#endif
