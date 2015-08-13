#include "rplidar.h"

#include <math.h>
#include <stdio.h>

#include "common/serial.h"
#include "common/ioutils.h"
#include "common/timestamp.h"

#define VERBOSE 1

volatile int halt = 0;

// Debugging
static void dump(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; ++i)
        printf("%x ", buf[i]);
    printf("\n");
}

/* Send a request to the device. Note that not all requests have responses, and
 * some requests have more than one response.
 */
static void send_command_raw(int dev, uint8_t request, uint8_t *payload, uint8_t len)
{
    uint8_t *req = malloc((4 + len)*sizeof(uint8_t));
    req[0] = MAGIC_0;
    req[1] = request;
    req[2] = len;

    for (int i = 0; i < len; i++)
        req[3+i] = payload[i];

    // Checksum
    int csidx = 4+len-1;
    req[csidx] = 0;
    for (int i = 0; i < csidx; i++)
        req[csidx] ^= req[i];

    // Write to device
    int res = write(dev, req, 4+len);
    if (res != 4+len)
        printf("ERR: There was a problem writing to the device\n");
    free(req);
}

static void read_response_descriptor(int dev, rp_descriptor_t *rd)
{
    rd->len = -1;

    // First, look for the magic bytes
    int res = 0;
    int header_have = 0;
    uint8_t buf[7];
    while (header_have < 2) {
        res = read_fully_timeout(dev, buf+header_have, 2-header_have, TIMEOUT_MS);
        if (res < 1)
            return;

        // Check for magic bytes
        if (buf[0] == MAGIC_0 && buf[1] == MAGIC_1)
            break;

        // Otherwise, shift buffer and read one more byte in
        header_have = 1;
        buf[0] = buf[1];
    }

    // Read in last 5 bytes of response descriptor
    res = read_fully_timeout(dev, buf+2, 5, TIMEOUT_MS);
    if (res < 1)
        return;
    if (VERBOSE)
        dump(buf, 7);

    rd->len = (buf[2] << 22) | (buf[3] << 14) | (buf[4] << 6) | ((buf[5] & 0xF7) >> 2);
    rd->send_mode = (0x3 & buf[5]);
    rd->data_type = buf[6];

    return;
}

void rp_lidar_stop(int dev)
{
    halt = 1;
    send_command_raw(dev, REQUEST_STOP, NULL, 0);
}

void rp_lidar_reset(int dev)
{
    halt = 0;
    send_command_raw(dev, REQUEST_RESET, NULL, 0);
}

void rp_lidar_scan(int dev, lcm_t *lcm, const char *channel)
{
    send_command_raw(dev, REQUEST_SCAN, NULL, 0);

    // Reading info back. Multiple response. Loops forever!
    rp_descriptor_t rd;
    read_response_descriptor(dev, &rd);
    if (rd.len < 0) {
        printf("ERR: Could not read back scan descriptor\n");
        return;
    }

    const float d2r = 2.0f*M_PI/360.0f;

    // Gather information forever and broadcast complete scans
    // Scan packets are 5 bytes
    rplidar_laser_t laser = {
        .ranges = calloc(1000, sizeof(*laser.ranges)),
        .thetas = calloc(1000, sizeof(*laser.thetas)),
        .times = calloc(1000, sizeof(*laser.times)),
        .intensities = calloc(1000, sizeof(*laser.intensities)),
    };

    int32_t count = 0;
    const int64_t startup = utime_now();
    while (!halt) {
        // Read in bytes
        uint8_t buf[5];
        int res = read_fully_timeout(dev, buf, 5, TIMEOUT_MS);
        int64_t now = utime_now();
        if (res < 5) {
            if (VERBOSE && (now-startup>1000000))
                printf("ERR: Could not read range return\n");
            continue;
        }

        int8_t quality = (buf[0] & 0xfc) >> 2;
        int16_t angle = ((buf[1] & 0xfe) >> 1) | (buf[2] << 7);
        int16_t range = buf[3] | (buf[4] << 8);

        if (range > 0) {
            laser.ranges[count] = (range/4.0f)/1000.0f;
            laser.thetas[count] = (angle/64.0f)*d2r;
            laser.times[count] = now;
            laser.intensities[count] = ((float)quality)/0x3f;

            count++;
        }

        // Check for new scan
        if ((buf[0] & 0x01) && !(buf[0] & 0x02)) {
            if (count) {
                laser.utime = now;
                laser.nranges = count;
                laser.nintensities = count;
                rplidar_laser_t_publish(lcm, channel, &laser);
                count = 0;
            }
        }
    }

    halt = 0;
    free(laser.ranges);
    free(laser.thetas);
    free(laser.times);
    free(laser.intensities);
}

void rp_lidar_force_scan(int dev, lcm_t *lcm, const char *channel)
{
    printf("NOT FULLY IMPLEMENTED\n");
    send_command_raw(dev, REQUEST_FORCE_SCAN, NULL, 0);
}

void rp_lidar_check_info(int dev)
{
    send_command_raw(dev, REQUEST_GET_INFO, NULL, 0);

    // Read info back. Single response
    rp_descriptor_t rd;
    read_response_descriptor(dev, &rd);
    if (rd.len < 0) {
        printf("ERR: Could not read back device info descriptor\n");
        return;
    }

    // Next 20 bytes are information
    int i;
    uint8_t buf[20];
    int res = read_fully_timeout(dev, buf, 20, TIMEOUT_MS);
    if (res < 0) {
        printf("ERR: Could not read back device info\n");
        return;
    }
    if (VERBOSE)
        dump(buf, 20);

    printf("Model: %d\nFirmware: %d.%d\nHardware: %d\nSerial: ", buf[0], buf[2], buf[1], buf[3]);
    for (i = 4; i < 20; i++)
        printf("%x ", buf[i]);
    printf("\n");
}

int rp_lidar_check_health(int dev)
{
    send_command_raw(dev, REQUEST_GET_HEALTH, NULL, 0);

    // Read health back. Single response
    rp_descriptor_t rd;
    read_response_descriptor(dev, &rd);
    if (rd.len < 0) {
        printf("ERR: Could not read back device health descriptor\n");
        return 2;
    }

    // Next 3 bytes are information
    uint8_t buf[3];
    int res = read_fully_timeout(dev, buf, 3, TIMEOUT_MS);
    if (res < 0) {
        printf("ERR: Could not read back device health\n");
        return 2;
    }

    if (VERBOSE)
        dump(buf, 3);

    if (buf[0] != HEALTH_GOOD) {
        switch (buf[0]) {
            case HEALTH_ERROR:
                printf("Health ERROR: ");
                break;
            case HEALTH_WARN:
                printf("Health WARNING: ");
                break;
        }
        printf("%x %x\n", buf[1], buf[2]);
    }

    return buf[0];
}
