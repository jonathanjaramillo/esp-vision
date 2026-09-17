// k10stream.h — UDP streaming of ORB features + accelerometer samples from a
// UNIHIKER K10 to a host, for visual(-inertial) odometry consumers.
//
// Wire format is packed/fixed-width so a host-side decoder (see
// tools/stream_recv.py) can struct.unpack it byte-for-byte — keep the two in
// sync if either changes. No display/camera dependency: takes a WiFiUDP and
// plain feature arrays.
#pragma once

#include <stdint.h>
#include <WiFiUdp.h>
#include <IPAddress.h>
#include "orb.h"

// Cap on corners per UDP packet: header(18) + n*corner(38) must stay under
// one Ethernet-MTU UDP payload (~1472 B) so it never fragments at the IP
// layer over WiFi. (18 + 34*38 = 1310.) Callers should pass corners
// strongest-first (fast_corner_detect already emits them that way) so
// truncating to the first N keeps the strongest ones, not an arbitrary
// scan-order subset.
#define K10STREAM_ORB_MAX_CORNERS 34

#pragma pack(push, 1)
typedef struct {
    char     magic[4];   // "ORBF"
    uint32_t seq;
    uint32_t t_us;        // micros() when the frame's corners were computed
    uint16_t frame_w;     // detection-frame size the (x,y) below are in
    uint16_t frame_h;
    uint16_t n;
} k10stream_orb_hdr_t;

typedef struct {
    int16_t x, y;
    int16_t angle_mrad;   // angle in milliradians, range (-3142..3142]
    uint8_t desc[ORB_DESC_BYTES];
} k10stream_orb_corner_t;

typedef struct {
    char     magic[4];   // "ACCL"
    uint32_t seq;
    uint32_t t_us;
    int16_t  ax, ay, az;
} k10stream_accel_t;
#pragma pack(pop)

// Connects WiFi STA and opens the two UDP sockets used below. Blocks up to
// timeout_ms. Non-fatal on failure — returns false and leaves WiFi however
// it ended up; callers should keep running with streaming disabled rather
// than treat this as fatal.
bool k10stream_wifi_begin(const char *ssid, const char *password,
                          WiFiUDP &udpOrb, WiFiUDP &udpAccel,
                          uint32_t timeout_ms = 15000);

// Send one frame's ORB features as a single UDP datagram to dest:port.
// Truncates to K10STREAM_ORB_MAX_CORNERS. seq is incremented by the caller
// (passed in/out) so multiple call sites could share a stream if ever needed.
// Returns true if both beginPacket/endPacket succeeded.
bool k10stream_send_orb(WiFiUDP &udp, const IPAddress &dest, uint16_t port,
                        uint32_t &seq, const orb_feature_t *feats, int n,
                        int frame_w, int frame_h, uint32_t t_us);

// Send one accelerometer sample as a single UDP datagram to dest:port.
bool k10stream_send_accel(WiFiUDP &udp, const IPAddress &dest, uint16_t port,
                          uint32_t &seq, int ax, int ay, int az, uint32_t t_us);
