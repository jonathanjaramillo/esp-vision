// k10stream.h — UDP streaming of ORB features / Lucas-Kanade tracks +
// accelerometer samples from a UNIHIKER K10 to a host, for visual(-inertial)
// odometry consumers.
//
// Wire format is packed/fixed-width so a host-side decoder (see
// tools/stream_recv.py, experiments/visual-odometry/recv_server.py) can
// struct.unpack it byte-for-byte — keep them in sync if either changes. No
// display/camera dependency: takes a WiFiUDP and plain feature/track arrays.
#pragma once

#include <stdint.h>

#ifdef K10STREAM_NO_WIFI   // host-side wire-format tests (see
                           // firmware/k10-lk-track/test/trck_wire_test.cpp) —
                           // lets the packed structs below compile on a laptop
                           // without the Arduino WiFi stack present
#define K10STREAM_WIFIUDP_DECLARED 1
class WiFiUDP;
class IPAddress;
#else
#include <WiFiUdp.h>
#include <IPAddress.h>
#endif

#include "orb.h"
#include "lktrack.h"

// Cap on corners per UDP packet: header(18) + n*corner(38) must stay under
// one Ethernet-MTU UDP payload (~1472 B) so it never fragments at the IP
// layer over WiFi. (18 + 34*38 = 1310.) Callers should pass corners
// strongest-first (fast_corner_detect already emits them that way) so
// truncating to the first N keeps the strongest ones, not an arbitrary
// scan-order subset.
#define K10STREAM_ORB_MAX_CORNERS 34

// Cap on tracks per UDP packet, same MTU reasoning as above: header(18) +
// n*track(10) must stay under ~1472 B (18 + 100*10 = 1018). Comfortably above
// LKT_MAX_TRACKS (40), so a full tracker's worth of tracks always fits in one
// datagram — the cap only exists so raising LKT_MAX_TRACKS can't silently
// start fragmenting.
#define K10STREAM_TRACK_MAX_TRACKS 100
#define K10STREAM_TRACK_PKT_BYTES  \
    (18 + K10STREAM_TRACK_MAX_TRACKS * 10)   // scratch-buffer size, see below

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

// One LK track. Position is Q4 fixed-point (level-0 pixels * 16) rather than
// plain int16 px: pyramidal LK's whole point is sub-pixel accuracy, and
// truncating to whole pixels here would throw that away on the wire.
//
// Host decoders: firmware/k10-lk-track/test/trck_wire_test.cpp (C layout) and
// experiments/visual-odometry/recv_server.py ("<4sIIHHH" / "<hhHHH") must stay
// in sync with this struct.
typedef struct {
    int16_t  x_q4, y_q4;  // level-0 px * 16, range ~+-2047 px — whole frame fits
    uint16_t id;
    uint16_t age;         // frames tracked
    uint16_t score;       // reserved — lkt_track() exposes no per-track quality
                          // score, so this is always 0 for now. Kept in the
                          // layout so adding one later doesn't change the wire
                          // format.
} k10stream_trck_track_t;

typedef struct {
    char     magic[4];   // "TRCK"
    uint32_t seq;
    uint32_t t_us;        // micros() when this frame's LK pass ran
    uint16_t frame_w;     // pyramid level-0 size the (x,y) below are in
    uint16_t frame_h;
    uint16_t n;
} k10stream_trck_hdr_t;

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

// Send every active track of `st` as a single UDP datagram to dest:port,
// taking the tracks in track-array (natural) order — unlike ORB's
// strongest-first list there is no ranking to preserve here, and truncating
// an unranked list would drop an arbitrary subset anyway. Truncates to
// K10STREAM_TRACK_MAX_TRACKS. frame_w/frame_h are the pyramid level-0 size
// the track coordinates are in. Returns true if begin/endPacket both did.
bool k10stream_send_tracks(WiFiUDP &udp, const IPAddress &dest, uint16_t port,
                           uint32_t &seq, const lkt_state_t *st,
                           int frame_w, int frame_h, uint32_t t_us);
