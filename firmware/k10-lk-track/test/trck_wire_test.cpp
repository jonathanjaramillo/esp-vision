// Static test of the TRCK wire format: builds a k10stream_trck_hdr_t +
// k10stream_trck_track_t[] the same way k10stream_send_tracks() lays them out
// in its scratch buffer, and checks the byte layout a host receiver
// (experiments/visual-odometry/recv_server.py) struct.unpacks.
//
//   c++ -O2 -std=c++17 -Wall -I ../../lib/lktrack -I ../../lib/orb \
//       -I ../../lib/k10stream test/trck_wire_test.cpp -o /tmp/trckt && /tmp/trckt
#define K10STREAM_NO_WIFI 1
#include "k10stream.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

int main() {
    printf("sizeof hdr  = %zu\n", sizeof(k10stream_trck_hdr_t));
    printf("sizeof track= %zu\n", sizeof(k10stream_trck_track_t));
    assert(sizeof(k10stream_trck_hdr_t) == 18);
    assert(sizeof(k10stream_trck_track_t) == 10);
    assert(K10STREAM_TRACK_PKT_BYTES == 18 + 100 * 10);
    // LKT_MAX_TRACKS worth of tracks must fit in one MTU-safe packet, else the
    // cap silently starts dropping tracks on real footage.
    assert(18 + LKT_MAX_TRACKS * 10 <= 1472);

    unsigned char buf[K10STREAM_TRACK_PKT_BYTES];
    memset(buf, 0, sizeof(buf));
    k10stream_trck_hdr_t *hdr = (k10stream_trck_hdr_t *) buf;
    memcpy(hdr->magic, "TRCK", 4);
    hdr->seq = 0x11223344;
    hdr->t_us = 0x55667788;
    hdr->frame_w = 120;
    hdr->frame_h = 160;
    hdr->n = 2;
    k10stream_trck_track_t *t0 = (k10stream_trck_track_t *) (buf + 18);
    t0->x_q4 = (short) (3.25f * 16.0f);   // 52 -> 3.25 px
    t0->y_q4 = (short) (7.0f * 16.0f);
    t0->id = 7;
    t0->age = 42;
    t0->score = 0;

    // Little-endian, packed: <4sIIHHH and <hhHHH
    assert(memcmp(buf, "TRCK", 4) == 0);
    unsigned int seq, tus;
    unsigned short fw, fh, n;
    memcpy(&seq, buf + 4, 4); memcpy(&tus, buf + 8, 4);
    memcpy(&fw, buf + 12, 2); memcpy(&fh, buf + 14, 2); memcpy(&n, buf + 16, 2);
    assert(seq == 0x11223344u && tus == 0x55667788u);
    assert(fw == 120 && fh == 160 && n == 2);
    short xq, yq; unsigned short id, age, sc;
    memcpy(&xq, buf + 18, 2); memcpy(&yq, buf + 20, 2);
    memcpy(&id, buf + 22, 2); memcpy(&age, buf + 24, 2); memcpy(&sc, buf + 26, 2);
    assert(xq == 52 && yq == 112 && id == 7 && age == 42 && sc == 0);
    printf("TRCK layout OK (hdr 18 B, track 10 B, Q4 round-trips 3.25px)\n");
    return 0;
}
