#include "k10stream.h"
#include <WiFi.h>
#include <math.h>
#include <string.h>

bool k10stream_wifi_begin(const char *ssid, const char *password,
                          WiFiUDP &udpOrb, WiFiUDP &udpAccel,
                          uint32_t timeout_ms) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeout_ms) delay(250);
    if (WiFi.status() != WL_CONNECTED) return false;

    udpOrb.begin(0);     // 0 = ephemeral local port; we only ever send
    udpAccel.begin(0);
    return true;
}

bool k10stream_send_orb(WiFiUDP &udp, const IPAddress &dest, uint16_t port,
                        uint32_t &seq, const orb_feature_t *feats, int n,
                        int frame_w, int frame_h, uint32_t t_us) {
    if (n > K10STREAM_ORB_MAX_CORNERS) n = K10STREAM_ORB_MAX_CORNERS;

    k10stream_orb_hdr_t hdr;
    memcpy(hdr.magic, "ORBF", 4);
    hdr.seq = seq++;
    hdr.t_us = t_us;
    hdr.frame_w = (uint16_t) frame_w;
    hdr.frame_h = (uint16_t) frame_h;
    hdr.n = (uint16_t) n;

    const int began = udp.beginPacket(dest, port);
    udp.write((const uint8_t *) &hdr, sizeof(hdr));
    for (int i = 0; i < n; i++) {
        k10stream_orb_corner_t c;
        c.x = feats[i].x;
        c.y = feats[i].y;
        c.angle_mrad = (int16_t) lroundf(feats[i].angle * 1000.0f);
        memcpy(c.desc, feats[i].desc, ORB_DESC_BYTES);
        udp.write((const uint8_t *) &c, sizeof(c));
    }
    const int ended = udp.endPacket();
    return began && ended;
}

bool k10stream_send_tracks(WiFiUDP &udp, const IPAddress &dest, uint16_t port,
                           uint32_t &seq, const lkt_state_t *st,
                           int frame_w, int frame_h, uint32_t t_us) {
    // Built in one scratch buffer rather than written field-by-field like
    // k10stream_send_orb does: WiFiUDP's TX buffer is append-only, so a
    // leading header whose `n` isn't known until the tracks have been walked
    // can't be patched in place. A static buffer (not the stack — 1 KB is too
    // much for the 8KB pipeline task's frame budget alongside the rest) also
    // keeps this safe to call from a task that's already tight on stack.
    static uint8_t buf[K10STREAM_TRACK_PKT_BYTES];

    k10stream_trck_hdr_t *hdr = (k10stream_trck_hdr_t *) buf;
    memcpy(hdr->magic, "TRCK", 4);
    hdr->seq = seq++;
    hdr->t_us = t_us;
    hdr->frame_w = (uint16_t) frame_w;
    hdr->frame_h = (uint16_t) frame_h;

    int off = (int) sizeof(*hdr);
    uint16_t n = 0;
    for (int i = 0; i < LKT_MAX_TRACKS; i++) {
        const lkt_track_t *t = &st->tracks[i];
        if (!t->active) continue;
        if (n == K10STREAM_TRACK_MAX_TRACKS ||
            off + (int) sizeof(k10stream_trck_track_t) > (int) sizeof(buf)) {
            break;
        }
        k10stream_trck_track_t *p = (k10stream_trck_track_t *) (buf + off);
        p->x_q4 = (int16_t) lroundf(t->x * 16.0f);
        p->y_q4 = (int16_t) lroundf(t->y * 16.0f);
        p->id = t->id;
        p->age = t->age;
        p->score = 0;   // no per-track quality score exists in lktrack yet
        off += (int) sizeof(*p);
        n++;
    }
    hdr->n = n;

    const int began = udp.beginPacket(dest, port);
    udp.write(buf, (size_t) off);
    const int ended = udp.endPacket();
    return began && ended;
}

bool k10stream_send_accel(WiFiUDP &udp, const IPAddress &dest, uint16_t port,
                          uint32_t &seq, int ax, int ay, int az, uint32_t t_us) {
    k10stream_accel_t p;
    memcpy(p.magic, "ACCL", 4);
    p.seq = seq++;
    p.t_us = t_us;
    p.ax = (int16_t) ax;
    p.ay = (int16_t) ay;
    p.az = (int16_t) az;

    const int began = udp.beginPacket(dest, port);
    udp.write((const uint8_t *) &p, sizeof(p));
    const int ended = udp.endPacket();
    return began && ended;
}
