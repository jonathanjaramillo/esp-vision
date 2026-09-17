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
