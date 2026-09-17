# K10 camera pixel-format tests

This is a standalone probe for the UNIHIKER K10 GC2145 camera. It does not
initialize the display or the K10 camera helper, so the reported capture time
and framebuffer layout are isolated from LVGL and the LCD flush.

The two PlatformIO environments test:

- `gray`: `PIXFORMAT_GRAYSCALE`, expected one byte per pixel
- `yuv`: `PIXFORMAT_YUV422`, expected two bytes per pixel

Run from this directory:

```sh
pio run -e gray -t upload
pio device monitor -e gray

# Then reflash the other format:
pio run -e yuv -t upload
pio device monitor -e yuv
```

The serial output reports the actual framebuffer `format`, dimensions, byte
length, inter-frame timing, first bytes, and simple byte-lane statistics. A
successful direct grayscale result should let the vision pipeline consume
`fb->buf` as luma without the RGB565 conversion. YUV422 may still require
extracting its luma byte lane, depending on the sensor's byte ordering.

On the bundled K10 camera driver, the grayscale test currently reaches the
GC2145 but logs `Requested format is not supported` and then hits an
integer-divide-by-zero panic in the driver. This is a driver failure path, not
a problem with the test; use the YUV environment after testing grayscale.
