# K10 camera FOV/performance benchmark

This compares three camera pipelines while producing identical downstream
buffers: a 240x320 RGB565 preview and a blurred 120x160 luma vision image.

- `stock_qvga`: stock centered 480x640 sensor window, sensor 1/2 sampling.
- `wide_qvga`: centered 720x960 sensor window, sensor 1/3 sampling.
- `vga_software`: 640x480 RGB565 capture, 2x2 box reduction plus portrait rotation on CPU.
- `sensor_1_4`: centered 1280x960 window, sensor 1/4 sampling to 320x240,
  then a QVGA-cost portrait rotation.
- `sensor_full_1_5`: full 1600x1200 sensor, sensor 1/5 sampling to 320x240,
  then a QVGA-cost portrait rotation.

The bundled camera binary has an enum/configuration quirk: requesting
`FRAMESIZE_VGA` retains a QVGA-sized DMA buffer. The `vga_software` environment
requests `FRAMESIZE_SVGA`, which reliably produces the actual 640x480 frame,
then rotates and box-reduces it to the portrait output.

The LCD is deliberately disabled so its common SPI cost does not hide camera
or preprocessing differences. The benchmark processes 20 warm-up and 200
measured frames and reports mean stage timings, end-to-end FPS, CPU preprocessing
share, bad geometry/frame counts, and memory headroom.

Run one mode:

```sh
pio run -e wide_qvga -t upload --upload-port /dev/cu.usbmodem1101
pio device monitor -e wide_qvga --port /dev/cu.usbmodem1101
```

Run all modes automatically:

```sh
python3 tools/run_all.py --pio ~/.platformio/venv/bin/pio \
  --port /dev/cu.usbmodem1101 | tee results.txt
```

See `RESULTS.md` for measurements from the connected K10.

## GC2145 package override

The framework provides `libesp32-camera.a` but not its driver source. The prebuild
script makes a project-local copy, renames the packaged `gc2145_init`, and links
`src/gc2145_wide.cpp` as a wrapper around the original driver. The shared
PlatformIO framework is never modified. Modes 1/4 and 1/5 therefore exercise
the package's previously disabled GC2145 subsampling selectors through the real
camera initialization path rather than patching registers later in application
code.

Select the sensor FOV independently with a compiler flag:

| `GC2145_FOV_MODE` | Sensor input | Sampling | Output |
|---:|---:|---:|---:|
| `0` | 480x640 centered | 1/2 | 240x320 portrait |
| `1` | 720x960 centered | 1/3 | 240x320 portrait |
| `2` | 1280x960 centered | 1/4 | 320x240 landscape |
| `3` | 1600x1200 full sensor | 1/5 | 320x240 landscape |

For example, add `-DGC2145_FOV_MODE=3` to an environment's `build_flags`.
Modes 2 and 3 retain the QVGA byte count but need a 90-degree rotation before
displaying on the K10's portrait LCD. `BENCH_MODE` is separate and only selects
the benchmark's matching preprocessing path.
