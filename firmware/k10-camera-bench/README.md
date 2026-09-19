# K10 camera FOV/performance benchmark

This compares three camera pipelines while producing identical downstream
buffers: a 240x320 RGB565 preview and a blurred 120x160 luma vision image.

- `stock_qvga`: stock centered 480x640 sensor window, sensor 1/2 sampling.
- `wide_qvga`: centered 720x960 sensor window, sensor 1/3 sampling.
- `vga_software`: 640x480 RGB565 capture, 2x2 box reduction plus portrait rotation on CPU.

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
