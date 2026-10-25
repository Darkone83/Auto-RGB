# Auto-RGB

<div align=center>

<img src="https://github.com/Darkone83/Auto-RGB/blob/main/img/Logo.png" width=400><img src="https://github.com/Darkone83/Auto-RGB/blob/main/img/Darkone83.png" width=400>

</div>

**Auto-RGB** is an open-source, eight-channel addressable RGB LED controller for automotive and other custom lighting projects. It uses an ESP32-S3 and provides local configuration without requiring a cloud service.

## Features

- Eight individually configurable addressable RGB outputs
- LED effects, zones, scenes, and brightness controls
- Built-in local Wi-Fi configuration interface
- Optional OLED display with rotary encoder
- Optional ambient-light sensor (ALS), real-time clock (RTC), and expansion hub
- Configuration saved on the controller

## LED requirements

Use **5 V individually addressable, WS2812B-compatible RGB LED strips**. Compatible 5 V SK6812 RGB strips may also work. Standard analog RGB strips and two-wire clocked LEDs are not supported by the current hardware.

**Suggested LEDs:** [ADRESUNO WS2812B addressable LED strip](https://www.amazon.com/dp/B09PBHDXSM)

Each LED run requires:

- **5 V** — regulated LED power
- **GND** — common ground shared with the controller
- **DATA IN** — data from the selected channel, connected in the correct direction

Set the actual **LED count for each connected channel** in the configuration interface. Unused channels can remain disconnected.

> **Important:** The controller's USB-A-style LED connectors have a **custom LED pinout**; they are **not USB ports**. Do not connect USB accessories, USB chargers, or computers to these outputs. Follow the PCB schematic and adapter wiring when making LED cables.

## Power

The controller and supported 5 V LED strips require a **regulated 5 V supply**. Do not connect vehicle 12 V directly to the controller's 5 V LED rail.

**Example 12 V-to-5 V converter:** [Aideepen 5 V / 3 A dual-USB DC-DC converter](https://www.amazon.com/dp/B09TFN79PG). The **3 A / 15 W rating is for the converter as a whole**, not for each output. Use a larger or separate properly rated supply when the lighting load exceeds that limit.

As a conservative sizing guide, allow up to **60 mA per WS2812B RGB LED** at maximum full-white output:

`Estimated maximum LED current (A) = number of LEDs × 0.06`

Size supplies, wiring, connectors, and fuses for the expected load. Use power injection for longer runs where voltage drop becomes an issue. If using multiple 5 V power branches, share the required signal ground but do not connect the outputs of independent regulators together. For automotive use, choose a converter and input protection suitable for vehicle voltage transients.

## Setup

1. Disconnect power and wire each compatible LED strip to its channel, checking **5 V, GND, and DATA IN**.
2. Connect a fused, regulated 5 V power source of adequate capacity to the designated power input.
3. Power on Auto-RGB and open its built-in local Wi-Fi configuration interface.
4. Enter the LED count for each channel, then configure your zones, scenes, and brightness.
5. Test at low brightness and **save** the configuration when everything works correctly.

## Optional modules and parts

The expansion modules are optional; the main controller works without them.

- **OLED remote display:** [ELEGOO 0.96-inch SSD1306 OLED module (3-pack)](https://www.amazon.com/dp/B0FSRQG23K) — the display used with the printable remote enclosure. Check the module's actual board dimensions against the enclosure model before printing.
- **RTC backup battery holder:** [Coin-cell holder](https://www.aliexpress.us/item/2255801053879832.html) — check the footprint, contacts, and polarity against the RTC PCB before assembly.
- **Ambient-light sensor and expansion hub:** See the hardware files for the corresponding board designs and connections.

Printable enclosure STLs and editable models can be included with the respective hardware files.

## Build firmware

The firmware targets the **ESP32-S3** using **ESP-IDF 5.5.5**.

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p YOUR_SERIAL_PORT flash monitor
```

Use the project's included partition configuration when flashing. Building and installation instructions may vary with the hardware revision.

## Hardware and safety

Refer to the included schematics, PCB layouts, and connector pinouts for your exact board revision. Check polarity and wiring before applying power. Fuse vehicle power feeds appropriately and avoid installing wiring near moving parts, hot surfaces, or vehicle safety systems.

This is a DIY electronics project; verify electrical and mechanical suitability for your application before installation.
