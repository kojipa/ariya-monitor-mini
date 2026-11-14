# Ariya Monitor Mini

Shows the Nissan Ariya's battery temperature, remaining energy and more on a small M5 screen.
A pocket-sized companion to the "Ariya Monitor" smartphone app that you can leave in the car.

[日本語](README.md)

## Flash from your browser

**https://kojipa.github.io/ariya-monitor-mini/**

Open the page in Chrome or Edge on a computer, connect the M5 by USB and press "書き込む" (Install).
The first time you flash an M5StickS3, hold the side power button for about 2 seconds and release it when the green LED blinks (download mode).

## What you need

| | |
|---|---|
| Device | **M5StickS3** (recommended) or M5Stack Core2 |
| Adapter | ELM327-compatible OBD2 adapter. StickS3 needs a **BLE or Wi-Fi** adapter (Bluetooth Classic-only adapters will not work). Core2 works with any type |

## Values

- Page 1: battery temperature (max / min), coolant temperature, outside temperature
- Page 2: remaining energy (kWh), SOC, power (+ charging / − driving), 12 V

Confirmed on the author's car (B6 2WD, early model). Some values may not be available on other model years or grades.

## Usage

1. Plug the adapter into the car and turn the car on
2. Start the M5. It scans for nearby BLE and Wi-Fi adapters (likely OBD adapters are listed first in green)
3. Follow the arrows on the screen edge: side button = next, front button = select
4. From then on it connects to the chosen adapter automatically

| Button | Press | Hold |
|---|---|---|
| Front | Switch page | Flip screen |
| Side | Brightness | Choose adapter again |

## Safety

Only read commands and adapter setup commands are sent to the car; anything else is blocked in the code.
Use at your own risk, and do not operate the device while driving.

## Build it yourself

Uses [PlatformIO](https://platformio.org/).

```
pio run -e sticks3 -t upload   # M5StickS3
pio run -e core2 -t upload     # M5Stack Core2
```

## License

MIT
