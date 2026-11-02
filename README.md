# FastTrack Pro driver for macOS

A user-space macOS driver for the **M-Audio FastTrack Pro** (USB `0763:2012`)
that unlocks the full hardware: no kernel extension, no DriverKit
entitlements, and SIP stays enabled.

## Goal

macOS's built-in USB audio driver runs the FastTrack Pro in a reduced mode:
16-bit only, 44.1 kHz playback, no S/PDIF input. Linux unlocks the full device.
This project brings the same to macOS.

## Results

| | Apple driver | This driver |
|---|---|---|
| Bit depth | 16-bit | **24-bit** |
| Sample rates | 44.1 kHz (playback) | **44.1 / 48 / 88.2 / 96 kHz** |
| Outputs | 4 | **4** (2 at 88.2/96 kHz) |
| Inputs | analog | **analog or S/PDIF** (selectable) |
| Volume / mute | none (no hardware controls) | **yes** (software, works with volume keys) |
| MIDI | yes | yes |

Measured on an Apple Silicon Mac, macOS 15, 48 kHz playback:

- **Clean audio:** 0 underruns, clock jitter under 1 ms.
- **CPU:** about 5.6% of one core.
- **Output latency:** about 17 ms.

At 88.2/96 kHz only outputs 1/2 work, because the USB bus has no bandwidth
for more (this matches the hardware spec).

## Install

Requires macOS 15 and the Xcode Command Line Tools.

```sh
make            # build
make install    # install and start (asks for your password)
make uninstall  # remove; the device goes back to Apple's driver
```

After installing, **FastTrack Pro** appears in Sound settings and in Audio
MIDI Setup. MIDI shows up as **FastTrack Pro MIDI**.

Logs: `/var/log/fasttrackd.log`

## How it works

```
 Core Audio apps
      │
 FastTrackPro.driver   Core Audio plug-in: the device, rates, volume/mute
      │  shared memory
 fasttrackd            background service (root): owns the USB device
      │  IOUSBHost
 FastTrack Pro ── MIDI ──► fasttrack-midi ──► CoreMIDI
```

`fasttrackd` takes the device from Apple's driver and switches it to its full
USB configuration. It then streams 24-bit audio, applying the same device
quirks as the Linux kernel (`sound/usb/quirks.c`).

## Status

- [x] Playback, 24-bit, volume/mute
- [ ] Recording through Core Audio
- [ ] 88.2/96 kHz
- [ ] MIDI
- [ ] About 3 ms of audio is lost once when a stream starts

## Development

`build/ftp-probe` is a test tool for the USB protocol. It can dump the
device descriptors (`--dump`, needs `brew install libusb`), and play test
tones and record (`sudo ftp-probe --tone 48000 --record in.wav`). Stop the
service first.
