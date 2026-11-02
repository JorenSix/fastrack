# FastTrack Pro user-space driver for macOS

An unofficial, full-featured macOS driver for the **M-Audio FastTrack Pro**
(USB `0763:2012`) that runs entirely in user space: no kernel extension, no
DriverKit entitlements, and SIP stays enabled.

## Why

macOS drives the FastTrack Pro with Apple's generic `AppleUSBAudio` class
driver. That driver leaves the device in **USB configuration 1**, which only
offers:

- 16-bit samples
- 44.1 kHz playback
- no S/PDIF input and no 88.2/96 kHz

The hardware's full feature set is in **configuration 2**: 24-bit at
44.1/48/88.2/96 kHz, the S/PDIF input, and four outputs. The Linux kernel
(`snd-usb-audio`) switches to configuration 2 and handles a few device
quirks. This project does the same on macOS.

## The idea: what Linux does, ported to macOS

Linux has no special device table entry for the FastTrack Pro. It treats it as
a standard USB Audio Class 1 device plus four quirks in `sound/usb/quirks.c`:

| Linux quirk | What it does | Here |
|---|---|---|
| `snd_usb_fasttrackpro_boot_quirk` | Switches the device from configuration 1 to 2 | `libusb_set_configuration(2)` after taking the device from AppleUSBAudio |
| `fasttrackpro_skip_setting_quirk` | Picks the 24-bit alt settings (2 for ≤48 kHz, 3 for ≤96 kHz) | Same choice; see the bandwidth note below |
| `is_big_endian_format` | 24-bit alt settings carry **3-byte big-endian** samples | `ftp_pack_s24be` / `ftp_unpack_s24be` (confirmed on hardware) |
| `snd_usb_audioformat_attributes_quirk` | Treats playback as adaptive and ignores the feedback endpoints | Adaptive OUT stream with exact per-millisecond packet sizes |

The sample rate is set like Linux's `set_sample_rate_v1`: UAC1 `SET_CUR
SAMPLING_FREQ` on the OUT endpoints 0x03 and 0x04.

**Bandwidth note.** On full-speed USB, alt setting 3 (584-byte packets) on all
three streams needs about 1750 bytes per 1 ms frame. That does not fit, and it
showed up as a sawtooth/noise sound. So this driver uses:

- 44.1/48 kHz: alt 2, giving **4 outputs + 2 inputs** at 24-bit
- 88.2/96 kHz: alt 3, giving **outputs 1/2 + 2 inputs** at 24-bit, which matches the hardware spec

**Volume.** The device has **no Feature Units**, so there is no hardware volume
or mute (Linux also shows "Controls: 0"). The driver adds Core Audio volume
and mute controls that work in software, so the volume keys and Audio MIDI
Setup sliders work.

## Architecture

```
 Core Audio apps
      │
 coreaudiod ── FastTrackPro.driver   (AudioServerPlugIn, sandboxed, no USB access)
      │   XPC "be.jsix.fasttrackd" for control + shared-memory ring buffers for audio
 fasttrackd  (root LaunchDaemon)
      │   libusb: capture the device from AppleUSBAudio, SET_CONFIGURATION 2,
      │   alt settings, sample rate, isochronous audio, bulk USB-MIDI
 FastTrack Pro ──────────── MIDI ──────────► fasttrack-midi (LaunchAgent) ─► CoreMIDI
```

Why not AudioDriverKit? Apple's
[DriverKit route](https://developer.apple.com/documentation/audiodriverkit/creating-an-audio-device-driver)
needs full Xcode, a paid developer account for the USB transport entitlement,
and a host app that activates a system extension. A HAL plug-in plus a root
helper builds with only the Command Line Tools and runs with SIP enabled.

Timing:

- The daemon's USB engine is the master clock.
- A delay-locked loop (`common/ftp_clock.c`) smooths the USB completion times
  into a clean (sample time, host time) line, shared through a seqlock.
- The plug-in derives its zero time stamps from that line.

## Layout

| Path | What |
|---|---|
| `common/ftp_usb.c` | libusb engine: capture, config 2, isochronous IN/OUT, MIDI, clock DLL feed |
| `common/ftp_format.c` | S24_3BE ↔ float, adaptive packet scheduler |
| `common/ftp_clock.c` | DLL time filter |
| `common/ftp_shm.h`, `ftp_proto.h` | Shared-memory layout and XPC protocol |
| `daemon/fasttrackd.c` | Root daemon: device hot-plug, streaming, XPC server |
| `plugin/FastTrackPro.c` | AudioServerPlugIn: 4-out/2-in device, rates, volume/mute, input source (analog / S/PDIF) |
| `midi-agent/fasttrack-midi.c` | CoreMIDI virtual port ↔ USB-MIDI packets |
| `tools/ftp-probe.c` | Protocol test tool: descriptor dump, test tones, recording |
| `tests/` | Unit tests for packing, scheduling and the DLL |

## Build & use

Requirements: macOS 15, Xcode Command Line Tools, `brew install libusb`.

```sh
make            # builds everything
make test       # unit tests
build/ftp-probe --dump                                    # descriptors (no root needed)
sudo build/ftp-probe --tone 48000 -s 5 --record in.wav    # test tones on outs 1-4, record input
make install    # installs daemon, plug-in, MIDI agent; restarts coreaudiod
make uninstall  # removes everything; AppleUSBAudio takes the device back
```

Logs:

```sh
tail -f /var/log/fasttrackd.log
log stream --predicate 'subsystem == "be.jsix.fasttrack"'
```

## Status

- [x] Protocol checked on real hardware with `ftp-probe`: config 2, 24-bit
      big-endian, clean tones on outputs 1–4 at 48 kHz, input capture, clock at
      48000 Hz with no xruns
- [ ] Daemon + HAL plug-in tested end-to-end (written, not yet tested)
- [ ] 88.2/96 kHz checked on hardware
- [ ] MIDI tested
- [ ] Long-run drift and dropout test

## Known limitations

- At 88.2/96 kHz only outputs 1/2 work (USB bandwidth).
- Volume and mute are in software; the front-panel gain knobs stay analog.
- Output latency is about 17 ms (8 × 2 ms USB transfer queue). It can be tuned
  with the `FTP_XFERS` / `FTP_PKTS` environment variables of `fasttrackd`.
- DTS/AC3 pass-through (interface 3, alt 7) is not supported.
- The daemon accepts XPC connections from any local process.

## References

- Linux `sound/usb/quirks.c`: [commit 0f5733b0](https://github.com/torvalds/linux/commit/0f5733b0c883158b13366ae34b5e4bd52a1ac346)
- [l3ku/ubuntu-fast-track-pro](https://github.com/l3ku/ubuntu-fast-track-pro)
- Apple: [AudioServerPlugIn.h](https://developer.apple.com/documentation/coreaudio/audioserverplugin), [Creating an audio device driver](https://developer.apple.com/documentation/audiodriverkit/creating-an-audio-device-driver)
- F. Adriaensen, *Using a DLL to filter time* (2005)
