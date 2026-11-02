CC       ?= clang
LIBUSB   ?= $(shell brew --prefix libusb 2>/dev/null || echo /opt/homebrew)
CFLAGS   ?= -O2 -g
CFLAGS   += -std=c11 -Wall -Wextra -Wno-unused-parameter -I$(LIBUSB)/include/libusb-1.0 -mmacosx-version-min=15.0
USBLIBS  = $(LIBUSB)/lib/libusb-1.0.a -framework IOKit -framework CoreFoundation -framework Security
B        = build

COMMON   = common/ftp_format.c common/ftp_clock.c
ENGINE   = $(COMMON) common/ftp_usb.c

.PHONY: all test clean install uninstall
DRIVER   = $(B)/FastTrackPro.driver

all: $(B)/ftp-probe $(B)/fasttrackd $(B)/fasttrack-midi $(DRIVER)

$(B):
	mkdir -p $(B)

$(B)/ftp-probe: tools/ftp-probe.c $(ENGINE) common/*.h | $(B)
	$(CC) $(CFLAGS) -o $@ tools/ftp-probe.c $(ENGINE) $(USBLIBS)

$(B)/fasttrackd: daemon/fasttrackd.c $(ENGINE) common/*.h | $(B)
	$(CC) $(CFLAGS) -o $@ daemon/fasttrackd.c $(ENGINE) $(USBLIBS)

$(B)/fasttrack-midi: midi-agent/fasttrack-midi.c common/*.h | $(B)
	$(CC) $(CFLAGS) -Wno-deprecated-declarations -o $@ midi-agent/fasttrack-midi.c -framework CoreMIDI -framework CoreFoundation

$(DRIVER): plugin/FastTrackPro.c plugin/Info.plist common/*.h | $(B)
	mkdir -p $@/Contents/MacOS
	cp plugin/Info.plist $@/Contents/Info.plist
	$(CC) $(CFLAGS) -arch arm64 -arch x86_64 -bundle -fvisibility=hidden -o $@/Contents/MacOS/FastTrackPro plugin/FastTrackPro.c \
		-framework CoreAudio -framework CoreFoundation
	codesign --force --sign - $@

install: all
	sudo ./scripts/install.sh

uninstall:
	sudo ./scripts/uninstall.sh

$(B)/test_format: tests/test_format.c $(COMMON) common/*.h | $(B)
	$(CC) $(CFLAGS) -o $@ tests/test_format.c $(COMMON)

test: $(B)/test_format
	./$(B)/test_format

clean:
	rm -rf $(B)
