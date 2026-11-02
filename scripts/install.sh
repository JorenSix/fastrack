#!/bin/bash
# Install the FastTrack Pro user-space driver. Run via `make install`.
set -euo pipefail
[ "$EUID" -eq 0 ] || exec sudo "$0" "$@"

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
B="$ROOT/build"
APP="/Library/Application Support/FastTrackPro"
HAL="/Library/Audio/Plug-Ins/HAL"
CONSOLE_USER="$(stat -f %Su /dev/console)"
CONSOLE_UID="$(id -u "$CONSOLE_USER")"

for f in fasttrackd fasttrack-midi FastTrackPro.driver; do
  [ -e "$B/$f" ] || { echo "missing $B/$f - run make first" >&2; exit 1; }
done

echo "Stopping running instances..."
launchctl bootout "gui/$CONSOLE_UID/be.jsix.fasttrack-midi" 2>/dev/null || true
launchctl bootout system/be.jsix.fasttrackd 2>/dev/null || true
sleep 1

echo "Installing binaries to $APP"
mkdir -p "$APP"
install -m 755 -o root -g wheel "$B/fasttrackd" "$B/fasttrack-midi" "$APP/"
codesign --force --sign - "$APP/fasttrackd" "$APP/fasttrack-midi"

echo "Installing Core Audio plug-in to $HAL"
rm -rf "$HAL/FastTrackPro.driver"
cp -R "$B/FastTrackPro.driver" "$HAL/"
chown -R root:wheel "$HAL/FastTrackPro.driver"

echo "Installing launchd jobs"
install -m 644 -o root -g wheel "$ROOT/launchd/be.jsix.fasttrackd.plist" /Library/LaunchDaemons/
install -m 644 -o root -g wheel "$ROOT/launchd/be.jsix.fasttrack-midi.plist" /Library/LaunchAgents/
launchctl bootstrap system /Library/LaunchDaemons/be.jsix.fasttrackd.plist
launchctl bootstrap "gui/$CONSOLE_UID" /Library/LaunchAgents/be.jsix.fasttrack-midi.plist || true

echo "Restarting coreaudiod"
killall coreaudiod 2>/dev/null || true
echo "Done. Log: /var/log/fasttrackd.log"
