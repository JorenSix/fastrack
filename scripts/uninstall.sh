#!/bin/bash
# Remove the FastTrack Pro user-space driver; macOS' AppleUSBAudio takes the device back.
set -uo pipefail
[ "$EUID" -eq 0 ] || exec sudo "$0" "$@"
CONSOLE_UID="$(id -u "$(stat -f %Su /dev/console)")"

launchctl bootout "gui/$CONSOLE_UID/be.jsix.fasttrack-midi" 2>/dev/null
launchctl bootout system/be.jsix.fasttrackd 2>/dev/null
rm -f /Library/LaunchDaemons/be.jsix.fasttrackd.plist /Library/LaunchAgents/be.jsix.fasttrack-midi.plist
rm -rf "/Library/Audio/Plug-Ins/HAL/FastTrackPro.driver" "/Library/Application Support/FastTrackPro"
killall coreaudiod 2>/dev/null
echo "Uninstalled. Replug the FastTrack Pro if it does not reappear as a standard device."
