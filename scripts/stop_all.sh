#!/bin/bash

pkill -f drone_manage 2>/dev/null
pkill -9 -f betaflight_SITL 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ruby.*gz" 2>/dev/null
pkill -9 -f gz_camera_worker 2>/dev/null
pkill -f "XboxBridge.app|Contents/MacOS/XboxBridge" 2>/dev/null
rm -f /tmp/donkeydrone_xbox.sock 2>/dev/null
