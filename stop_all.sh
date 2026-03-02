#!/bin/bash

pkill -9 -f "bin/px4" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ruby.*gz" 2>/dev/null
pkill -f mavsdk_server 2>/dev/null
pkill -f drone_manage 2>/dev/null