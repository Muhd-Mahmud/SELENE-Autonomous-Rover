#!/usr/bin/env python3
"""Read a single JSON telemetry snapshot from Selene's Wi-Fi AP.
Standard library only. Run while connected to SELENE_ROVER.
"""
import json
from urllib.request import urlopen
from urllib.error import URLError

url = 'http://192.168.4.1/api/state'
try:
    with urlopen(url, timeout=5) as response:
        state = json.load(response)
except (URLError, ValueError, TimeoutError) as exc:
    raise SystemExit(f'Cannot read Selene telemetry at {url}: {exc}')
for key in (
    'firmware', 'mode', 'estop', 'wifi_clients', 'imu_model', 'imu_ok',
    'tof_present', 'tof_ok', 'tof_mm', 'tof_status', 'enc_l', 'enc_r',
    'wheel_diameter_mm', 'track_width_mm', 'encoder_ppr',
    'odom_configured', 'calibrated', 'geometry_verified',
    'odom_quality', 'odom_x_mm', 'odom_y_mm', 'odom_theta_rad',
):
    print(f'{key:24s} {state.get(key, "not provided")}')
