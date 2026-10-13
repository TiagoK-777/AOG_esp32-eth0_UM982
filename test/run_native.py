#!/usr/bin/env python3
"""Regressões locais com hardware simulado e ASan/UBSan; não testa o escalonador do ESP32."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
kalman = root / '.pio/libdeps/wt32-eth01/SimpleKalmanFilter/src'
if not kalman.is_dir():
    raise SystemExit('Compile primeiro: pio run -e wt32-eth01')
sources = [
    'test/native/test_main.cpp', 'src/autosteer/pid.cpp', 'src/autosteer/safety.cpp',
    'src/autosteer/autosteer_task.cpp', 'src/config/settings.cpp', 'src/gps/nmea_handlers.cpp',
    'src/gps/fusion.cpp', 'src/gps/panda.cpp', 'src/gps/gps_task.cpp', 'src/imu/rvc.cpp', 'src/network/pgn_protocol.cpp',
    'src/network/rtcm_buffer.cpp', 'src/hardware/motor_pwm.cpp', 'src/hardware/ads1115.cpp',
    'src/state/guidance_state.cpp', 'src/state/steer_state.cpp', 'src/state/imu_state.cpp', 'src/state/pending.cpp',
    'src/persist/preferences.cpp',
]
with tempfile.TemporaryDirectory(prefix='aog-native-') as directory:
    binary = Path(directory) / 'regression'
    environment = dict(os.environ, ASAN_OPTIONS='detect_leaks=0')
    for name, flags in [('PANDA', []), ('PAOGI', ['-DTEST_PAOGI']), ('dual/IMU', ['-DTEST_DUAL'])]:
        subprocess.run([
            'g++', '-std=c++17', '-pthread', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            *flags, '-I', str(root / 'test/native/stubs'), '-I', str(root / 'src'), '-I', str(kalman),
            *(str(root / path) for path in sources), str(kalman / 'SimpleKalmanFilter.cpp'),
            '-o', str(binary),
        ], check=True)
        print(name, flush=True)
        subprocess.run([str(binary)], check=True, env=environment)
