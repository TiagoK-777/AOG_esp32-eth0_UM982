#!/usr/bin/env python3
"""Regressões locais com hardware simulado e ASan/UBSan; não testa o escalonador do ESP32."""
from pathlib import Path
import argparse
import itertools
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
arguments = argparse.ArgumentParser(description=__doc__)
arguments.add_argument('--suite', choices=['all', 'core', 'navigation'], default='all')
suite = arguments.parse_args().suite
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
    environment = dict(os.environ, ASAN_OPTIONS='detect_leaks=0',
                       UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    def compile_binary(inputs, flags=()):
        subprocess.run([
            'g++', '-std=c++17', '-pthread', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
            *flags, '-I', str(root / 'test/native/stubs'), '-I', str(root / 'src'), '-I', str(kalman),
            *(str(root / path) for path in inputs), str(kalman / 'SimpleKalmanFilter.cpp'),
            '-o', str(binary),
        ], check=True)
    if suite in ('all', 'core'):
        compile_binary([
            'test/native/test_network.cpp', 'src/network/udp_rx.cpp', 'src/network/pgn_protocol.cpp',
            'src/network/rtcm_buffer.cpp', 'src/config/settings.cpp',
            'src/state/guidance_state.cpp', 'src/state/steer_state.cpp', 'src/state/imu_state.cpp',
            'src/state/pending.cpp',
        ])
        subprocess.run([str(binary)], check=True, env=environment, timeout=30)
        for name, flags in [('PANDA', []), ('PAOGI', ['-DTEST_PAOGI']), ('dual/IMU', ['-DTEST_DUAL'])]:
            compile_binary(sources, flags)
            print(name, flush=True)
            subprocess.run([str(binary)], check=True, env=environment, timeout=30)
    if suite in ('all', 'navigation'):
        compile_binary([
            'test/native/test_navigation.cpp', 'src/config/settings.cpp',
            'src/gps/nmea_handlers.cpp', 'src/gps/fusion.cpp', 'src/gps/panda.cpp',
            'src/gps/gps_task.cpp', 'src/imu/rvc.cpp', 'src/state/imu_state.cpp',
            'src/network/rtcm_buffer.cpp',
        ])
        failures = []
        profiles = list(itertools.product(range(3), (0, 1), (0, 900), (0, 1), (0, 1)))
        for mode, ogi, mount, yaw_rate, invert_roll in profiles:
            name = (f'{("single+IMU", "dual", "dual+IMU")[mode]} '
                    f'{"PAOGI" if ogi else "PANDA"} montagem={mount} '
                    f'yawRate={yaw_rate} invertRoll={invert_roll}')
            settings = dict(environment, NAV_MODE=str(mode), NAV_OGI=str(ogi),
                            NAV_MOUNT=str(mount), NAV_YAW_RATE=str(yaw_rate),
                            NAV_INVERT_ROLL=str(invert_roll))
            try:
                result = subprocess.run([str(binary)], env=settings, capture_output=True, text=True, timeout=30)
            except subprocess.TimeoutExpired:
                failures.append(name)
                print(f'FAIL: {name}: excedeu 30 segundos', flush=True)
                continue
            if result.returncode:
                failures.append(name)
                print(f'FAIL: {name}\n{result.stdout}{result.stderr}', flush=True)
        print(f'Navegação: {len(profiles) - len(failures)}/{len(profiles)} perfis aprovados', flush=True)
        if failures:
            raise SystemExit(1)
