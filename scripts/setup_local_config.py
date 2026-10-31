#!/usr/bin/env python3
"""Create an untracked local configuration from the public safe template.

Never overwrites an existing working calibration/password.
"""
from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[1]
source = root / 'include' / 'selene_config.example.h'
target = root / 'include' / 'selene_config.h'
if target.exists():
    print(f'Already exists; preserved: {target}')
else:
    shutil.copyfile(source, target)
    print(f'Created {target}')
    print('ACTION: Set a UNIQUE Wi-Fi password and verify all local pins/voltages.')
    print('Motor tests and guarded crawl are OFF in this template.')
