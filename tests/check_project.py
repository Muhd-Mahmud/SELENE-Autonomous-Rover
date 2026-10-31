#!/usr/bin/env python3
"""Offline integrity checks. No dependency on an actual ESP32 or PlatformIO."""
from pathlib import Path
from html.parser import HTMLParser
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC=(ROOT/'src/main.cpp').read_text()
CONFIG=(ROOT/'include/selene_config.h').read_text()
HTML=(ROOT/'web/dashboard.html').read_text()
EMBED=(ROOT/'src/dashboard_html.h').read_text()

for expected in ["#define SELENE_ENABLE_MOTOR_TESTS 0", "#define SELENE_ENABLE_GUARDED_CRAWL 0", "#define SELENE_DRIVETRAIN_VERIFIED 0"]:
    assert expected in CONFIG, f"unsafe default: {expected}"
for expected in ['WiFi.softAPIP()', '/api/state', '/api/command', '/api/ping', 'hardStop(', 'validFront()', 'REMOTE_LEASE_MS', 'motorUntil', 'SELENE_ENABLE_MOTOR_TESTS']:
    assert expected in SRC+HTML+CONFIG, f"missing expected feature: {expected}"
assert 'SELENE_PAGE[] PROGMEM' in EMBED
assert HTML in EMBED, 'web HTML and compiled HTML differ. Run python3 scripts/embed_dashboard.py'
class PageValidator(HTMLParser):
    def __init__(self): super().__init__();self.ids=set();self.tags=[]
    def handle_starttag(self,tag,attrs):
        self.tags.append(tag)
        for k,v in attrs:
            if k=='id':self.ids.add(v)
p=PageValidator();p.feed(HTML)
for id in ['connection','distance','mpustate','tracker','left','right','arm','stop','guard','commandstatus','floorack']:
    assert id in p.ids, f"Missing element #{id}"
js=re.search(r'<script>(.*?)</script>',HTML,re.S).group(1)
js_path=ROOT/'tests'/'_test_js.js'
try:
    js_path.write_text(js)
    if subprocess.call(['node','--check',str(js_path)])!=0:
        raise AssertionError('JavaScript syntax failure')
finally:
    js_path.unlink(missing_ok=True)
print('PASS: motion defaults locked, source file/HTML integration, DOM elements, JS syntax, API commands')
