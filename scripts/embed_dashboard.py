#!/usr/bin/env python3
"""Build src/dashboard_html.h from the editable web/dashboard.html file."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
html=(root/'web'/'dashboard.html').read_text(encoding='utf8')
assert ')SELENEv4"' not in html
out=root/'src'/'dashboard_html.h'
out.write_text('#pragma once\nstatic const char SELENE_PAGE[] PROGMEM = R"SELENEv4('+html+')SELENEv4";\n',encoding='utf8')
print('Generated',out,'size:',out.stat().st_size)
