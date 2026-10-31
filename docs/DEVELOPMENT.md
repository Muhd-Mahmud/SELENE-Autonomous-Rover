# Development and release workflow

[← Back to README](../README.md)

## One-time setup

```bash
python3 scripts/setup_local_config.py
# Edit include/selene_config.h with your LOCAL unique password and measured values.
python3 scripts/embed_dashboard.py
pio run -e selene_v4
```

The local configuration header is ignored by Git on purpose. Edit the tracked `include/selene_config.example.h` only when publishing a **safe, general configuration change**, not real credentials.

## Building and uploading

```bash
pio run -e selene_v4
pio run -e selene_v4 -t upload
pio device monitor -b 115200
```

On the present Linux setup the USB-UART bridge is CH343, often exposed as `/dev/ttyACM0`. Verify your actual port with `pio device list`.

## Live browser and telemetry

Connect to the `SELENE_ROVER` AP, then open `http://192.168.4.1/`.

```bash
python3 scripts/telemetry_snapshot.py
curl -s http://192.168.4.1/api/state | python3 -m json.tool
```

## Updating the dashboard

```bash
# edit web/dashboard.html
python3 scripts/embed_dashboard.py
git add web/dashboard.html src/dashboard_html.h
```

## Update cycle

```bash
git status
git switch -c feat/your-short-task
# make your changes; test and document
git add .
git commit -m "feat: describe the verified change"
git push -u origin feat/your-short-task
```

Open a pull request, review it, then merge into `main`. For small solo updates, commits directly to `main` also work once the working tree is clean.

## Before a release

- [ ] Firmware builds and core behavior is checked on physical hardware.
- [ ] Browser and serial output reflect actual firmware version.
- [ ] `docs/CHANGELOG.md` and `docs/ROADMAP.md` reflect the work.
- [ ] No private Wi-Fi password or tokens appear in Git diff.
- [ ] Safety checks are not accidentally bypassed.
