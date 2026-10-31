# Contributing to SELENE

This repository is an evolving robotics research prototype. Contributions, reproducible bug reports and documentation improvements are welcome if the maintainer opens collaboration.

## Development workflow

1. Create an issue describing the expected behavior and how to reproduce the problem.
2. Use a short-lived branch: `git switch -c feat/<description>` or `fix/<description>`.
3. Keep firmware changes focused. Avoid rewriting working MPU6500, ToF, Wi-Fi or motor safety code for unrelated tasks.
4. Before opening a pull request, run `python3 scripts/embed_dashboard.py` if dashboard HTML changed; run `python3 tests/check_project.py` and `pio run -e selene_v4` when possible.
5. Describe test conditions explicitly: **static check**, **bench/wheels raised**, or **measured floor test**. Do not describe simulated tests as physical validation.
6. Keep private configuration (`include/selene_config.h`), Wi-Fi passwords, keys, and data captures out of commits.

## Commit message examples

- `feat(odom): add calibrated wheel distance measurement`
- `fix(tof): recover from ranging signal failures`
- `docs(hardware): update driver wiring and supply notes`
- `test(encoders): record three-turn PPR measurements`

## Hardware change reporting

For electrical changes document module revision, pin assignments, measured voltages, photos or wiring references, and rollback steps. The diagram in `docs/diagrams/` is a **system-level drawing**, not a substitute for electrical checks.
