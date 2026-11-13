# FZI Reference Driver (vendored)

This folder contains an unmodified copy of the KUKA KRL side of
[fzi-forschungszentrum-informatik/kuka_rsi_driver](https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver)
(`kuka_rsi_driver/krl/`, `main` branch), kept here for side-by-side comparison
and deployment against the b_ctrldbox driver on the same controller.

License: BSD-3-Clause, Copyright 2024 FZI Forschungszentrum Informatik (see
license headers in each file, or the upstream `LICENSE` file).

## Layout

| Upstream path | Vendored here | Notes |
|---|---|---|
| `krl/KR_C4/kuka_rsi_driver.src` | `KRC/R1/Program/FZIdriver/KR_C4/kuka_rsi_driver.src` | RSI-only motion program, KRC4 controllers |
| `krl/KR_C4/ros_rsi.rsi(.diagram/.xml)` | `Config/User/Common/SensorInterface/KR_C4/` | Split RSIVisual project (KSS < 8.6 packaging) |
| `krl/KR_C5/kuka_rsi_driver.src` | `KRC/R1/Program/FZIdriver/KR_C5/kuka_rsi_driver.src` | RSI-only motion program, KRC5 controllers |
| `krl/KR_C5/ros_rsi.rsix` | `Config/User/Common/SensorInterface/KR_C5/` | Packed RSIVisual project (KSS >= 8.6 packaging) |
| `krl/Common/ros_rsi_ethernet.xml` | Duplicated into both `KR_C4/` and `KR_C5/` config folders | Same file both variants reference; duplicated so `deploy.bat` can `xcopy` a single source folder per variant |

There is no ext_axis/GPIO equivalent upstream — this driver only supports the
6-axis standard configuration.

## Deploying

Use `deploy.bat` in the parent folder and choose "FZI reference driver" at
the driver prompt, then pick KR C4 or KR C5. It deploys to
`C:\KRC\ROBOTER\KRC\R1\Program\fzi_rsi_driver` (kept separate from
`b_ctrldbox`'s program folder) and cleans up any stale RSI config files from
a previously deployed driver/version/variant in the shared SensorInterface
folder first — see `RSI_CONFIGURATIONS.md`.
