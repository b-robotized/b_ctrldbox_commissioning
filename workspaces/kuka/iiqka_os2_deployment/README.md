# KUKA iiQKA.OS2 Deployment (b»controlled box)

This is the iiQKA.OS2 equivalent of `../kss_deployment/` — the same b»controlled
box RSI/EKI setup, adapted for controllers running iiQKA.OS2 instead of
classic KSS/VxWorks (no KLI interface, no WorkVisual/USB file transfer).

Source template: `kuka_external_control_sdk/krc_setup/iiqka_os2/` in
[kroshu/kuka-external-control-sdk](https://github.com/kroshu/kuka-external-control-sdk),
following `doc/iiqka_os2_setup.md`.

## What was changed vs. the upstream template

Comparing `kss_deployment/` to its own upstream template (`krc_setup/kss/`)
showed the b»controlled box customization is only ever: network config
(IP/port) + renaming the RSI context to `b_ctrldbox_rsi` — no KRL program
logic was changed. The same two changes are applied here:

1. **`Config/RobotSensorInterface/Ethernet configuration/b_ctrldbox_rsi_eth.xml`**
   — copied from `rsi_ethernet.xml`, with `IP_NUMBER` set to `10.23.23.28`
   and `PORT` to `28283` (same values as `kss_deployment`'s
   `b_ctrldbox_rsi_eth.xml`). Named `_eth` rather than `_ethernet` to match
   the `kss_deployment` naming convention, so tooling/scripts that expect
   that suffix work across both RSI versions. Element mapping (Stop,
   AK.A1-A6, DEF_RIst, DEF_AIPos, DEF_EIPos, DEF_Delay) is unchanged from
   upstream.

2. **`Program/RSI/rsi_joint_pos.dat`** — `CONTEXT_NAME[]` changed from
   `"rsi_joint_pos"` to `"b_ctrldbox_rsi"`, matching the renamed Context file
   below. `rsi_joint_pos.src` itself is untouched — it loads the context via
   the `CONTEXT_NAME[]` variable, not a hardcoded string, so no edit was
   needed there (unlike the classic KSS `.src`, which hardcodes the name in
   `RSI_CREATE(...)`).

3. **`Config/RobotSensorInterface/Context/b_ctrldbox_rsi.rsix`** — renamed
   from `rsi_joint_pos.rsix` to match `CONTEXT_NAME[]` above. This file is
   plain XML (not binary, despite the extension), and it internally
   references its ethernet config by filename in two places (`ConfigFile`
   parameter). Both were updated from `rsi_ethernet.xml` to
   `b_ctrldbox_rsi_eth.xml` to match — confirmed against the same pattern in
   the classic KSS `.rsix`, which references `b_ctrldbox_rsi_eth.xml` the
   same way. No other content was changed.

Everything else (`Config/EthernetKRL/.../b_ctrldbox_EkiKSSinterface.xml`,
all of `Program/EKIServer/`, `Program/RSI/rsi_helper.*`) is an **unmodified
copy** of the upstream `iiqka_os2` template — verified byte-identical. The
EKI config already used port `54600`, matching `kss_deployment`'s default,
so no change was needed there.

## Not yet verified

- **Whether renaming the `.rsix` file is sufficient**, or whether iiQWorks.Sim
  also needs the Context explicitly named/registered as `b_ctrldbox_rsi`
  during import (`Option packages > iiQKA.RobotSensorInterface > Context`).
  Classic KSS RSI loads contexts by filename on the controller's filesystem;
  iiQKA.OS2 imports contexts as project artifacts through iiQWorks.Sim, and
  it's not confirmed the naming works the same way. Check this when
  importing.
- **Deliberately deferred:** only the **Standard** configuration (6 axes, no
  external axis, no GPIO) is ported, matching MauRob's current needs.
  `kss_deployment` also has `ext_axis` and `gpios` variants (see its
  `RSI_CONFIGURATIONS.md`); the upstream `iiqka_os2` template has equivalents
  (`rsi_ext_axis_ethernet.xml`, `rsi_ext_axis_example.rsix`,
  `rsi_gpio_ethernet.xml`, `rsi_gpio_joint_pos.rsix`) that can be ported the
  same way once Standard is confirmed working on the real controller — don't
  port them speculatively before that.
- **Tested on a real controller** — KSS 9.2.2, RSI 6.2.1.4. See
  `../kss_deployment/RSI_CONFIGURATIONS.md`'s "Tested Combinations" table.
- `kss_deployment/deploy.bat` (raw file-copy deployment via `xcopy` to
  Windows filesystem paths like `C:\KRC\ROBOTER\...`) does **not** apply
  here — iiQKA.OS2 has no such filesystem, it deploys via iiQWorks.Sim
  project import (see Import steps below). No equivalent script is needed
  unless iiQWorks.Sim turns out to support/require its own scripted import.
- **`b_ctrldbox` subfolder not yet mirrored.** `deploy.bat` shows KSS deploys
  RSI *and* EKI program files into a dedicated subfolder,
  `C:\KRC\ROBOTER\KRC\R1\Program\b_ctrldbox\` (matching the
  `&PARAM DISKPATH = KRC:\R1\Program\b_ctrldbox` label in those `.src`
  files) — program names are still resolved globally by KRL regardless of
  subfolder, so this is purely organizational, not required for the program
  to run. The iiQKA.OS2 upstream doc (`iiqka_os2_setup.md`) just says
  "import into `KRC/R1/Program`" without specifying a subfolder. Step 4
  below currently follows the vendor doc as-is (flat, no subfolder) — check
  in iiQWorks.Sim whether you can name/create a `b_ctrldbox` subfolder on
  import, and do so for consistency with `kss_deployment` if possible.

## Import steps (per `iiqka_os2_setup.md`)

1. Import `Config/RobotSensorInterface/Context/b_ctrldbox_rsi.rsix` under
   **Option packages > iiQKA.RobotSensorInterface > Context**.
2. Import `Config/RobotSensorInterface/Ethernet configuration/b_ctrldbox_rsi_eth.xml`
   under the same option package's **Ethernet configurations**.
3. Import `Config/EthernetKRL/Ethernet configuration/b_ctrldbox_EkiKSSinterface.xml`
   under **Option packages > iiQKA.EthernetKRL > Context**.
4. Import `Program/RSI/` and `Program/EKIServer/` into `KRC/R1/Program`
   (into a `b_ctrldbox` subfolder if iiQWorks.Sim supports it, matching
   `kss_deployment` — not yet confirmed either way, see above).
5. Deploy the project onto the controller.
