# KUKA with b»Controlled Box - Getting Started

This guide walks you from an unconfigured KUKA controller (classic KSS 8.x) to a moving robot,
starting with the Standard configuration and then extending it with an external axis or GPIOs.
All configurations stream joint positions, torques, motor currents, setpoint positions, program
status and the Cartesian pose to ROS 2.

> ⚠️ The configurations are tested with **RSI 4.1.x (KSS 8.6)** only. The RSI 3.3.x/4.0.x/6.x
> contexts in this repository don't match the current ethernet configs yet - see
> [`RSI_CONFIGURATIONS.md`](kss_deployment/RSI_CONFIGURATIONS.md).

- Controller network setup (RSI and EKI interfaces): [`docs/supported_robots/KUKA.md`](../../docs/supported_robots/KUKA.md)
- Reference for every RSI configuration (indices, objects, tested versions): [`kss_deployment/RSI_CONFIGURATIONS.md`](kss_deployment/RSI_CONFIGURATIONS.md)
- KSS 9.2.2 / iiQKA.OS2 uses a different import procedure, see "iiQKA.OS2 (RSI 6.x) Deployment" in `RSI_CONFIGURATIONS.md`.

---

## 1. The files involved

RSI exchanges one XML telegram per cycle (4 ms or 12 ms) between the robot controller and the
driver. Which values the telegram contains is defined in up to five places. Their names must
match each other:

| File | Where it lives | Read by | Defines |
| :--- | :------------- | :------ | :------ |
| **Ethernet config** `b_ctrldbox_rsi_eth.xml` | robot controller, `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` (source: `kss_deployment/.../SensorInterface/common/<config>/`) | RSI on the controller | IP/port of the driver and the **telegram layout**: which XML tags are sent (`SEND`) and received (`RECEIVE`), and the Ethernet port (`INDX`) each tag is connected to |
| **RSI context** `b_ctrldbox_rsi.rsix` | robot controller, same folder (source: `kss_deployment/.../SensorInterface/rsi_<version>/<config>/`) | RSI on the controller | the **signal flow** on the controller: RSI objects (joint corrections, stop, I/O, torque, ...) and which Ethernet port each object reads from or writes to. Tags with `INDX="INTERNAL"` (`DEF_*`) are built-ins and need no wiring |
| **RSI program** `rsi_joint_pos_4ms.src` | robot controller, `KRC:\R1\Program\b_ctrldbox\` (source: `kss_deployment/KRC/R1/Program/RSI_kss/`) | KRL interpreter | loads the context (`RSI_CREATE("b_ctrldbox_rsi", ...)`) and starts the sensor-guided motion (`RSI_MOVECORR()`) |
| **Driver YAML** | ROS side, `workspaces/kuka/rsi_xml_config/<config>.yaml` | `kuka_rsi_driver` (`rsi_xml_config_file:=...`) | the telegram layout as seen by the driver, and which ROS interface each value goes to. **Required for all configurations** - it is also the source the ethernet config is generated from (section 5) |
| **Robot description** (URDF, `ros2_control` part) | ROS side, `kuka_robot_descriptions` + `kuka_rsi_driver/config/gpio_config.xacro` | `ros2_control` | the state/command interfaces the driver exports (joints, GPIOs, sensors) |

How the files work together, for one value:

```
controller                                                         ROS side
DigIn $IN[132] --(.rsix)--> Ethernet port 7 --(eth.xml: INDX="7")--> <GPIO input_01="1"/> --(YAML)--> gpio/input_01 state interface (URDF)
```

The ethernet config, the context and the driver must always describe the **same** layout -
always deploy the `.rsix` and the ethernet config from the same configuration, and use the
matching driver settings.

---

## 2. Basic example: Standard configuration

The Standard configuration streams the six joint positions, torques, motor currents and setpoint
positions, the program status and the Cartesian pose to the driver, and receives the six joint
corrections back.

### 2.1 Prepare the controller

Set up the RSI network interface (and the EKI interface if you use `eki_rsi`) as described in
[`KUKA.md`](../../docs/supported_robots/KUKA.md), sections 1 and 5. Default addresses:

| | IP |
| :- | :- |
| Driver (ctrlX CORE / PC) RSI interface | `10.23.23.28`, UDP port `28283` |
| Robot controller RSI interface | `10.23.23.201` |
| Robot controller KLI (EKI) | `10.28.23.240`, TCP port `54600` |

To use other addresses, change `IP_NUMBER`/`PORT` in the ethernet config (and `<IP>` in
`kss_deployment/Config/User/Common/EthernetKRL/kss/b_ctrldbox_EkiKSSinterface.xml` for EKI)
before deploying.

### 2.2 Back up the current controller files - `backup.bat`

Copy the whole `kss_deployment` folder to the controller (USB stick), minimize the SmartHMI,
and run as Expert:

```
backup.bat
```

It copies the files that `deploy.bat` would overwrite (RSI and EKI configs, and
`KRC\R1\Program\b_ctrldbox\`) into `kss_deployment\Backup\<timestamp>\` next to the script.
Run it before every deployment so you can roll back or compare.

### 2.3 Deploy - `deploy.bat`

```
deploy.bat
```

1. Select the RSI version matching your KSS version (`3` = RSI 4.1.x / KSS 8.6, check
   `Help > Info > Installed additional software`).
2. Select configuration `1` (Standard).
3. Confirm each copy step (`Y`).

| Copied from `kss_deployment\` | To the controller |
| :---------------------------- | :---------------- |
| `Config\User\Common\SensorInterface\common\` (ethernet config) | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `Config\User\Common\SensorInterface\rsi_4.1.x\` (context) | same folder |
| `KRC\R1\Program\RSI_kss\` (RSI programs) | `C:\KRC\ROBOTER\KRC\R1\Program\b_ctrldbox\` |
| `Config\User\Common\EthernetKRL\kss\` + `KRC\R1\Program\EKIServer_kss\` (EKI, `eki_rsi` only) | `...\EthernetKRL\` / `...\Program\b_ctrldbox\` |

Then do a **cold restart** with "Reload files" (see `KUKA.md`, section 4).

### 2.4 What the files contain in this example

**Ethernet config** (`common/b_ctrldbox_rsi_eth.xml`, generated from `rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml`):

```xml
<SEND>                                                        <!-- controller -> driver -->
  <ELEMENT TAG="DEF_RIst"  TYPE="DOUBLE" INDX="INTERNAL"/>      <!-- Cartesian pose (actual), built-in -->
  <ELEMENT TAG="DEF_RSol"  TYPE="DOUBLE" INDX="INTERNAL"/>      <!-- Cartesian pose (setpoint), built-in -->
  <ELEMENT TAG="DEF_AIPos" TYPE="DOUBLE" INDX="INTERNAL"/>      <!-- joint positions, built-in -->
  <ELEMENT TAG="GearTorque.A1" TYPE="DOUBLE" INDX="1"/>         <!-- joint torques: Ethernet inputs 1-6 -->
  ...
  <ELEMENT TAG="GearTorque.A6" TYPE="DOUBLE" INDX="6"/>
  <ELEMENT TAG="DEF_MACur" TYPE="DOUBLE" INDX="INTERNAL"/>      <!-- motor currents, built-in -->
  <ELEMENT TAG="DEF_ASPos" TYPE="DOUBLE" INDX="INTERNAL"/>      <!-- axis setpoint positions, built-in -->
  <ELEMENT TAG="ProgStatus.R" TYPE="LONG" INDX="7"/>            <!-- $PRO_STATE1: Ethernet input 7 -->
  <ELEMENT TAG="OvPro.R" TYPE="DOUBLE" INDX="8"/>               <!-- $OV_PRO: Ethernet input 8 -->
  <ELEMENT TAG="DEF_Delay" TYPE="LONG" INDX="INTERNAL"/>        <!-- late packets, built-in -->
</SEND>
<RECEIVE>                                                     <!-- driver -> controller -->
  <ELEMENT TAG="Stop"  TYPE="BOOL"   INDX="1" HOLDON="0"/>      <!-- Ethernet output 1 -->
  <ELEMENT TAG="AK.A1" TYPE="DOUBLE" INDX="2" HOLDON="1"/>      <!-- Ethernet outputs 2-7 -->
  ...
  <ELEMENT TAG="AK.A6" TYPE="DOUBLE" INDX="7" HOLDON="1"/>
</RECEIVE>
```

**RSI context** (`rsi_4.1.x/b_ctrldbox_rsi.rsix`, open it in WorkVisual/RSI Visual to see the diagram):

| Object | Connected to | Purpose |
| :----- | :----------- | :------ |
| `Ethernet_1` | loads `b_ctrldbox_rsi_eth.xml` | sends/receives the telegram |
| `GearTorque_1` | Ethernet inputs 1-6 | measured joint torques A1-A6 (gear side) |
| `Status_1` (`ProState_R`) / `OV_PRO_1` | Ethernet inputs 7 / 8 | program state `$PRO_STATE1` / override `$OV_PRO` |
| `Stop_1` | Ethernet output 1 (`Stop`) | lets the driver stop the RSI motion |
| `AxisCorr_1` | Ethernet outputs 2-7 (`AK.A1`-`AK.A6`) | applies the joint corrections; limits are set from the software limit switches by `rsi_helper.src` |
| `AxisCorrMon_1` + `Monitor_1` | `AxisCorr_1` | correction monitoring / RSI monitor |

Tags with `INDX="INTERNAL"` are filled by the controller itself and need no object in the context.

**Driver YAML** (`rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml`, Standard is active as shipped): maps every telegram value to a ROS interface,
e.g. `AIPos.A1` → `joint_1/position`, `GearTorque.A1` → `joint_1/effort`, `MACur.A1` →
`joint_1/current`, `ASPos.A1` → `joint_1/position_setpoint`, `ProgStatus.R`/`OvPro.R` → the
`robot_status` sensor. Pass it to the driver with `rsi_xml_config_file:=<absolute path>`.

### 2.5 Run the ROS side

#### ctrlX CORE (b»Controlled Box)

> ⚠️ The split launch files below (`publish_description.launch.py`, `spawn_controllers.launch.py`)
> don't expose `rsi_xml_config_file` and the `read_*` arguments yet. Without the driver YAML, the
> driver can't read the current telegram layout, so the configurations in this repository can't be
> used on the ctrlX CORE until the driver supports these arguments there. Use the Linux PC setup
> below in the meantime.

> **Note:** On the ctrlX CORE we currently support only the `eki_rsi` driver version.

> **Test steps:** for a step-by-step test on the ctrlX CORE with the single scenario launch file
> (`scenario_eki_rsi.launch.xml`, with and without external axis), see [`TEST_CTRLX.md`](TEST_CTRLX.md).

**Step 1: Launch the robot driver.** This loads the robot description (URDF) and starts the RSI
hardware interface. The arguments have the same meaning as in the Linux PC table below.

- `robot_family`/`robot_model`: the robot model. **Adjust to your robot.** Supported models are in
  [`kuka_robot_descriptions`](https://github.com/kroshu/kuka_robot_descriptions).
- `controller_ip`: the controller's EKI (KLI) IP, used to start/stop the RSI program.
- `client_ip`: the ctrlX CORE IP on the RSI subnet. The driver listens for the RSI telegrams on this address.

```bash
ros2 launch kuka_rsi_driver publish_description.launch.py \
robot_family:=agilus \
robot_model:=kr10_r900_2 \
controller_ip:=10.28.23.240 \
driver_version:=eki_rsi \
client_ip:=10.23.23.28 \
client_port:=28283 \
use_gpio:=false \
verify_robot_model:=false
```

**Step 2: Spawn the controllers.** ⚠️ ***IMPORTANT:*** for now, this must be launched from the same
directory as this `LAUNCH.md` file (it reads `scenario_controllers.yaml`).

```bash
ros2 launch kuka_rsi_driver spawn_controllers.launch.py driver_version:=eki_rsi use_gpio:=false
```

⚠️ ***IMPORTANT:*** For real-time performance, switch the ctrlX controller to **OPERATIONAL** mode
before proceeding.

**Step 3: Run the robot manager node.** It activates the hardware and the controllers through its lifecycle states.

```bash
ros2 launch kuka_rsi_driver robot_manager.launch.py robot_model:=kr10_r900_2 driver_version:=eki_rsi use_gpio:=false
```

**Step 4: Activate the robot.** Set the pendant to `EXT` mode (required for `eki_rsi`), then:

```bash
ros2 lifecycle set /robot_manager configure
ros2 lifecycle set /robot_manager activate
```

With `eki_rsi`, the EKI server selects and starts the RSI program automatically.

#### Linux PC (single launch file)

Tested: KR 210 R3100-2, KSS 8.6.8, RSI 4.1.3, `rsi_only`. Adjust `robot_family`/`robot_model`.

```bash
ros2 launch kuka_rsi_driver startup.launch.py \
  driver_version:=rsi_only \
  robot_family:=quantec robot_model:=kr210_r3100_2 \
  client_ip:=10.23.23.28 client_port:=28283 \
  use_gpio:=false \
  rsi_xml_config_file:=<absolute path>/workspaces/kuka/rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml \
  read_robot_status:=true read_cartesian_pose:=true read_cartesian_setpoint:=true
```

**Launch arguments** (`startup.launch.py`; default in brackets):

| Argument | Meaning |
| :------- | :------ |
| `driver_version` [`rsi_only`] | `rsi_only`: RSI only, you start the RSI program on the pendant. `eki_rsi` / `mxa_rsi`: the program is started automatically through EKI / mxAutomation (needs `controller_ip` and `EXT` mode) |
| `robot_family`, `robot_model` [`agilus`, `kr6_r700_sixx`] | the robot description to load, e.g. `quantec` / `kr210_r3100_2`. Valid combinations: [`kuka_robot_descriptions`](https://github.com/kroshu/kuka_robot_descriptions) |
| `client_ip` [`0.0.0.0`] | IP of this PC on the RSI network - the driver listens for the RSI telegrams here. Must match `IP_NUMBER` in the ethernet config |
| `client_port` [`59152`] | UDP port for RSI. Must match `PORT` in the ethernet config (`28283` in this repository) |
| `controller_ip` [`0.0.0.0`] | KLI IP of the robot controller - only used with `eki_rsi`/`mxa_rsi` |
| `verify_robot_model` [`true`] | with `eki_rsi`/`mxa_rsi`: check that the robot model reported by the controller matches `robot_model` |
| `rsi_xml_config_file` [empty] | **absolute** path to `rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml`, with the lines of the deployed configuration uncommented (section 5). Empty = driver defaults (basic layout only) |
| `use_gpio` [`false`] | `true` adds the GPIO interfaces (`gpio_config.xacro`) and the `gpio_controller` - required for the GPIO configuration |
| `use_external_axis` [`false`] | `true` composes the robot with a KUKA linear unit at launch time and adds the external joint - required for the External Axis configuration |
| `kl_model` [`kl100_2`] | linear unit model used with `use_external_axis:=true` |
| `kl_prefix` [`rail_`] | name prefix of the external axis' links and joints (joint `rail_joint_1`). If changed, also change `joint_identifier` in the `[EXT_AXIS]` lines of the YAML |
| `read_robot_status` [`false`] | `true` adds the `robot_status` sensor (program state, speed scaling) - needs `ProgStatus`/`OvPro` in the YAML |
| `read_cartesian_pose` [`false`] | `true` adds the `cartesian_pose` sensor (actual TCP pose, `RIst`) |
| `read_cartesian_setpoint` [`false`] | `true` adds the `cartesian_setpoint` sensor (setpoint TCP pose, `RSol`) - needs `cartesian_setpoint.enabled: true` in the YAML |

Optional arguments you may need:

| Argument | Meaning |
| :------- | :------ |
| `mode` [`hardware`] | `mock` runs the driver without a robot (simulated hardware), e.g. to check the robot description |
| `namespace` [empty] | runs all nodes and controllers in a namespace and prefixes the robot's links/joints (`<namespace>_`), e.g. for two robots |
| `x`, `y`, `z`, `roll`, `pitch`, `yaw` [`0`] | pose of the robot's `base_link` in the `world` frame (m / rad) |
| `rt_core`, `rt_prio`, `non_rt_cores` [`-1`, `70`, empty] | CPU core and priority of the real-time control loop, and cores for the other threads (real-time PC tuning) |
| `lock_memory` [`true`] | lock the control loop's memory to avoid page faults (needs the memlock limit set for the user) |
| `enable_rsi_monitoring` [`false`] | `true` starts a node that monitors the RSI packets on `client_port` |

Motor currents (`current`) and setpoint positions (`position_setpoint`) are always exported; they
are filled when the YAML maps them. There is no `read_current` argument anymore.

```bash
ros2 lifecycle set robot_manager configure
ros2 lifecycle set robot_manager activate
```

With `rsi_only`, start the RSI program manually within 10 s after `activate`: select
`rsi_joint_pos_4ms` on the pendant (T1), keep start pressed until `RSI_MOVECORR()` and confirm
"!!! Attention - Sensor correction goes active !!!".

If `activate` fails (e.g. the program was not started in time or the configurations don't
match), recover without relaunching:

```bash
ros2 lifecycle set robot_manager cleanup
ros2 lifecycle set robot_manager configure
ros2 lifecycle set robot_manager activate
```

After `activate`, spawn the status and pose broadcasters (not spawned automatically):

```bash
ros2 run controller_manager spawner robot_status_broadcaster cartesian_pose_broadcaster cartesian_setpoint_broadcaster \
  -c controller_manager \
  --param-file $(ros2 pkg prefix --share kuka_rsi_driver)/config/kuka_cartesian_pose_broadcaster_config.yaml
```

What to check:

| Data | Where | Expected |
| :--- | :---- | :------- |
| Joint positions / torques | `/joint_states` → `position` / `effort` | real values (not `NaN`); torques clearly non-zero on A2/A3 at standstill |
| Motor currents, setpoint positions | `/dynamic_joint_states` → `current`, `position_setpoint` | real values for every joint |
| Program state / speed scaling | `/robot_status_broadcaster/program_state`, `/robot_status_broadcaster/speed_scaling` | follows the program state and the override on the pendant |
| Cartesian pose | `/cartesian_pose_broadcaster/pose`, `/cartesian_setpoint_broadcaster/pose` | position in **metres**, TCP of the active tool in the active base |

The driver log must not contain "... is not configured in RSI XML" warnings for this data.

### 2.6 First motion

Read the current joint positions and send a small, slow move (here: joint 6 by +0.05 rad over 10 s).
Replace the values with your own; only change one joint.

```bash
ros2 topic echo /joint_states --once
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [{positions: [<j1>, <j2>, <j3>, <j4>, <j5>, <j6 + 0.05>], time_from_start: {sec: 10}}]}}"
```

RSI applies every setpoint directly without smoothing - avoid tools that send jumps, such as
`rqt_joint_trajectory_controller`, and use MoveIt with low velocity/acceleration scaling for
real motion.

---

## 3. External axis example

Deploy configuration `2` (External Axis). Compared to Standard:

| File | Change |
| :--- | :----- |
| Ethernet config (`common/ext_axis/`) | `SEND`: `DEF_EIPos`, `DEF_MECur`, `DEF_ESPos` (built-ins) and `GearTorqueExt.E1` at `INDX="7"`; status moves to 8/9. `RECEIVE`: `EK.E1` at `INDX="8"` (external axis correction) |
| RSI context (`rsi_4.1.x/ext_axis/`) | additional `GearTorqueExt_1` (Ethernet input 7) and `AxisCorrExt_1` (Ethernet output 8, limits E1 ±1000 mm or deg) |
| Driver YAML | `rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml` with the lines tagged `[EXT_AXIS]` uncommented - the external joint is `rail_joint_1` |
| Robot description | robot + linear unit (KL) composed at launch time by `use_external_axis:=true` - no URDF file to edit |

ROS side (Linux PC example; arguments explained in section 2.5):

```bash
ros2 launch kuka_rsi_driver startup.launch.py \
  driver_version:=rsi_only \
  robot_family:=quantec robot_model:=kr210_r3100_2 \
  client_ip:=10.23.23.28 client_port:=28283 \
  use_external_axis:=true kl_model:=kl100_2 kl_prefix:=rail_ \
  use_gpio:=false \
  rsi_xml_config_file:=<absolute path>/workspaces/kuka/rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml \
  read_robot_status:=true read_cartesian_pose:=true read_cartesian_setpoint:=true
```

The external axis appears as an additional joint (`rail_joint_1`, position in metres) with the same
interfaces as the robot joints, and is part of the joint trajectory controller
(`joint_trajectory_controller_config_6_axis_kl.yaml`). The external axis must also be configured
on the KUKA controller; otherwise its values stay `0` and `EK.E1` corrections have no effect.

---

## 4. GPIO example

Deploy configuration `3` (GPIO). It adds 8 digital inputs and 12 digital outputs, exchanged
every RSI cycle:

| File | Change |
| :--- | :----- |
| Ethernet config (`common/gpios/`) | `SEND`: `GPIO.input_01`-`GPIO.input_08` at `INDX` 7-14; status moves to 15/16. `RECEIVE`: `GPIO.output_01`-`GPIO.output_12` at `INDX` 8-19 |
| RSI context (`rsi_4.1.x/gpios/`) | `DigIn_132`-`DigIn_139` (read `$IN[132..139]`) wired to Ethernet inputs 7-14; `Map2DigOut_33`-`_36`, `_39`-`_46` (write `$OUT[...]`) wired to Ethernet outputs 8-19 |
| Driver YAML | `rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml` with the lines tagged `[GPIO]` uncommented - lists the GPIO names (`motion_state.gpio` / `control_signal.gpio`) |
| Robot description | must declare exactly these GPIO interfaces (`input_01`.. as state interfaces, `output_01`.. as command interfaces), see section 6 |
| GPIO controller config | the same names, in `scenario_controllers.yaml` (`gpio_controller`) or `kuka_rsi_driver/config/gpio_controller_config.yaml` |

The signal names and I/O numbers are an example - adapt them to your application (section 6).

ROS side: as in section 2.5 (see the argument table there) with `use_gpio:=true` and
`rsi_xml_config_file:=<absolute path>/workspaces/kuka/rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml` (`[GPIO]` lines uncommented). Then:

```bash
# read inputs
ros2 topic echo /gpio_controller/gpio_states
# set output_01
ros2 topic pub --once /gpio_controller/commands control_msgs/msg/DynamicInterfaceGroupValues \
  "{interface_groups: [gpio], interface_values: [{interface_names: ['output_01'], values: [1.0]}]}"
```

While RSI is active, RSI writes the mapped outputs every cycle - don't write the same `$OUT`
from KRL at the same time.

---

## 5. Changing the message layout: the driver YAML

All three configurations use one driver YAML, [`rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml`](rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml). The
Standard configuration is active as shipped; the lines of the other configurations are commented
out and tagged `[EXT_AXIS]` or `[GPIO]`. Uncomment the tagged lines of the configuration you
deployed (remove the first `# `), and enable at most one of them:

| RSI configuration (`deploy.bat`) | Uncomment in the YAML | Matching ethernet config / context |
| :------------------------------- | :-------------------- | :--------------------------------- |
| 1 (Standard) | nothing | `common/b_ctrldbox_rsi_eth.xml` + `rsi_4.1.x/b_ctrldbox_rsi.rsix` |
| 2 (External Axis) | lines tagged `[EXT_AXIS]` | `common/ext_axis/b_ctrldbox_rsi_eth.xml` + `rsi_4.1.x/ext_axis/b_ctrldbox_rsi.rsix` |
| 3 (GPIO) | lines tagged `[GPIO]` | `common/gpios/b_ctrldbox_rsi_eth.xml` + `rsi_4.1.x/gpios/b_ctrldbox_rsi.rsix` |

```bash
# e.g. for the External Axis configuration (use [GPIO] for the GPIO configuration):
sed -i '/\[EXT_AXIS\]/s/# //' rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml
```

The YAML is the single source of truth. After changing it, regenerate the ethernet config instead
of editing it by hand - the generator assigns the indices in the order the driver expects:

```bash
ros2 run kuka_rsi_driver generate_krc_rsi_config.py --config rsi_xml_config/b_ctrldbox_rsi_xml_config.yaml \
  --client-ip 10.23.23.28 --client-port 28283 \
  --output kss_deployment/Config/User/Common/SensorInterface/common/b_ctrldbox_rsi_eth.xml
```

Tags without `INDX` (`DEF_*`) are built-ins. Every tag with an `INDX` needs an object in the
`.rsix` wired to that Ethernet port - check the indices in the generated file and rewire the
context in RSI Visual if they moved. Deploy the context and the ethernet config together.

---

## 6. Next steps: adding or renaming a GPIO

A GPIO name appears in up to five places; all of them must use the same name. Example: add a
digital input `vacuum_ok` on `$IN[140]` and a digital output `light` on `$OUT[47]` to the GPIO
configuration.

### 6.1 Robot description - GPIO interfaces (URDF)

The driver exports one `ros2_control` interface per GPIO. They are declared in
`kuka_rsi_driver/config/gpio_config.xacro` inside a `<gpio>` element that **must be named `gpio`**:
`state_interface` = robot → ROS (inputs), `command_interface` = ROS → robot (outputs).

```xml
<gpio name="gpio">
  <state_interface name="input_01" data_type="bool">
    <param name="initial_value">false</param>
  </state_interface>
  <!-- ... input_02 .. input_08 ... -->
  <state_interface name="vacuum_ok" data_type="bool">          <!-- new input -->
    <param name="initial_value">false</param>
  </state_interface>

  <command_interface name="output_01" data_type="bool">
    <param name="initial_value">false</param>
  </command_interface>
  <!-- ... output_02 .. output_12 ... -->
  <command_interface name="light" data_type="bool">            <!-- new output -->
    <param name="initial_value">false</param>
  </command_interface>
</gpio>
```

The interfaces are then available as `gpio/vacuum_ok` and `gpio/light`. The order of the state
interfaces is the order in which the driver expects the input values in the telegram.

> The robot descriptions include this file from the `kuka_rsi_driver` package, so changing GPIOs
> currently means editing that package and rebuilding it.

### 6.2 GPIO controller config

Add the names to the `gpio_controller` (`scenario_controllers.yaml` on the ctrlX CORE, otherwise
`kuka_rsi_driver/config/gpio_controller_config.yaml`):

```yaml
gpio_controller:
  ros__parameters:
    type: gpio_controllers/GpioCommandController
    gpios: [gpio]
    command_interfaces:
      gpio:
        interfaces: ["output_01", ..., "output_12", "light"]
    state_interfaces:
      gpio:
        interfaces: ["input_01", ..., "input_08", "vacuum_ok"]
```

### 6.3 Ethernet config (controller)

Don't edit the ethernet config by hand - regenerate it from the YAML after step 6.5. The
generator appends the new signals in their direction and moves the following fields:

```xml
<SEND>
  ...
  <ELEMENT TAG="GPIO.input_08"   TYPE="BOOL"   INDX="14"/>
  <ELEMENT TAG="GPIO.vacuum_ok"  TYPE="BOOL"   INDX="15"/>               <!-- new -->
  <ELEMENT TAG="ProgStatus.R"    TYPE="LONG"   INDX="16"/>               <!-- moved from 15 -->
  <ELEMENT TAG="OvPro.R"         TYPE="DOUBLE" INDX="17"/>               <!-- moved from 16 -->
  ...
</SEND>
<RECEIVE>
  ...
  <ELEMENT TAG="GPIO.output_12" TYPE="BOOL" INDX="19" HOLDON="1"/>
  <ELEMENT TAG="GPIO.light"     TYPE="BOOL" INDX="20" HOLDON="1"/>       <!-- new -->
</RECEIVE>
```

The part after `GPIO.` must be the interface name from 6.1. The inputs are in the same order as
the state interfaces in the URDF and the names in the YAML.

### 6.4 RSI context (controller)

In WorkVisual / RSI Visual, open `rsi_4.1.x/gpios/b_ctrldbox_rsi.rsix` and:

- input: add a `DigIn` object with `Index = 140` and connect its output to `Ethernet_1` **input 15**
- move `Status_1` to `Ethernet_1` **input 16** and `OV_PRO_1` to **input 17** (they moved in 6.3)
- output: add a `Map2DigOut` object with `Index = 47` and connect `Ethernet_1` **output 20** to its input

The Ethernet port numbers are the `INDX` values from the regenerated ethernet config. Save, then
deploy the `.rsix` and the ethernet config together.

### 6.5 Driver YAML

Add the names to `motion_state.gpio.xml_attributes` (inputs) and
`control_signal.gpio.xml_attributes` (outputs) - the `[GPIO]` blocks of `b_ctrldbox_rsi_xml_config.yaml` - then regenerate the ethernet
config with `generate_krc_rsi_config.py` (section 5) instead of editing it by hand - the generator
assigns the indices. Note that the robot status fields
(`ProgStatus.R`, `OvPro.R`) come after the GPIO inputs, so their ports move as well: rewire them in
the `.rsix` to the new indices.

### 6.6 Checklist

| # | Place | Inputs | Outputs |
| :- | :---- | :----- | :------ |
| 1 | `gpio_config.xacro` | `state_interface` | `command_interface` |
| 2 | GPIO controller config | `state_interfaces` | `command_interfaces` |
| 3 | Ethernet config (regenerated from 5) | `SEND`, `GPIO.<name>`, new `INDX` - later fields move | `RECEIVE`, `GPIO.<name>`, new `INDX`, `HOLDON="1"` |
| 4 | `.rsix` | `DigIn` (`$IN` index) → Ethernet input `INDX` | Ethernet output `INDX` → `Map2DigOut` (`$OUT` index) |
| 5 | Driver YAML (`[GPIO]` blocks) | `motion_state.gpio.xml_attributes` | `control_signal.gpio.xml_attributes` |

If the driver refuses to start with "motion_state.gpio.xml_attributes has N entries but M GPIO
state interfaces", places 1 and 5 don't match. If values stay `0`/`false`, check places 3 and 4.
