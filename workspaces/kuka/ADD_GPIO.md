# Adding a GPIO (KUKA RSI with the b»Controlled Box)

How to add a digital input or output to the RSI telegram, and which file to change where. The
example extends the `gpio_51` test configuration (`input_01` ← `$IN[51]`, `output_01` →
`$OUT[51]`) by one more input and output:

| New GPIO | Direction | KUKA signal |
| :------- | :-------- | :---------- |
| `input_02` | robot → ROS (state interface) | `$IN[52]` |
| `output_02` | ROS → robot (command interface) | `$OUT[52]` |

Before you start, make sure the KUKA signals are free: not used by safety, a PLC handshake, a
gripper or a KRL program. While RSI is active, it writes every mapped output each cycle.

---

## 1. Where the files are

A GPIO name appears in five files. They live in three places, and all of them must use the same
names in the same order.

| # | File | Repository / location | Used by | Deployed to |
| :- | :--- | :-------------------- | :------ | :---------- |
| 1 | Driver YAML `rsi_xml_config/b_ctrldbox_rsi_xml_config_gpio_51.yaml` | `b_ctrldbox_commissioning/workspaces/kuka/` | driver on the ctrlX | ctrlX, `/var/snap/rexroth-automationcore/common/solutions/activeConfiguration/b-controlled-box/` |
| 2 | Ethernet config `b_ctrldbox_rsi_eth.xml` | `b_ctrldbox_commissioning/workspaces/kuka/kss_deployment/Config/User/Common/SensorInterface/common/gpio_51/` | RSI on the KUKA | KUKA, `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` (`deploy.bat`, option 4) |
| 3 | RSI context `b_ctrldbox_rsi.rsix` | `b_ctrldbox_commissioning/workspaces/kuka/kss_deployment/Config/User/Common/SensorInterface/rsi_4.1.x/gpio_51/` | RSI on the KUKA | KUKA, same folder as 2 (`deploy.bat`, option 4) |
| 4 | Robot description `config/gpio_config.xacro` | `kuka_drivers/kuka_rsi_driver/` | `publish_description.launch.py` on the PC | nothing - read on the PC when the launch file starts |
| 5 | GPIO controller config `scenario_controllers.yaml` (`gpio_controller`) | `b_ctrldbox_commissioning/workspaces/kuka/` | controller manager on the ctrlX | ctrlX, same folder as 1 |

Without the ctrlX (Linux PC, `startup.launch.py`), file 5 is
`kuka_rsi_driver/config/gpio_controller_config.yaml` and file 1 is passed with
`rsi_xml_config_file:=<absolute path>`.

How the files connect, for one input:

```
$IN[52] --(.rsix: DigIn_52)--> Ethernet input 8 --(eth.xml: INDX="8")--> <GPIO input_02="1"/>
        --(driver YAML: motion_state.gpio)--> gpio/input_02 state interface (gpio_config.xacro)
        --(gpio_controller)--> /gpio_controller/gpio_states
```

---

## 2. Step by step

Make the changes in this order. The ethernet config follows the order of the driver YAML, and the
`.rsix` wiring depends on the indices in the ethernet config.

### 2.1 Driver YAML (file 1)

Add the names to the two `gpio` blocks. Inputs go to `motion_state.gpio`, outputs to
`control_signal.gpio`:

```yaml
rsi_xml_config:
  motion_state:
    ...
    gpio:
      xml_element: GPIO
      xml_attributes:
        - input_01                 # $IN[51]
        - input_02                 # $IN[52]   <- new
  control_signal:
    ...
    gpio:
      xml_element: GPIO
      xml_attributes:
        - output_01                # $OUT[51]
        - output_02                # $OUT[52]  <- new
```

The order of `motion_state.gpio.xml_attributes` must be the order of the state interfaces in
`gpio_config.xacro` (2.4).

### 2.2 Ethernet config and RSI context with an AI agent (files 2 and 3)

Both files are plain XML. Let a coding agent (e.g. Claude Code) update them from the driver YAML,
using the existing `gpio_51` files as the template. No WorkVisual / RSI Visual is needed to edit
them. Start the agent in the `b_ctrldbox_commissioning` folder and give it this prompt (adjust the
names and signals):

```text
In workspaces/kuka, I added input_02 ($IN[52]) and output_02 ($OUT[52]) to
rsi_xml_config/b_ctrldbox_rsi_xml_config_gpio_51.yaml. Update the two KUKA RSI files of the
gpio_51 configuration in kss_deployment/Config/User/Common/SensorInterface/ to match it:

1. common/gpio_51/b_ctrldbox_rsi_eth.xml (ethernet config):
   - add one ELEMENT per new GPIO: inputs in <SEND>, outputs in <RECEIVE> with HOLDON="1",
     TAG="GPIO.<name>" and TYPE="DOUBLE" like the existing GPIO elements;
   - keep the order of the driver YAML: GPIO inputs come before ProgStatus.R and OvPro.R, so
     renumber the INDX of every element after them.
2. rsi_4.1.x/gpio_51/b_ctrldbox_rsi.rsix (RSI context):
   - add a DigIn object (Index = 52) connected to the Ethernet_1 input of the new input, and a
     Map2DigOut object (Index = 52) fed from the Ethernet_1 output of the new output;
   - reconnect every existing object whose INDX moved (Status_1, OV_PRO_1);
   - Ethernet_1 ports in the .rsix are 0-based: INDX n in the ethernet config = InIdx/OutIdx n-1;
   - copy the structure of DigIn_51 / Map2DigOut_51, use unique ObjIds, keep the UTF-8 BOM and
     the CRLF line endings.
Don't change anything else. Then show me a table of every Ethernet element with its old INDX,
new INDX and the .rsix object it is connected to.
```

Expected ethernet config (only the changed part):

```xml
<SEND>
  ...
  <ELEMENT TAG="GPIO.input_01" TYPE="DOUBLE" INDX="7"/>
  <ELEMENT TAG="GPIO.input_02" TYPE="DOUBLE" INDX="8"/>              <!-- new -->
  <ELEMENT TAG="ProgStatus.R"  TYPE="LONG"   INDX="9"/>              <!-- moved from 8 -->
  <ELEMENT TAG="OvPro.R"       TYPE="DOUBLE" INDX="10"/>             <!-- moved from 9 -->
  ...
</SEND>
<RECEIVE>
  ...
  <ELEMENT TAG="GPIO.output_01" TYPE="DOUBLE" INDX="8" HOLDON="1"/>
  <ELEMENT TAG="GPIO.output_02" TYPE="DOUBLE" INDX="9" HOLDON="1"/>    <!-- new -->
</RECEIVE>
```

Inputs come before the robot status fields (`ProgStatus.R`, `OvPro.R`), so every new input moves
the status fields by one index, and the RSI context must be reconnected for them too.

> ⚠️ **GPIO data type.** Use `TYPE="DOUBLE"` for the GPIO elements. The robot description declares
> the GPIOs as `double` (see 2.4), and the driver sends output values as `1.0000` / `0.0000`. This
> combination runs on a real controller, see section 4.

### 2.3 Check the agent's result (files 2 and 3)

The agent can make mistakes, and a wrong index switches the wrong output or makes the KUKA reject
the telegram. Check before deploying:

- **Ethernet config:** the order of the `GPIO.` elements is the order in the driver YAML, and the
  `INDX` values have no gaps or duplicates.
- **RSI context:** for this example, `Ethernet_1` must be connected like this (`.rsix` index =
  `INDX` - 1):

  | Object | Ethernet config | `.rsix` connection |
  | :----- | :-------------- | :----------------- |
  | `DigIn_52` (`Index = 52`) | `GPIO.input_02`, `INDX="8"` | `Ethernet_1` `InIdx="7"` |
  | `Status_1` | `ProgStatus.R`, `INDX="9"` | `Ethernet_1` `InIdx="8"` |
  | `OV_PRO_1` | `OvPro.R`, `INDX="10"` | `Ethernet_1` `InIdx="9"` |
  | `Map2DigOut_52` (`Index = 52`) | `GPIO.output_02`, `INDX="9"` | `OutIdx="8"` from `Ethernet_1` |

- **Only these two files changed:** `git diff --stat`.
- **Opens in RSI Visual:** recommended. Open the `.rsix` once in WorkVisual / RSI Visual and check
  that it loads and shows the new connections.

Keep both files in the `gpio_51` folders. For RSI 3.3.x, the RSI context consists of three files
(`.rsi`, `.rsi.xml`, `.rsi.diagram`): name all three in the prompt.

### 2.4 Robot description (file 4)

Add one interface per GPIO to `kuka_rsi_driver/config/gpio_config.xacro`. The `<gpio>` element
must be named `gpio`. `state_interface` = input, `command_interface` = output:

```xml
<gpio name="gpio">
  <!-- output_01 -> $OUT[51] -->
  <command_interface name="output_01" data_type="double">
    <limits enable="false"/>
    <param name="initial_value">0.0</param>
  </command_interface>
  <!-- output_02 -> $OUT[52] -->
  <command_interface name="output_02" data_type="double">
    <limits enable="false"/>
    <param name="initial_value">0.0</param>
  </command_interface>

  <!-- input_01 <- $IN[51] -->
  <state_interface name="input_01" data_type="double">
    <limits enable="false"/>
    <param name="initial_value">0.0</param>
  </state_interface>
  <!-- input_02 <- $IN[52] -->
  <state_interface name="input_02" data_type="double">
    <limits enable="false"/>
    <param name="initial_value">0.0</param>
  </state_interface>
</gpio>
```

- Use `data_type="double"` and `initial_value` `0.0`. With `data_type="bool"`, activation fails
  with `Handle::set_value ... value_ptr_ is null`: the driver and
  `gpio_controllers/GpioCommandController` access GPIO interfaces as `double`. 0 = off, 1 = on.
- Every robot family includes this file (`use_gpio:=true`), so the change applies to all robots.
- Don't write `: ` (colon followed by a space) in comments in this file. The launch file reads the
  whole robot description as YAML, and a colon-space breaks it with "Unable to parse the value of
  parameter robot_description as yaml".
- The robot description is generated on the PC. With a symlink install (`colcon build
  --symlink-install`), no rebuild is needed - the next launch uses the changed file. The ctrlX
  snap doesn't need to be rebuilt either.

### 2.5 GPIO controller config (file 5)

Add the names to `gpio_controller` in `b_ctrldbox_commissioning/workspaces/kuka/scenario_controllers.yaml`:

```yaml
gpio_controller:
  ros__parameters:
    type: gpio_controllers/GpioCommandController
    gpios:
      - gpio
    command_interfaces:
      gpio:
        interfaces:
           - output_01
           - output_02
    state_interfaces:
      gpio:
        interfaces:
           - input_01
           - input_02
```

The controller manager on the ctrlX reads the parameters only from its own copy of this file. If
the section is missing on the ctrlX, the spawner fails with "No command or state interfaces are
configured".

---

## 3. Deploy and test

1. **KUKA:** run `kss_deployment/deploy.bat`, select RSI version 3 (4.1.x) and configuration
   4 (GPIO 51 test). This copies the ethernet config and the `.rsix` to
   `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\`.
2. **ctrlX:** upload `scenario_controllers.yaml` and
   `rsi_xml_config/b_ctrldbox_rsi_xml_config_gpio_51.yaml` to
   `/var/snap/rexroth-automationcore/common/solutions/activeConfiguration/b-controlled-box/`, then
   restart the ctrlX app. How to upload through the ctrlX OS web interface:
   [`TEST_CTRLX.md`](TEST_CTRLX.md), section 1.1.
3. **KUKA pendant:** EXT mode, no program selected, no simulated inputs/outputs left on.
4. **PC:** launch in the commissioning container (`controllers_dir` = same
   `scenario_controllers.yaml` as on the ctrlX), wait 10 seconds, then configure and activate:

   ```bash
   ros2 launch kuka_rsi_driver scenario_eki_rsi.launch.xml \
     robot_family:=quantec robot_model:=kr210_r3100_2 controller_ip:=172.31.1.178 \
     rsi_xml_config:=b_ctrldbox_rsi_xml_config_gpio_51.yaml use_gpio:=true \
     read_robot_status:=true read_cartesian_pose:=true read_cartesian_setpoint:=true \
     controllers_dir:=/home/b-robotized/comissioning/ros2_jazzy/src/b_ctrldbox_commissioning \
     is_async:=true

   ros2 lifecycle set /robot_manager configure
   ros2 lifecycle set /robot_manager activate
   ros2 control list_controllers -c /b_controlled_box_cm    # gpio_controller: active
   ```

   On `configure`, the ctrlX log lists the GPIOs: "Configured GPIO commands: Name: output_02 ...",
   "Configured GPIO states: Name: input_02 ...".

5. **Read the inputs:**

   ```bash
   ros2 topic echo /gpio_controller/gpio_states
   ```

   Switch `$IN[52]` on the KUKA; `input_02` changes between `0.0` and `1.0`.

6. **Set an output:**

   ```bash
   ros2 topic pub --once /gpio_controller/commands control_msgs/msg/DynamicInterfaceGroupValues \
     "{interface_groups: [gpio], interface_values: [{interface_names: ['output_02'], values: [1.0]}]}"
   ```

   Check `$OUT[52]` on the pendant (Display → Inputs/outputs → Digital outputs). Send `0.0` to switch
   it off.

---

## 4. Troubleshooting

| Symptom | Cause | Check |
| :------ | :---- | :---- |
| Launch fails: "Unable to parse the value of parameter robot_description as yaml" | `: ` in a comment in `gpio_config.xacro` | file 4 |
| Spawner: "No command or state interfaces are configured" | `gpio_controller` section missing or outdated on the ctrlX | file 5 on the ctrlX |
| Hardware doesn't start: "motion_state.gpio.xml_attributes has N entries but M GPIO state interfaces are defined" | YAML and robot description list different inputs | files 1 and 4 |
| `activate` fails: "value_ptr_ is null" | GPIO interfaces declared as `bool` | file 4, use `double` |
| `activate` fails: "Received XML is missing configured GPIO element" | the KUKA doesn't send the GPIO element the YAML expects | files 1 and 2, deployed configuration |
| RSI stops right after activation ("Failed to receive motion state", "External control stopped") | the KUKA rejects the reply, e.g. ethernet config on the KUKA not updated, or `BOOL`/`DOUBLE` mismatch | message window on the pendant, file 2 on the KUKA |
| Input/output values never change | wrong `$IN`/`$OUT` index or wiring in the `.rsix`, or a simulated signal still active | file 3, pendant I/O display |

Status: GPIO elements as `TYPE="DOUBLE"` on the KUKA with `double` GPIO interfaces in the robot
description run on a KR 10 cell (Bosch gripper, digital outputs for suction and cylinders, sensor
inputs). The `gpio_51` configuration activates on a KR 210 R3100-2 (KSS 8.6, RSI 4.1); there, RSI
stopped right after activation in the first test with `TYPE="DOUBLE"`. If this happens, check the
message window on the pendant and that the ethernet config on the KUKA was updated (row "RSI stops
right after activation" above).
