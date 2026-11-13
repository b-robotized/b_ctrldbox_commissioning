# AGENTS.md

Guidance for AI agents (Claude Code, Codex, and others) and for people who use this repository to commission a **b»controlled box**.

## What this repository is

Documentation and tools to commission b»controlled box: ROS 2 `ros2_control` in real time on a Bosch Rexroth ctrlX CORE, connected to an industrial robot.

| Path | Content |
|---|---|
| `README.md` | Overview, compatibility, getting started |
| `docs/SETUP_CTRLX.md` | ctrlX CORE setup with the web UI |
| `docs/SETUP_CTRLX_REMOTE.md` | ctrlX CORE setup over the REST API (`ctrlx-remote`) |
| `docs/REQUIRED_SNAPS.md` | Apps and versions every box needs |
| `docs/BACKUP_AND_TRANSFER.md` | Backup, restore, and the backup format |
| `docs/MOVE_TO_ANOTHER_BOX.md` | Move the application from one ctrlX CORE to another |
| `docs/supported_robots/<ROBOT>.md` | Robot-side setup per manufacturer |
| `docs/SETUP_COMMMISSIONING.md`, `start.sh`, `workspaces/` | ROS 2 commissioning container and robot workspaces |
| `docs/CTRLX_REST_API_NOTES.md` | ctrlX REST API facts and known issues that are not in the public description |
| `tools/ctrlx_remote/` | CLI, Python library, and MCP server for the ctrlX REST API |
| `.claude/skills/ctrlx_box_operations/` | Agent skill: check, update, back up, and transfer devices with `ctrlx-remote` |

## Commissioning order

1. Network: connect the PC to `XF10` of the ctrlX CORE ([SETUP_CTRLX.md](docs/SETUP_CTRLX.md#1-network-connection)).
2. Device check and backup: `ctrlx-remote info`, `ctrlx-remote backup <dir>`.
3. System app update: `ctrlx-remote system-apps update <apps-folder>` ([SETUP_CTRLX_REMOTE.md](docs/SETUP_CTRLX_REMOTE.md#4-update-the-system-apps)).
4. b»controlled box apps: `ros2-base-jazzy`, `zenohd-router-jazzy-binary`, `b-controlled-box-rt-jazzy`, `b-controlled-box-ui-jazzy` ([REQUIRED_SNAPS.md](docs/REQUIRED_SNAPS.md)).
5. Robot setup ([supported_robots/](docs/supported_robots/)).
6. Scheduler, operation mode SETUP, check the controllers.
7. Backup after commissioning; keep it in a folder that the customer controls.

To move an existing application to a new box, follow [MOVE_TO_ANOTHER_BOX.md](docs/MOVE_TO_ANOTHER_BOX.md).

## Tools

```bash
pip install "./tools/ctrlx_remote[mcp]"
ctrlx-remote --env-file ~/.config/b-ctrldbox/<box>.env info
ctrlx-mcp --devices-dir ~/.config/b-ctrldbox            # read-only MCP server
```

See [tools/ctrlx_remote/README.md](tools/ctrlx_remote/README.md) for all commands.
Claude Code loads the skill `ctrlx_box_operations` for updates, backups, and transfers; other agents read `.claude/skills/ctrlx_box_operations/SKILL.md` directly.

## Safety rules for agents

The ctrlX CORE controls a real robot.

- Ask the user before each action that changes a device: install, update, or remove apps; change the operation mode; apply a backup; change network, firewall, or users; restart.
- Never switch a device to **OPERATING** without an explicit request from the user for that device. In OPERATING, controllers can move the robot.
- Make a backup before you change a device.
- Do not change the network settings of the port you use to reach the device.
- Check the free space on the data disk first (`ctrlx-remote info`). The ctrlX CORE X3 has only 2.9 GB; a full disk stops configuration saving.
- Use the MCP server without `--allow-write` unless the user asks for changes.

## Secrets and files

- Device passwords are only in the device env files (`~/.config/b-ctrldbox/*.env`, `chmod 600`). Never write them into this repository, logs, or messages.
- Backups (`*_ctrlx-backup*.zip`) contain password hashes and keys. Never commit them; `.gitignore` excludes them.
- Do not commit `.snap`, `.app`, or license (`.bin`) files.

## Writing style

- Simple technical English (ASD-STE100): short sentences, active voice, one term for one thing.
- One sentence per line in Markdown files.
- Every REST call that the tools use is also documented in the docs, so users can work without the tools.
