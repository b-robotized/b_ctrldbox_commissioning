# ctrlx-remote

Command line tool, Python library, and MCP server for the remote commissioning of ctrlX CORE devices with **b»controlled box**.
It uses the ctrlX OS REST API.

Tested with ctrlX OS 4.6 (ctrlX CORE X3).

## Install

```bash
pip install .            # CLI and library
pip install ".[mcp]"     # also the MCP server
```

Requirements: Python 3.10 or newer, `requests`; `mcp` for the MCP server.

## Device settings

One file per device, for example `~/.config/b-ctrldbox/box1.env` (`chmod 600`, never in git):

```bash
CTRLX_IP=192.168.1.1
CTRLX_USER=boschrexroth
CTRLX_PASSWORD=<password>
# CTRLX_SOURCE_ADDRESS=192.168.1.10   # optional local IP to send from
```

## CLI

```bash
ctrlx-remote --env-file ~/.config/b-ctrldbox/box1.env <command>
```

| Command | Action | Changes the device |
|---|---|---|
| `info` | ctrlX OS version, architecture, mode, free disk space | no |
| `apps list` | installed apps and App Storage content | no |
| `system-apps check <apps-folder>` | outdated system apps and missing standard apps | no |
| `system-apps stage <apps-folder>` | upload them to App Storage | yes |
| `system-apps update <apps-folder> [--extra <app>...]` | upload, switch to SERVICE, install in the correct order | yes |
| `apps install <file>...` | upload and install `.snap`/`.app` files (SERVICE mode) | yes |
| `storage-remove <name>...` | remove apps from App Storage; installed apps stay | yes |
| `mode get` / `mode set <SERVICE\|SETUP\|OPERATING>` | operation mode | `set`: yes |
| `backup <dir> [--without-apps]` | create a backup, download it, delete it on the device | yes (temporary file) |
| `transfer prepare <backup> <out>` | remove device sections (IPs, host name, users, licenses, certificates) from a backup; offline | no |
| `transfer apps <backup> --snaps-dir <dir>` | install the non-system apps of the source device in the same versions | yes |
| `transfer apply <clean-backup>` | apply a prepared backup in merge mode | yes |
| `raw <METHOD> <path> [--body <json>]` | one REST request with a valid token | depends |

The tool asks before each write action; `-y` skips the question.
See [docs/SETUP_CTRLX_REMOTE.md](../../docs/SETUP_CTRLX_REMOTE.md) for the commissioning steps.

## MCP server

```bash
ctrlx-mcp --devices-dir ~/.config/b-ctrldbox             # read-only tools
ctrlx-mcp --devices-dir ~/.config/b-ctrldbox --allow-write
```

Each `<name>.env` file in the devices folder is one device; tools take the device name.

| Tool | Write |
|---|---|
| `list_devices`, `device_info`, `list_apps`, `check_updates`, `read_datalayer` | no |
| `set_mode`, `update_apps`, `create_backup` | only with `--allow-write` |

Add it to Claude Code:

```bash
claude mcp add ctrlx -- ctrlx-mcp --devices-dir ~/.config/b-ctrldbox
```

## Library

```python
from ctrlx_remote import CtrlxClient, Credentials

client = CtrlxClient(Credentials.from_env("~/.config/b-ctrldbox/box1.env"))
print(client.mode())
print(client.packages())
```

## Tests

```bash
python3 -m pytest tests   # offline, no device needed
```

## Notes

- The App Storage endpoints (`/package-manager/api/v1/repositories/...`) and the batch install format (`"location": "batch://<id>"`) are not in the public OpenAPI description 1.15.0.
  They are taken from the ctrlX OS 4.6 web UI.
- The device uses a self-signed certificate; the tool does not check it unless you give `--ca-cert`.
- HTTP/3 is switched off; the `urllib3-future` package fails on the device certificate with HTTP/3.
