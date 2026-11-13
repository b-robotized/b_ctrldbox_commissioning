"""REST client for ctrlX OS devices.

Covers the APIs that commissioning needs: identity manager (token),
package manager (apps), setup (backup and restore), system info,
and Data Layer (operation mode, scheduler).
API reference: https://boschrexroth.github.io/rest-api-description/ctrlx-automation/ctrlx-core
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable

import requests
import urllib3
from requests.adapters import HTTPAdapter

PACKAGE_MANAGER = "/package-manager/api/v1"
SETUP = "/setup/api/v1"
SYSTEM = "/system/api/v1"
DATALAYER = "/automation/api/v2"
IDENTITY = "/identity-manager/api/v2"

MODE_NODE = "scheduler/admin/state"
MODES = ("SERVICE", "SETUP", "OPERATING")

APP_STORAGE = {
    "id": "control_repository",
    "name": "Local storage",
    "path": "file:///var/snap/rexroth-deviceadmin/common/repositories/control_repository",
    "type": "snap",
    "writeable": True,
}


class CtrlxError(RuntimeError):
    """Error response or failed task on the device."""

    def __init__(self, message: str, status_code: int | None = None) -> None:
        super().__init__(message)
        self.status_code = status_code


@dataclass
class Credentials:
    host: str
    user: str
    password: str

    @classmethod
    def from_env(cls, env_file: str | None = None) -> "Credentials":
        """Read CTRLX_IP (or CTRLX_HOST), CTRLX_USER, CTRLX_PASSWORD.

        Values from `env_file` take precedence over the process environment.
        """
        values = dict(os.environ)
        if env_file:
            values.update(read_env_file(env_file))
        host = values.get("CTRLX_HOST") or values.get("CTRLX_IP")
        missing = [
            name
            for name, value in (
                ("CTRLX_IP", host),
                ("CTRLX_USER", values.get("CTRLX_USER")),
                ("CTRLX_PASSWORD", values.get("CTRLX_PASSWORD")),
            )
            if not value
        ]
        if missing:
            raise CtrlxError(f"Missing settings: {', '.join(missing)}")
        return cls(host, values["CTRLX_USER"], values["CTRLX_PASSWORD"])


def read_env_file(path: str) -> dict[str, str]:
    """Read KEY=VALUE lines. Ignore empty lines and comments."""
    result = {}
    for line in Path(path).expanduser().read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip().strip('"').strip("'")
    return result


class _CtrlxAdapter(HTTPAdapter):
    """HTTP adapter for ctrlX devices.

    - Optional local source IP address. Use it when a VPN route hides
      the local network of the device.
    - HTTP/3 off. The `urllib3-future` package (it replaces `urllib3`
      on some systems) switches to HTTP/3 after the first response and
      fails on the self-signed device certificate.
    """

    def __init__(self, source_address: str | None = None, **kwargs: Any) -> None:
        self._source_address = (source_address, 0) if source_address else None
        super().__init__(**kwargs)

    def init_poolmanager(self, *args: Any, **kwargs: Any) -> None:
        if self._source_address:
            kwargs["source_address"] = self._source_address
        http_version = getattr(urllib3, "HttpVersion", None)
        if http_version is not None:
            kwargs["disabled_svn"] = {http_version.h3}
        super().init_poolmanager(*args, **kwargs)


class CtrlxClient:
    """Authenticated session to one ctrlX OS device."""

    def __init__(
        self,
        credentials: Credentials,
        verify: bool | str = False,
        source_address: str | None = None,
        timeout: float = 30.0,
    ) -> None:
        self.credentials = credentials
        self.base_url = f"https://{credentials.host}"
        self.timeout = timeout
        self.session = requests.Session()
        self.session.verify = verify
        if verify is False:
            # ctrlX devices use a self-signed certificate by default.
            urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
        self.session.mount("https://", _CtrlxAdapter(source_address))
        self._token: str | None = None

    # -- basic HTTP ----------------------------------------------------------

    def login(self) -> None:
        response = self.session.post(
            f"{self.base_url}{IDENTITY}/auth/token",
            json={"name": self.credentials.user, "password": self.credentials.password},
            timeout=self.timeout,
        )
        _raise_for_status(response)
        self._token = response.json()["access_token"]
        self.session.headers["Authorization"] = f"Bearer {self._token}"

    def request(self, method: str, path: str, **kwargs: Any) -> requests.Response:
        if self._token is None:
            self.login()
        kwargs.setdefault("timeout", self.timeout)
        response = self.session.request(method, f"{self.base_url}{path}", **kwargs)
        if response.status_code == 401:
            # Token expired: log in again once.
            self.login()
            response = self.session.request(method, f"{self.base_url}{path}", **kwargs)
        _raise_for_status(response)
        return response

    def get_json(self, path: str, **kwargs: Any) -> Any:
        return self.request("GET", path, **kwargs).json()

    # -- system --------------------------------------------------------------

    def system_info(self) -> dict[str, Any]:
        return self.get_json(f"{SYSTEM}/info")

    def typeplate(self) -> dict[str, Any]:
        return self.get_json(f"{SYSTEM}/typeplate")

    def hostname(self) -> str:
        result = self.get_json(f"{SYSTEM}/hostname")
        return result["hostname"] if isinstance(result, dict) else str(result)

    # -- Data Layer ----------------------------------------------------------

    def datalayer_read(self, node: str) -> Any:
        return self.get_json(f"{DATALAYER}/nodes/{node}")

    def datalayer_write(self, node: str, value: Any, type_: str) -> Any:
        return self.request(
            "PUT", f"{DATALAYER}/nodes/{node}", json={"type": type_, "value": value}
        ).json()

    def datalayer_browse(self, node: str) -> list[str]:
        result = self.get_json(f"{DATALAYER}/nodes/{node}", params={"type": "browse"})
        return result.get("value", [])

    # -- package manager -----------------------------------------------------

    def packages(self) -> list[dict[str, Any]]:
        return self.get_json(f"{PACKAGE_MANAGER}/packages")

    def package_settings(self) -> dict[str, Any]:
        return self.get_json(f"{PACKAGE_MANAGER}/settings")

    def set_package_settings(self, **settings: Any) -> dict[str, Any]:
        """Change app settings, e.g. allowUnknownApps=True. Returns the new settings."""
        current = self.package_settings()
        current.update(settings)
        self.request("PUT", f"{PACKAGE_MANAGER}/settings", json=current)  # empty response body
        return self.package_settings()

    def batch_upload(self, files: Iterable[Path]) -> dict[str, Any]:
        """Upload app files to a temporary batch folder (removed after 1 h)."""
        handles = [open(path, "rb") for path in files]
        try:
            multipart = [
                ("files", (Path(h.name).name, h, "application/octet-stream")) for h in handles
            ]
            return self.request(
                "POST", f"{PACKAGE_MANAGER}/batch-upload", files=multipart, timeout=1800
            ).json()
        finally:
            for handle in handles:
                handle.close()

    def wait_batch(self, batch_id: str, timeout: float = 900, poll: float = 2.0) -> dict[str, Any]:
        """Wait until the device has checked all files of a batch upload."""
        deadline = time.monotonic() + timeout
        while True:
            batch = self.get_json(f"{PACKAGE_MANAGER}/batch-upload/{batch_id}")
            if batch.get("state") not in ("pending", "running"):
                return batch
            if time.monotonic() > deadline:
                raise CtrlxError(f"Batch {batch_id} not ready in {timeout} s")
            time.sleep(poll)

    def install_files(
        self, files: Iterable[Path], installed: Iterable[str], **wait_kwargs: Any
    ) -> list[dict[str, Any]]:
        """Upload app files as one batch, then install or update each app.

        `POST /packages` refuses apps that are already installed; this works for both.
        """
        installed = set(installed)
        batch = self.wait_batch(self.batch_upload(files)["id"])
        results = []
        for entry in batch.get("files", []):
            package = entry.get("result") or {}
            if not package.get("id"):
                raise CtrlxError(f"Device rejected {entry.get('filename')}: {entry}")
            release = package["releases"][0]
            response = self.request(
                "POST",
                f"{PACKAGE_MANAGER}/tasks",
                json={
                    "action": "update" if package["name"] in installed else "install",
                    "parameters": {
                        "id": package["id"],
                        "version": release["version"],
                        "location": release["location"],
                    },
                },
            )
            results.append(self.wait_task(f"{PACKAGE_MANAGER}/tasks", _task_id(response), **wait_kwargs))
        return results

    def install_batch(self, batch_id: str, **wait_kwargs: Any) -> dict[str, Any]:
        """Install all files of a batch upload in one task."""
        response = self.request(
            "POST",
            f"{PACKAGE_MANAGER}/tasks",
            # ctrlX OS 4.6 web UI uses "location"; the public spec 1.15.0 shows "id".
            json={"action": "batch", "parameters": {"location": f"batch://{batch_id}"}},
        )
        return self.wait_task(f"{PACKAGE_MANAGER}/tasks", _task_id(response), **wait_kwargs)

    def install_file(self, path: Path, **wait_kwargs: Any) -> dict[str, Any]:
        """Upload and install (or update) one app file."""
        with open(path, "rb") as handle:
            response = self.request(
                "POST",
                f"{PACKAGE_MANAGER}/packages",
                files={"file": (path.name, handle, "application/octet-stream")},
                timeout=1800,
            )
        return self.wait_task(f"{PACKAGE_MANAGER}/tasks", _task_id(response), **wait_kwargs)

    def uninstall(self, package_id: str, **wait_kwargs: Any) -> dict[str, Any]:
        """Remove an installed app. Its configuration in Solutions stays."""
        response = self.request(
            "POST",
            f"{PACKAGE_MANAGER}/tasks",
            json={"action": "uninstall", "parameters": {"id": package_id}},
        )
        return self.wait_task(f"{PACKAGE_MANAGER}/tasks", _task_id(response), **wait_kwargs)

    # -- App Storage ---------------------------------------------------------
    # Not in the public OpenAPI spec (packagemanager 1.15.0); taken from the
    # ctrlX OS 4.6 web UI ("Settings > Apps > App Storage").

    def app_storage_active(self) -> bool:
        repositories = self.get_json(f"{PACKAGE_MANAGER}/repositories") or []
        return any(repo.get("id") == APP_STORAGE["id"] for repo in repositories)

    def activate_app_storage(self) -> None:
        if not self.app_storage_active():
            self.request("POST", f"{PACKAGE_MANAGER}/repositories", json=APP_STORAGE)

    def app_storage_packages(self) -> list[dict[str, Any]]:
        if not self.app_storage_active():
            return []
        path = f"{PACKAGE_MANAGER}/repositories/{APP_STORAGE['id']}/packages"
        return self.get_json(path) or []

    def app_storage_upload(self, path: Path) -> Any:
        with open(path, "rb") as handle:
            response = self.request(
                "POST",
                f"{PACKAGE_MANAGER}/repositories/{APP_STORAGE['id']}/packages",
                files={"fileName": (path.name, handle, "application/octet-stream")},
                timeout=1800,
            )
        return response.json() if response.content else None

    def install_from_app_storage(
        self, package_id: str, version: str, update: bool, **wait_kwargs: Any
    ) -> dict[str, Any]:
        """Install or update one app from App Storage."""
        response = self.request(
            "POST",
            f"{PACKAGE_MANAGER}/tasks",
            json={
                "action": "update" if update else "install",
                "parameters": {
                    "id": package_id,
                    "version": version,
                    "location": f"repository://{APP_STORAGE['id']}",
                },
            },
        )
        return self.wait_task(f"{PACKAGE_MANAGER}/tasks", _task_id(response), **wait_kwargs)

    def app_storage_delete(self, package_id: str, version: str) -> None:
        self.request(
            "DELETE",
            f"{PACKAGE_MANAGER}/repositories/{APP_STORAGE['id']}/packages/{package_id}/releases/{version}",
        )

    # -- operation mode ------------------------------------------------------

    def mode(self) -> str:
        return self.datalayer_read(MODE_NODE)["value"]["state"]

    def set_mode(self, target: str, timeout: float = 300, poll: float = 2.0) -> str:
        """Switch to SERVICE, SETUP, or OPERATING, one step at a time.

        The order is OPERATING <-> SETUP <-> SERVICE, as in the web UI.
        """
        target = target.upper()
        if target not in MODES:
            raise CtrlxError(f"Unknown mode {target}; use one of {', '.join(MODES)}")
        current = self.mode()
        while current != target:
            step = MODES[MODES.index(current) + (1 if MODES.index(target) > MODES.index(current) else -1)]
            self.request(
                "PUT",
                f"{DATALAYER}/nodes/{MODE_NODE}",
                json={"type": "object", "value": {"state": step}},
            )
            deadline = time.monotonic() + timeout
            while (current := self.mode()) != step:
                if time.monotonic() > deadline:
                    raise CtrlxError(f"Mode did not change to {step} in {timeout} s")
                time.sleep(poll)
        return current

    # -- setup (backup and restore) -----------------------------------------

    def create_backup(
        self, password: str | None = None, include_apps: bool = True, **wait_kwargs: Any
    ) -> dict[str, Any]:
        """Create a backup of settings, certificates, and configurations.

        include_apps=False leaves out the app files (only the app list is kept).
        Use it when the data disk has no space for a full backup.
        """
        body: dict[str, Any] = {}
        if password:
            body["$password"] = password
        if not include_apps:
            body["packageManagement"] = {"$content": "items"}
        response = self.request("POST", f"{SETUP}/create", json=body)
        return self.wait_task(f"{SETUP}/tasks", _task_id(response), **wait_kwargs)

    def apply_setup(
        self, setup_file: Path, mode: str = "merge", password: str | None = None, **wait_kwargs: Any
    ) -> dict[str, Any]:
        """Apply a setup/backup file. `merge` keeps settings that are not in the file."""
        data = {"mode": mode}
        if password:
            data["password"] = password
        with open(setup_file, "rb") as handle:
            response = self.request(
                "POST",
                f"{SETUP}/apply",
                files={"setup": (setup_file.name, handle, "application/zip")},
                data=data,
                timeout=1800,
            )
        return self.wait_task(f"{SETUP}/tasks", _task_id(response), **wait_kwargs)

    def list_setups(self) -> Any:
        return self.get_json(f"{SETUP}/setups")

    def download_setup(self, file_name: str, target: Path) -> Path:
        response = self.request("GET", f"{SETUP}/setups/{file_name}", stream=True, timeout=1800)
        target.parent.mkdir(parents=True, exist_ok=True)
        with open(target, "wb") as handle:
            for chunk in response.iter_content(chunk_size=1 << 20):
                handle.write(chunk)
        return target

    # -- tasks ---------------------------------------------------------------

    def wait_task(
        self,
        tasks_path: str,
        task_id: str,
        timeout: float = 3600,
        poll: float = 3.0,
        on_progress: Callable[[dict[str, Any]], None] | None = None,
    ) -> dict[str, Any]:
        """Poll a task until it is done or failed.

        The device can restart services during system app updates;
        connection errors and 502/503/504 during polling are retried until the timeout.
        """
        deadline = time.monotonic() + timeout
        while True:
            try:
                task = self.get_json(f"{tasks_path}/{task_id}")
            except (requests.ConnectionError, requests.Timeout):
                task = {"id": task_id, "state": "unreachable"}
            except CtrlxError as error:
                # 502/503/504: web server is up, the app behind it still starts.
                if error.status_code not in (502, 503, 504):
                    raise
                task = {"id": task_id, "state": "starting"}
            if on_progress:
                on_progress(task)
            if task.get("state") == "done":
                return task
            if task.get("state") == "failed":
                raise CtrlxError(f"Task {task_id} failed: {task.get('result')}")
            if time.monotonic() > deadline:
                raise CtrlxError(f"Task {task_id} did not finish in {timeout} s")
            time.sleep(poll)


def _task_id(response: requests.Response) -> str:
    """Task ID from the JSON body or from the Location header (/tasks/<id>)."""
    if response.content:
        try:
            return response.json()["id"]
        except (ValueError, KeyError, TypeError):
            pass
    location = response.headers.get("Location", "")
    if not location:
        raise CtrlxError(f"No task ID in response of {response.request.method} {response.url}")
    return location.rstrip("/").rsplit("/", 1)[-1]


def _raise_for_status(response: requests.Response) -> None:
    if response.ok:
        return
    detail = response.text[:500]
    raise CtrlxError(
        f"{response.request.method} {response.url} -> {response.status_code}: {detail}",
        response.status_code,
    )
