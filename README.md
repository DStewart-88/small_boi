## Disabled-Only HTTP Log Server

- Port: `5800` (default). Binds only while the robot is Disabled.
- Log directory: prefers `/U/logs` (USB) if present; otherwise `/home/lvuser/logs`.
- Allowed extensions: `.wpilog`, `.hoot` (top-level files only).
- Endpoints:
  - `GET /` → health text.
  - `GET /status.json` → `{ "enabled": bool, "time": ISO-8601 }`.
  - `GET /logs.json` → JSON array of filenames (newest-first).
  - `GET /<filename>` → raw bytes with exact `Content-Length`.
- Safety: The server is fully stopped in Auto/Teleop/Test. If a request arrives during an enable transition, it returns `503` when `DriverStation.isEnabled()` is true.

