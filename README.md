## Disabled-Only HTTP Log Server

- Port: `5801` (binds only while Disabled).
- Log directory: prefers `/U/logs` (USB) if present; otherwise `/home/lvuser/logs`.
- Allowed extensions: `.wpilog`, `.hoot` (top-level files only).
- Endpoints:
  - `GET /` → health text.
  - `GET /status.json` → `{ "enabled": bool, "time": ISO-8601 }`.
  - `GET /logs.json` → JSON array of filenames (newest-first).
  - `GET /<filename>` → raw bytes with exact `Content-Length`.
- Safety: The server is fully stopped in Auto/Teleop/Test. If a request arrives during an enable transition, it returns `503` when `DriverStation.isEnabled()` is true.

## Elastic Remote Layout Download

- Port: `5800`, serving the deploy directory via `WebServer.start(...)`.
- Layout file path: `src/main/deploy/elastic/can-status.json` (deploys to `/home/lvuser/deploy/elastic/can-status.json`).
- URL pattern: `http://<roborio-hostname>:5800/elastic/can-status.json` (e.g., `roborio-####-frc.local`).
- Replace the placeholder JSON with your exported Elastic layout that targets `/CANHealth/<DeviceKey>` boolean topics.
