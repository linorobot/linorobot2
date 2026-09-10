# Console's own Docker Compose stack

`linorobot2_console` runs Docker/Podman install mode entirely from **this**
directory — it never writes into the repo's top-level `docker/` dir.

- `docker-compose.yaml` — the console's stack. Reuses the upstream
  `linorobot2:<BASE_IMAGE>` image + `docker/Dockerfile` (built, not modified)
  and drives navigation/SLAM through the console's own
  `launch_nav2.py` / `launch_bringup.py`.
- `.env` — written by the console from the Install tab (gitignored).
- `devices.generated.yaml` — written by the console: the base-serial + lidar
  device mappings for the `bringup` service (gitignored).

The console invokes it as:

```
docker compose --env-file .env \
  -f docker-compose.yaml -f devices.generated.yaml <build|up|down> <service>
```
