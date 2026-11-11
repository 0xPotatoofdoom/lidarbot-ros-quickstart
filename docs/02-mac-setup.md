# 2. Mac setup

## Docker Desktop

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) for Apple Silicon.
2. **Settings > Resources > Network > Enable host networking**, then restart Docker Desktop.
   Without it the container can't bind the Mac's ports.

If `docker` says `docker-credential-desktop not found`, your shell doesn't have Docker's
helpers on its PATH. The scripts here add
`/Applications/Docker.app/Contents/Resources/bin` themselves; for your own commands, add
it to your shell profile.

> If your terminal runs under Rosetta, `uname -m` reports `x86_64` even on an M-series
> Mac. `sysctl hw.optional.arm64` tells you the truth. The image builds natively for
> arm64 either way.

## Python (for the Mac-side tools)

Two tools run natively on the Mac: the robot config tool and the UDP relay. They need
Python 3.10+ and pyserial in a virtual env at `.venv`:

```bash
uv venv .venv
uv pip install --python .venv pyserial
```

(`python3 -m venv .venv && .venv/bin/pip install pyserial` works too.)

## Yahboom's ROS packages

The container builds four of Yahboom's packages: `microros_v2_bringup` (launch files,
drivers, configs), `robot_description` (URDF and meshes), `interfaces` and `common_demo`.
They're under Yahboom's license, so this repo downloads them instead of including them:

```bash
./fetch-yahboom
```

This downloads `mircoros_v2-1.0.1.zip` from Yahboom's Google Drive, checks its SHA-256,
copies the four packages into `vendor/yahboom/src`, and applies [patches/](../patches):

- `0001`: lets the launch file start the agent on TCP 8888 (for the relay) instead of UDP 8090.
- `0002`: drops a dependency on `smart_tracker`, a package that isn't in the zip.

It also deletes two files Yahboom shipped with their own credentials in them
(`cli/config_robot` and `config/multi_brains_pre_setting.yaml`); nothing here uses them.

Google Drive limits how often a shared file can be downloaded each day. If the script
says the quota is exceeded, open the URL it prints in a browser where you're signed in
to Google, save the zip, and run:

```bash
./fetch-yahboom --zip ~/Downloads/mircoros_v2-1.0.1.zip
```

If the checksum fails, Yahboom has published a new version. Open an issue, or try
updating `SHA256` in the script and check that the patches still apply.

## Build the image

```bash
docker compose build
```

The first build takes several minutes (it compiles the micro-ROS agent). The image is
`lidarbot-ros:humble`.

## Local settings

```bash
cp config/robot_info.example.json config/robot_info.json
```

Set `camera_ip` once you know it (next chapter), or `""` if your kit has no Wi-Fi camera.

Two settings live in `docker-compose.yml` and can be overridden from the environment:

| Variable | Default | Meaning |
|---|---|---|
| `ROS_DOMAIN_ID` | `77` | Must match the domain ID you give the robot |
| `CAR_TYPE` | `ptz` | `ptz` (Superior kit), `no_ptz` or `basic_car`; picks the robot model |
