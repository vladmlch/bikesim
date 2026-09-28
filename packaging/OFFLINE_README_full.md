# bike-sim — offline bundle (Linux x86_64, Python 3.13)

Self-contained package: works without internet access.

## Contents

- `project/` — full project source tree (src, tests, tools, docs, XML models)
- `wheelhouse/` — all dependency wheels for linux x86_64 / CPython 3.13 + `bike_sim` wheel
- `requirements.txt` — pinned locked versions with sha256 hashes (from `uv.lock`)
- `install.sh` — creates a venv and installs everything from `wheelhouse/` only

## Install

```bash
tar -xzf bike-sim-offline-py313-linux-x64.tar.gz
cd bike-sim-offline-py313-linux-x64
./install.sh            # creates ./.venv
source .venv/bin/activate
bike-sim --help
```

Requirements on the target machine:

- Python 3.13 (`python3` or `python3.13` in PATH)
- `python3.13-venv` (on Debian/Ubuntu) for `python3 -m venv` to work
- For the interactive MuJoCo viewer (`bike-playground`, `bike-ride`): OpenGL/X11
  system libraries. In headless environments use EGL/OSMesa
  (`MUJOCO_GL=egl` / `MUJOCO_GL=osmesa`) or run non-viewer commands.

## Rebuilding the bundle (on a machine with internet)

```bash
uv build
uv export --locked --no-dev --no-emit-project -o requirements.txt
pip download -r requirements.txt -d wheelhouse \
  --only-binary=:all: --python-version 3.13 --implementation cp \
  --abi cp313 --abi abi3 --abi none \
  --platform manylinux_2_17_x86_64 --platform manylinux2014_x86_64 \
  --platform manylinux_2_24_x86_64 --platform manylinux_2_26_x86_64 \
  --platform manylinux_2_27_x86_64 --platform manylinux_2_28_x86_64 \
  --platform linux_x86_64
```
