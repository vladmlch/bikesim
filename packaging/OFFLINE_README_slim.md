# bike-sim — offline bundle, SLIM (Linux x86_64, Python 3.13)

Self-contained package: works without internet access.
**Slim variant** — reuses packages already installed on the target machine.

## Assumes pre-installed system packages

- `numpy>=1.24.0`
- `scipy>=1.18.0`
- `matplotlib>=3.7.0`
- `pytest>=7.0.0`

`install.sh` checks them up front and fails with a clear message if versions
don't fit. If the target machine does not have them, use the full bundle
(`bike-sim-offline-py313-linux-x64.tar.gz`) instead.

## Contents

- `project/` — full project source tree (src, tests, tools, docs, XML models)
- `wheelhouse/` — wheels only for what's missing: `mujoco` + its deps
  (`absl-py`, `etils`, `fsspec`, `typing-extensions`, `zipp`, `pyopengl`),
  `glfw`, plus the `bike_sim` wheel itself
- `requirements.txt` — pinned versions with sha256 hashes for those wheels
- `install.sh` — creates a venv with `--system-site-packages` and installs
  everything else from `wheelhouse/` only (`--no-index`)

## Install

```bash
tar -xzf bike-sim-offline-py313-linux-x64-slim.tar.gz
cd bike-sim-offline-py313-linux-x64-slim
./install.sh            # creates ./.venv
source .venv/bin/activate
bike-sim --help
```

Requirements on the target machine:

- Python 3.13 (`python3` or `python3.13` in PATH) + the packages listed above
- `python3.13-venv` (on Debian/Ubuntu)
- For the interactive MuJoCo viewer (`bike-playground`, `bike-ride`): OpenGL/X11
  system libraries; headless: `MUJOCO_GL=egl`/`osmesa` or non-viewer commands.
