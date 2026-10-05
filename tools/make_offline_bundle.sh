#!/usr/bin/env bash
# Build offline bundles (full + slim) for Linux x86_64 / CPython 3.13.
# Produces bike-sim-offline-py313-linux-x64{,-slim}.tar.gz in the repo root.
# Run on any machine with internet access; the bundles then install with --no-index.
set -euo pipefail
cd "$(dirname "$0")/.."

PYVER=3.13
PLATFORMS=(
  manylinux_2_17_x86_64 manylinux2014_x86_64 manylinux_2_24_x86_64
  manylinux_2_26_x86_64 manylinux_2_27_x86_64 manylinux_2_28_x86_64
  linux_x86_64
)
# Assumed to be preinstalled on the slim-bundle target machine
# (their exclusive transitive deps are dropped too).
SLIM_DROP="numpy scipy matplotlib pytest contourpy cycler fonttools \
kiwisolver pillow pyparsing python-dateutil six iniconfig packaging \
pluggy pygments colorama"

WORK=.work/offline-build
rm -rf "$WORK"
mkdir -p "$WORK"

echo "==> uv build"
uv build --out-dir "$WORK/dist"

echo "==> uv export (locked, no project itself)"
uv export --locked --no-dev --no-emit-project -o "$WORK/requirements.txt"

echo "==> downloading wheels for linux x86_64 / cp$PYVER"
PLATFORM_ARGS=()
for p in "${PLATFORMS[@]}"; do PLATFORM_ARGS+=(--platform "$p"); done
python3 -m pip download -r "$WORK/requirements.txt" -d "$WORK/wheelhouse" \
  --only-binary=:all: --python-version "$PYVER" --implementation cp \
  --abi "cp${PYVER/./}" --abi abi3 --abi none "${PLATFORM_ARGS[@]}"

WHEEL=$(basename "$WORK"/dist/bike_sim-*-py3-none-any.whl)
cp "$WORK/dist/$WHEEL" "$WORK/wheelhouse/"

copy_project() {
    rsync -a \
      --exclude '.git' --exclude '.venv' --exclude 'output' --exclude '/dist*/' \
      --exclude '__pycache__' --exclude '.pytest_cache' --exclude '*.egg-info' \
      --exclude '.DS_Store' --exclude '.idea' --exclude '.agents' --exclude '.claude' \
      --exclude '.codex' --exclude '.junie' --exclude '.superpowers' --exclude '.ai' \
      --exclude '.mcp.json' --exclude '.work' --exclude 'skills-lock.json' \
      --exclude 'bike-sim-offline*' --exclude 'packaging' \
      ./ "$1/project/"
}

# ---------- full bundle ----------
echo "==> assembling full bundle"
FULL="bike-sim-offline-py${PYVER/./}-linux-x64"
rm -rf "$FULL" "$FULL.tar.gz"
mkdir -p "$FULL"
copy_project "$FULL"
cp -r "$WORK/wheelhouse" "$FULL/wheelhouse"
cp "$WORK/requirements.txt" "$FULL/requirements.txt"
cp packaging/install_full.sh "$FULL/install.sh"
cp packaging/OFFLINE_README_full.md "$FULL/OFFLINE_README.md"
chmod +x "$FULL/install.sh"
python3 tools/check_offline_sources.py --source "$FULL/project/src" --wheel "$FULL/wheelhouse/$WHEEL"

# ---------- slim bundle ----------
echo "==> assembling slim bundle"
python3 - "$WORK/requirements.txt" "$WORK/req-slim.txt" $SLIM_DROP <<'EOF'
import re, sys
src, dst, *drop = sys.argv[1:]
drop = {d.lower() for d in drop}
lines = open(src).read().splitlines()
out, keep, started = [], True, False
for ln in lines:
    m = re.match(r"^([A-Za-z0-9_.-]+)==", ln)
    if m:
        keep = m.group(1).lower().replace("_", "-") not in drop
        started = True
    if keep or not started:
        out.append(ln)
open(dst, "w").write("\n".join(out) + "\n")
EOF

SLIM="$FULL-slim"
rm -rf "$SLIM" "$SLIM.tar.gz"
mkdir -p "$SLIM/wheelhouse"
copy_project "$SLIM"
for whl in "$WORK"/wheelhouse/*.whl; do
    name=$(basename "$whl" | cut -d- -f1 | tr 'A-Z_' 'a-z-')
    drop=0
    for d in $SLIM_DROP; do [ "$name" = "$d" ] && drop=1; done
    [ "$drop" = 0 ] && cp "$whl" "$SLIM/wheelhouse/"
done
cp "$WORK/req-slim.txt" "$SLIM/requirements.txt"
cp packaging/install_slim.sh "$SLIM/install.sh"
cp packaging/OFFLINE_README_slim.md "$SLIM/OFFLINE_README.md"
chmod +x "$SLIM/install.sh"
python3 tools/check_offline_sources.py --source "$SLIM/project/src" --wheel "$SLIM/wheelhouse/$WHEEL"

echo "==> packing"
tar -czf "$FULL.tar.gz" "$FULL"
tar -czf "$SLIM.tar.gz" "$SLIM"
ls -lh "$FULL.tar.gz" "$SLIM.tar.gz"
