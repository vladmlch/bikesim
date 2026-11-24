# agy in an Apple `container` instance: keep its state on the host

agy stores every conversation (task.md, plans, the `brain/` artefacts) under
`$HOME/.gemini/antigravity-cli/`. Inside a container that is `/root/.gemini`,
so the artefacts die with the container unless they are copied out by hand
(`container cp fedora:/root/.gemini/antigravity-cli/brain/<id>/task.md ...`,
2026-11-17). agy has no flag for its data dir; mount the directory instead.

`tools/linux/container.sh` is the Docker lane and is disabled by its
plan-change guard; this recipe is for the ad-hoc Apple `container` instance
(the `fedora` one) used for agy sessions.

## Create the instance with the mounts

```bash
repo=/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean
state=/Volumes/SSD_1TB_M2/work/agy-container-gemini   # or ~/.cache/agy-container-gemini without the SSD
mkdir -p "$state"
container system start            # if `container ls` reports an XPC error
scratch=$(cd -P "$repo/.scratch" && pwd)               # .scratch is a symlink (SSD)
skills=/Users/vladislav.molchanov/Desktop/Projects_my/llm-orchestrators
container run -d --name fedora \
  --volume "$repo:$repo" \
  --volume "$scratch:$scratch" \
  --volume "$skills:$skills:ro" \
  --volume "$state:/root/.gemini" \
  <image> sleep infinity
container exec -it fedora bash
```

- The repo is mounted at its host path, so paths in reports and briefs mean
  the same file on both sides (that is why `.work/` survived on 2026-11-17).
- `.scratch` and the project skill links (`.claude/skills/*`,
  `.agents/skills/*`) are absolute symlinks to host paths; mounting their
  targets at the same paths keeps them resolving inside the container.
  `.deps` (darwin-arm64 libraries) is not mounted: it is useless in a Linux
  container.
- An existing instance cannot gain a mount: recreate it (`container stop`,
  `container rm`, then the `run` above). Copy anything still only inside it
  first.
- Log in to agy once inside; the credentials then live in `$state` too, so
  treat that directory like `~/.gemini` (private, not in the repo).

## Check

```bash
container exec fedora ls /root/.gemini/antigravity-cli/brain | head
ls "$state/antigravity-cli/brain" | head    # same ids on the host
```
