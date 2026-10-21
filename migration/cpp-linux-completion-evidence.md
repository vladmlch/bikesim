# C++ + Linux completion: evidence log

Append-only evidence for
docs/superpowers/plans/2026-10-20-cpp-linux-completion.md.
Historical measurements in the plan are not measurements of this patch.
Executed gates must record the source revision, host/image identity,
commands, exit codes, test counts and failing node IDs.

## 2026-10-21 — source-only Phase 1 implementation session

Input: repomix-output.xml(20261021-145241).zip.
Input SHA-256: 8226e739c82198a9ce9bdf34e1b3e4581c0e5bb34deac1ca1272dfc78738e0b9.
The Repomix-unmixer skill
restored 1,992 text files into an isolated working copy. Original extracted
files were retained separately for patch generation. Git objects, branch
refs, omitted binaries and dependency environments are not part of this
snapshot; its Git HEAD cannot be established from the archive.

The current user instruction prohibits compilation and execution.
Accordingly no project configure, build, test, static-analysis sweep,
interpreter import, container start, numerical probe, simulation, Git
merge or push is executed in this session. Filesystem extraction,
source inspection, editing and deliverable packaging are the evidence.
No L1/L2/L3 pass or current-platform qualification is claimed.

### Implementation work allocation

- Tasks 1.01–1.05: existing-fix inspection, Linux lane, migration policy,
  source-supported finding review and honest evidence bookkeeping.
- Tasks 1.06–1.09: checker schema/platform/tool/control/child-configure
  parity, plus the depth-2 script sweep and provider-binding regressions.
- Tasks 1.10–1.12, 1.17–1.18: numerics probe/face/callers, binary64
  feasibility, and measurement preparation for non-finite trigonometry.
- Tasks 1.13–1.16: recursive Value ownership, Linux source branches,
  explicit lock types and GCC source compatibility.
- Task 1.19 and checkpoints: static integration review here; runtime
  verification remains pending under the current user restriction.

The existing detailed plan is the implementation specification; its
gate-dependent checkboxes and historical plans remain unchanged.

### Task 1.01 — inherited fixes

The supplied BikeLinkage.cmake already guards optional ARGV1 with ARGC,
then falls back to _native_mujoco_library for one-argument callers.
The supplied io/value.hpp already includes <algorithm>.
These are input facts, not new fixes or newly verified build results.

### Task 1.03 — missing merge input

The archive has only ADR-0001, no ADR-0002, no tools/research/ payload
from feat/drop-oracle-parity and no Git objects for either branch.
No substitute merge, invented 22-file manifest or fabricated commit is
created. Task 1.03 remains pending in the real checkout. The known D12
transition is recorded directly in ADR-0003 and the active contracts;
ADR-0002's original text/status must be reconciled when its actual merge
lands, retaining ADR-0003's precedence.

### Task 1.05 — source-supported fixes, verification still open

The snapshot's PhysicalStep now calls cruise.compute() for the admitted
ideal-speed mode (F2), adds resistance from refreshed engine contacts
when no tire writer exists (F3), and passes the configured recovery
switch to the rider controller (F5).
The corresponding Python physical_runtime.py branches and test_t23.cpp
regressions support the intended changes by source inspection.
The completion plan attributes those corrections to 89cfcb1; that Git
object is not available in this archive.

No regression was run on either graph. The 73 python-removal rows and
four physics-research task records therefore retain their existing
implemented_tested_findings_open statuses and findings_open arrays.
Nothing is promoted to tested, verified, qualified or closed.

### Task 1.02 — acquisition provenance, not runtime qualification

The Dockerfile records metadata fetched read-only on 2026-10-21:

- Ubuntu 24.04 index digest:
  sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac.
  Source: https://hub.docker.com/layers/library/ubuntu/24.04/images/sha256-7f622ca8766bccb22f04242ecb6f19f770b2f08827dc4b8c707de5e78a6da7ab
  (the current page reports the index digest above).
- uv 0.13.1 aarch64 GNU/Linux archive SHA-256:
  9041c73c99fdc5892cf128140beb354eca9231d502068cc36e6cb53a8e59ad14.
  Source: https://releases.astral.sh/github/uv/releases/download/0.13.1/uv-aarch64-unknown-linux-gnu.tar.gz.sha256

No image or executable archive was run to obtain these values. The image
was not built or inspected with Docker. Its package versions, Python
runtime and numerical provider still require the planned lane checks.

### Source implementation review — Tasks 1.02, 1.04, 1.06–1.19

The following is a source-review status table. "Implemented" describes
the edited source and regression cases; it does not mean a task's
execution gate passed.

| Task | Source result | Outstanding requirement |
|---|---|---|
| 1.01 | Both fixes were already present in the supplied snapshot. | Rebuild the affected graphs when execution is authorized. |
| 1.02 | Added the pinned Linux image recipe and checkout-owned container driver; both script sweeps select depth 2 and reject an empty selection. | Build/provision the image, record actual versions and qualify the lane. |
| 1.03 | No merge was attempted; required branch objects, ADR-0002 and research payload are absent. | Perform the specified merge from the real branch and reconcile its actual files. |
| 1.04 | Added ADR-0003 and updated AGENTS.md, native engineering rules and CONTEXT.md to the agreed per-file golden transition. | Amend the actual ADR-0002 when Task 1.03 is possible. |
| 1.05 | Reviewed the existing F2/F3/F5 source fixes and regression definitions. | Run the required regressions on both graphs before changing any open finding/status. |
| 1.06 | Added schema-2 platform/SDK/sysroot/stdlib/policy/capability validation and context-only CLI validation. | Run T56 and the affected sweeps on the supported platforms. |
| 1.07 | Added optional sysroot validation/forwarding and the reference tool candidate chain, preserving executable symlink spelling. | Run tool/sysroot regression cases and sweeps. |
| 1.08 | Added shared platform control dispatch/listing and the five Linux context controls; Darwin-only controls remain gated. | Execute each platform's control set. |
| 1.09 | Child configure uses CMake directly and only the recorded binding interpreter; added Linux standard-library hardening controls. | Run the binding-context L1 command and hardened/unhardened controls. |
| 1.10 | Probe, record validators, CMake projection and fixtures now require eight operations with exact spellings and separate ABI classes. | Regenerate real Darwin and Linux lock entries; the generated lock is intentionally unchanged here. |
| 1.11 | Added typed raw ILP64 dot, unconjugated complex dot, dgesv and dsyevd adapters and analytic contract cases. | Measure the raw-dot zero bits and execute adapter/provider contracts. |
| 1.12 | Replaced all five direct caller bindings with the typed faces; added structural provider-boundary checks and necessary CMake usage requirements. | Build/link both graphs and run the affected bitwise and structural regressions. |
| 1.13 | Applied the plan's primary ObjectList solution and added aggregate/ownership/copy/move/lexeme guards. | Confirm the diagnosed incomplete-pair chain is resolved on both Linux compilers and preserve macOS behavior; no boxing fallback was selected without that evidence. |
| 1.14 | Corrected the glibc spawn-chdir gate, bounded executable-path construction and spawn lock type. | Compile the platform objects and run T06. |
| 1.15 | Supplied explicit mutex template arguments at 37 lock declarations, retaining the recursive mutex where required. | Run the affected compile/sweep/runtime gates. |
| 1.16 | Made the tyre-tier table constexpr with string_view names, removed the duplicate research-startup default argument and used a stateless FILE deleter. | Compile both graphs and run the covering tests. |
| 1.17 | Changed interval storage/arithmetic to binary64 while retaining every outward down/up/nextafter step; added boundary and one-ULP guards. | Run feasibility/native regression gates on both platforms. |
| 1.18 | Added analytic native sin/cos contracts, a native/libm/subtraction bit-table mode and scalar/contiguous-array/native NumPy comparison support. | Measure both platforms, freeze the actual tables, and only then decide the Linux nonfinite branch. The existing branch is retained. |
| 1.19 | Reviewed source integration, new interfaces, callers, CMake edges and regression fixtures. | Discover and resolve remaining compiler/runtime findings through the specified Linux/macOS gates. |

No task/checkpoint checkbox in the completion plan was marked complete.
Checkpoints 1-A/1-B and Phase 1's final acceptance remain pending. Linux
root-graph/C++ checker execution still belongs to Phase 2; Phase 1's
Linux binding sweeps use the Python commands specified by Task 1.19.

#### Integration decisions and independent review

- The numerics interface keeps ordinary `blas::ddot`'s existing `0. +`
  seed and Darwin legacy symbol. `blas::ddot_raw` forwards the original
  unseeded ILP64 fold; its platform-specific zero sign was not guessed.
- Probe/discovery distinguishes legacy BLAS, always-ILP64 BLAS and
  LAPACK. New LAPACK and dot bindings must belong to the corresponding
  NumPy import set as well as the provider's export set.
- Geometry's inline functions require `Bike::Numerics` as a PUBLIC
  dependency in both graphs. Model, kinematics and filter callers carry
  explicit adapter edges. The adapter exports the MuJoCo include
  requirement of its public CBLAS header; generated provider names stay
  private to the backend translation units.
- Checker validation/control code was compared directly with
  `tools/native_checks.py` and `tools/native_toolchain.py`. The fixtures
  use the actual Python messages where the plan's prose was stale:
  `missing configured sysroot` and `invalid <kind> override`.
- The standard-library control is deliberately stricter than the current
  Python implementation in one case: an unhardened probe that does not
  compile fails the C++ sweep. A failed compile cannot establish that
  hardening flags did not leak into the negative control.
- The contracts command's default repository-root traversal was one
  parent short. It now reaches the repository root; a T56 case covers
  invocation without `--repo-root`.
- Independent source reviews covered checker fixtures/CLI, typed
  numerical ABI signatures/callers/CMake dependencies, Value ownership
  and Linux scripts. No new ABI-width, argument-order or fixture blocker
  was identified by those reviews. This is not compilation evidence.
- The driver stages source selection/archive creation before touching
  container sources, checks checkout ownership, excludes host dependency
  and build directories, and rejects symlink parents. It retains Ninja's
  failure status and does not publish a zero count for an unreadable log.
  Rebuilding an image does not replace an existing container: an explicit
  `down`/`sync`/`venv` cycle is documented.

#### Added regression coverage (not executed)

- T56: schema-2 valid and malformed contexts, platform controls/CLI modes,
  Linux sysroot rejection, override precedence, provider binding boundary,
  script depth/empty selection, and default repository-root resolution.
- T05: aggregate Value shape, insertion order, large integer and signed
  zero lexemes, object/array deep copies, copy/move assignments,
  moved-from reassignment, range/base-vector construction, null vs object.
- T28: a one-ULP contradictory interval cannot yield a feasible witness,
  and a boundary-feasible interval cannot be certified empty.
- Platform BLAS/LAPACK: raw/seeded dot separation, strides, unconjugated
  complex products, linear solve/singularity and eigenvalue workspace/
  eigenvector contracts.
- Dependency portability: all eight operation records, import
  classification/membership, exact-name rejection and platform entries.
- Nonfinite trigonometry: native analytic contracts and explicit measured
  table output, plus NumPy scalar/contiguous-array/native agreement.

#### Local suppression records

These entries record source-level intent. No checker was executed to
claim a new observed diagnostic in this session.

| Checker / location | Origin, reason and reproducer | Removal condition |
|---|---|---|
| clang-tidy `modernize-use-equals-default`, `native/src/io/value.hpp` | Task 1.13's recorded libstdc++ 13 chain: defaulted constexpr destruction reaches incomplete `pair<string, Value>`. Reproducer is the plan's isolated `io/json.cpp` object compile. Non-constexpr out-of-line bodies are the prescribed fix. | Qualified Linux compilers accept the defaulted form and the Value/config/JSON copy/move guards pass. |
| clang-tidy `misc-redundant-expression`, `native/tests/test_platform_sincos.cpp` | Task 1.18 explicitly requires measuring `x-x` as a candidate nonfinite rule. The source expression in `dump_nonfinite` is intentional; its four inputs are the two infinities and signed quiet NaNs. | The candidate measurement is no longer required or the checker recognizes this intentional probe without an exception. |
| Clang `-Wunsafe-buffer-usage-in-container`, `native/tests/test_platform_sincos.cpp` | The CLI constructs a bounded span from C/C++ `main`'s `argv` and `argc`, then inspects only admitted indices. Reproducer is that single construction under the project's warning set; the local pragma matches the existing test CLI pattern. | The supported compiler accepts the bounded entry-point adapter, or an equally bounded adapter can express it without a local diagnostic exception. |
| ShellCheck `SC2016`, `tools/linux/container.sh` | The inner Bash snippets intentionally defer variables/positionals to the container process. The annotated sync/build/failures command strings are the reproducer; host interpolation would alter their meaning. | The snippets no longer cross a shell boundary, or ShellCheck can recognize the deferred interpretation. |

#### Qualification handoff — commands below were not executed

First obtain the missing branch payload for Task 1.03 and provision the
two exact dependency environments. The extended validator rejects the
old four-operation record even on Darwin. Do not add symbol names or
Linux provider facts by hand and do not configure against that stale
lock expecting a qualified build.

In the real macOS checkout, with its `.venv` and the Linux container
already provisioned, regenerate the record through the actual probe:

~~~sh
numerics_backup_dir="$(mktemp -d "${TMPDIR:-/tmp}/bike-numerics-lock.XXXXXXXX")"
mv native/numerics.lock.json "$numerics_backup_dir/numerics.lock.json"
uv run --frozen --group native python tools/native_dependencies.py \
  --python .venv/bin/python3 \
  --write-numerics-record native/numerics.lock.json
tools/linux/container.sh sync
tools/linux/container.sh exec uv run --frozen --group native python \
  tools/native_dependencies.py \
  --python /opt/bike-venv/bin/python3 \
  --write-numerics-record native/numerics.lock.json
tools/linux/container.sh pull native/numerics.lock.json
~~~

Moving the old lock aside is necessary because the writer validates an
existing file before preserving other platforms' entries. The first
successful probe writes the new Darwin record; the Linux probe then
preserves it while appending its measured record. Retain the backup
until both new records are reviewed.

After the affected graph can build, record raw-dot bits and the two
platforms' scalar/array/native/libm/subtraction sin/cos tables. The new
`native_platform_sincos_tests --dump-nonfinite` mode and
`test_sin_cos_nonfinite_scalar_array_agreement` reference test provide
measurement output; neither is a frozen per-platform acceptance table.
Then follow the existing plan's focused regressions, Checkpoint 1-A,
Checkpoint 1-B and final Phase 1 gates. Record exact source/image
identities, commands, exits and failing cases before closing findings.

## Phase 2 source implementation — 2026-10-21 (execution prohibited)

Source: the user-supplied Repomix ZIP, SHA-256
`db0b8be6eac8a6e7abea6d4a000f11023532af770ec19d5cb9426d83a14f7ddb`.
The archive restored 1,997 text files and included the Phase-1 source
work and its open prerequisites recorded above. It contains no usable
Git branch history for the prescribed merge/commit/push sequence.

The user explicitly required that nothing be compiled or run. No
project configure, build, CTest, pytest, checker, ShellCheck, container,
application, viewer or numerical probe was executed. Filesystem edits,
archive extraction, byte/hash inspection and source comparisons are the
only verification evidence from this session. No runtime exit codes,
test counts or platform qualification are added.

### Source coverage

| Tasks | Source result | Open evidence |
|---|---|---|
| 2.01–2.02 | Shared dependency/platform/compiler-policy layers now drive the root graph. Root consumers use resolved shared-library paths; the policy is applied directly. | Darwin provenance/load-name parity and effective compiler-flag comparison. |
| 2.03a–2.03b | One schema-2 emitter serves both graphs. Root-only metadata and null Python fields have strict validation. The two unused legacy modules are deleted. | T56, binding-context parity, sweeps and regenerated build identities. |
| 2.04–2.05 | Schema-2 platform selection, locked Linux aarch64 artifacts, explicit acquisition and a narrow Docker acquire stage. Container initialization preserves existing dependency trees. | Actual acquisition/image provisioning and platform configure gates. |
| 2.06–2.07 | Linux RECORD_ONLY maps measured records to locked providers with glibc/hash/containment checks. Root provider declarations avoid Darwin-only probes. Linux T03 loader checks and root integration are prepared. | Real record regeneration and the first Linux root configure/build/CTest. |
| 2.08 | Relative installed rpaths, installed-only Darwin load-name/signature handling, Linux provider/runtime-library installation, moved-prefix smoke and strict root installed-provider identity. | Both-platform install smoke, T03 partial installs and native runtime verification. |
| 2.09–2.10 | Complete GLFW tree and X11 selection; native-only `--close-on-finish` for physical/research viewers; T47 definitions and bounded Xvfb script. | GUI builds and smoke. Desktop viewer acceptance remains separate. |
| 2.11 | The existing common gate runner is exposed through the container with explicit Clang/environment selection and wrong-compiler cache refusal. | L3 Linux, actual counts/failing IDs, qualification and repository publication. |

The detailed decisions, source-level compiler-policy comparison,
regression inventory and remaining requirements are in
[`cpp-linux-phase2-source-review.md`](cpp-linux-phase2-source-review.md).

### Dependency byte evidence

All eight selected Linux archives and all 17 selected required-file
hashes were checked against their bytes. Exact source URLs and identities
are committed in `native/dependencies.lock.json`; acquisition details are
in `native/dependencies.md`. Original Darwin pins are preserved. The
GLFW archive hash is unchanged; its top-level and `src/` CMake files are
now pinned and the whole source tree is retained.

Static ELF inspection established that the selected OpenBLAS image
requires its wheel's renamed libgfortran companion. Both are pinned and
installed together; neither was executed. Archive inspection also led to
the RapidJSON `license.txt` to `LICENSE` mapping and deterministic DuckDB
MIT-license extraction. These byte checks do not establish ABI or runtime
qualification. The acquisition script and Dockerfile were not run.

### Review decisions and preservation

- The actual context validators require only a numerics object and a
  nonempty backend. No additional nullable numerics fields or invented
  probe evidence were needed. Binding validation remains strict.
- The root migration numerics helper now uses the shared target on every
  platform; it no longer adds a raw Darwin framework edge on Linux.
- Installed-only Darwin signature repair is defined defensively after
  verification; no launch failure or measured signing requirement is
  claimed. Full `@rpath` names use one CMake substitution field.
- Linux installed-provider fallback is root-only, uses exact adjacent
  files and recorded hashes, and then checks real loaded-image membership.
  It addresses the identity API rather than asserting an unobserved CLI
  failure. Binding and configure-probe identity rules are unchanged.
- Both actual viewer launch consumers receive the new option. Research
  retains its runtime-owned duration. Nonzero GUI diagnostic and timeout
  statuses are propagated without an unmeasured success exception.
- Stale Linux-refusal numerics tests and Darwin-only root-test host guards
  were updated. The lane explicitly includes Make for generator-unspecified
  CMake fixtures as well as Ninja. All regression definitions are unrun.
- `native/numerics.lock.json`, `migration/cli-contracts.json`, the original
  completion plan, captured oracle/source manifests, historical baselines
  and viewer acceptance records retain their supplied content.

The numerics lock still contains the old four-operation Darwin record
and no Linux entry. Its real Phase-1 regeneration is a prerequisite;
it was not hand-edited to make an unrun configure appear acceptable.
The missing Task 1.03 branch payload remains an inherited prerequisite.

No task/checkpoint checkbox was changed. **Checkpoint 2-A and L3 Linux
(end of Phase 2) were not run and remain open.** No commit, push or new
qualified lane is claimed. The source delivery is ready for review and
the existing plan's execution gates when execution is authorized.

## 2026-10-21 — Task 1.03 source merge from supplied snapshots

The user supplied both `orig.xml.zip` and `drop-oracle-parity.xml.zip` and
requested the merge required by the completion plan. Input ZIP SHA-256:

- orig: `d76f46a7745130aa8d3ad8c9c56b94acbaeecdf86b4657d4a531a05833f22037`.
- drop-oracle-parity: `d30493e3fbe353a16c6256a65346b12073e0470a317283934dd3dfefe0ea9092`.

Repomix-unmixer restored 2,004 and 1,010 text files respectively. The
source merge imports the 144 donor-only docs/research-tool files and
recovers one expected JSON from the donor's preserved audit output.
It reconciles ADR-0002 with the already adopted ADR-0003, retaining the
original Phase-1/Phase-2 implementation and every original path.

This resolves the missing text-payload portion of Task 1.03 recorded
above. It does not establish Git ancestry or a real branch merge commit:
the supplied snapshots contain no branch objects. Earlier missing-input
entries remain historical, and no checkpoint or task gate is marked green.

All original native code, root/binding build graphs, dependency locks,
Python implementation/tests, active agent contracts, oracle manifests,
and historical plans retain their input bytes. The donor's older source
and build definitions do not overwrite orig. `research/fp32-physics` is
outside this merge. Historical r2 policy text is subordinate to ADR-0003:
live parity stays a per-file gate until both-platform capture and the
C++ port land; Python is removed at T63.

The nine compressed alpine snapshots and lateral source images/PDFs are
not present in the supplied text archives. The expected statistics JSON
is recovered from existing input bytes, not recalculated. Its generated
output ignore rule is anchored so the expected reference is trackable.
One overlong report filename is shortened without altering its text.

Only source inspection, filesystem extraction/copying, hashing and patch
application to temporary source copies are performed. No project code,
configure, build, test, checker, container, numerical probe or application
is executed. No L1/L2/L3 result or platform qualification is claimed.
The source review, provenance, availability limits and application notes
are in [cpp-linux-task103-source-merge.md](cpp-linux-task103-source-merge.md);
the exact imported-path/hash inventory is in
[cpp-linux-task103-merge-manifest.json](cpp-linux-task103-merge-manifest.json).

## Task 1.03 — Python files added by the r2 merge (OD-1)

Exactly 22 new files under `tools/research/`, copied unchanged from the
supplied donor snapshot: 17 OSM/DEM scripts, three lateral-digitisation
scripts and two adjudication scripts. This is the real input inventory,
not a reconstructed list from the plan. OD-1 remains pending until the
Task-3.02 triage; this merge does not delete or relocate these files.

- `tools/research/adjudication/audit_golden_cases.py`
- `tools/research/adjudication/gemini_golden_cases.py`
- `tools/research/alpine_osm_dem/analyze_profiles.py`
- `tools/research/alpine_osm_dem/bandfrac.py`
- `tools/research/alpine_osm_dem/compare_stats.py`
- `tools/research/alpine_osm_dem/dem_resolution_check.py`
- `tools/research/alpine_osm_dem/dense_check.py`
- `tools/research/alpine_osm_dem/descfrac.py`
- `tools/research/alpine_osm_dem/download_tiles.py`
- `tools/research/alpine_osm_dem/fetch_dem_index.py`
- `tools/research/alpine_osm_dem/fetch_osm_paths.py`
- `tools/research/alpine_osm_dem/noise_check.py`
- `tools/research/alpine_osm_dem/prepare.py`
- `tools/research/alpine_osm_dem/scree_analyze.py`
- `tools/research/alpine_osm_dem/scree_collect.py`
- `tools/research/alpine_osm_dem/scree_elev.py`
- `tools/research/alpine_osm_dem/scree_stats.py`
- `tools/research/alpine_osm_dem/scree_tiles.py`
- `tools/research/alpine_osm_dem/stats2.py`
- `tools/research/lateral_digitise/analyse.py`
- `tools/research/lateral_digitise/digitise.py`
- `tools/research/lateral_digitise/finalise.py`

## 2026-10-21 — Task 1.03 actual-checkout integration

Starting revision: `fc848de2d0d16389725b12337983bdf844e7e65c`.
Merged branch: `feat/drop-oracle-parity` at
`64c0a50043fd87a695e61a402572a63ee44fcfc4`.

The previously imported text files are now reconciled with the actual Git
branch. `git diff --name-status HEAD...feat/drop-oracle-parity -- native/
CMakeLists.txt` exited 0 with no changes. Initial `git merge-tree
--write-tree --name-only` exited 1 because the snapshot imports already
exist as separate additions. Repeating with `-Xignore-space-at-eol` reduced
the unresolved set to nine documentation/policy files. The actual merge
used `git merge --no-ff --no-commit -Xignore-space-at-eol
feat/drop-oracle-parity` (exit 1 pending those resolutions).

Conflict resolution preserves the current ADR-0003 transition, portable
reference-source lookup, native engineering rules, testing documentation,
anchored generated-output ignore rule and lateral-source availability
note. It restores all nine compressed alpine datasets from the branch.
Each restored file's `git hash-object` equals the branch's `git rev-parse
<branch>:<path>`; `gzip -t` exits 0 for all nine. The additional tyre/terrain
research report equals the user's pre-existing untracked file byte for
byte (blob `a0264f50f9d177903446891095b1538da84af456`); that file is now
tracked. The geomorphology report already exists under its shortened
portable name with identical text, so the duplicate 303-byte basename is
excluded. No existing container or worktree is removed.

Checks before the merge commit:

- no unresolved index entries;
- `git diff --cached --quiet HEAD -- native/ CMakeLists.txt`: exit 0;
- `git diff --cached --quiet HEAD -- docs/superpowers`: exit 0;
- nine blob comparisons and nine archive-integrity checks: exit 0;
- comparison with the preserved untracked report: exit 0;
- `git diff --check` and `git diff --cached --check`: exit 0.

No project build, pytest/CTest suite, research calculation or L1/L2/L3
gate is run for this source/history integration. Task 1.03's build/checker
verification and the phase checkpoints remain open. The 22 research Python
files and OD-1 are unchanged; no extra Python file is introduced here.

## 2026-10-21 — Task 1.10 measured numerics-record regeneration

Source revision: `cb057ef` (the actual Task-1.03 merge).
Generator `tools/native_dependencies.py` SHA-256:
`0204c0543ba6213224b7d84766e7d50e593c1cb665a3a5dcf97f22e69a6ff226`.
The user's follow-up request to perform items 1 and 2 authorized the
numerics probes needed for regeneration. No simulation, project test
suite, CMake configure or full gate was run.

### Measurement identities and outcomes

| Fact | macOS arm64 | Linux aarch64 |
|---|---|---|
| Interpreter | CPython 3.13.12, existing checkout `.venv` | CPython 3.13.16, image-owned `/opt/bike-venv` |
| NumPy / MuJoCo | 2.5.2 / 3.12.0 | 2.5.2 / 3.12.0 |
| Provider | Accelerate; macOS build 26A434, SDK build 26A425 | NumPy scipy-openblas ILP64; glibc 2.39 |
| ABI-control compiler | Apple clang 21.0.0 (clang-2100.3.34.2) | Ubuntu GCC 13.3.0 |
| ABI compile and link/run | pass / pass | pass / pass |
| NumPy interpolation | contracted, 1511 distinguishing queries / 4096 | separate, 1470 / 4096 |
| Broadcast NaN winner | scalar, 64 distinguishing lanes | element, 64 distinguishing lanes |
| SciPy interposition | checked, stable (SciPy 1.17.0) | checked, stable (SciPy 1.17.0) |

Both ABI controls reported:

```text
BIKE-ABI-OK ddot=32 dgemv=[10 26 42] dgemm=[4 5;10 11] ddot_ilp64=32 zdotu_sub=[-18 92] dgesv=[1 2] dsyevd=[1 3] dgelsd_rank=1 x=[0.6 1.2]
```

Docker Desktop was initially stopped. `open -a Docker` exited 1 (application
not found); opening the outer and inner application paths also exited 1
with `kLSNoExecutableErr`. `docker desktop start` subsequently exited 0
and the daemon became available. No installation/settings repair was made.
An existing image was reused for this measurement only:
`bike-linux-image:port`, image ID
`sha256:959091e9b453461be335eb31edadc63e36269b827755994f6b54d3796703ca69`,
architecture `arm64`, OS `linux`. A new container
`bike-dev-python-free-cpp-numerics` has no bind mounts; only the current
generator and generated macOS record were copied into it. Existing
containers and worktrees were retained. This is not qualification of the
new `tools/linux/Dockerfile` recipe.

### Commands and record checks

The first macOS attempt with `--python .venv/bin/python3` exited 1:
`selected interpreter must be an absolute path`. The successful commands
use the absolute interpreter. An initial container invocation with bare
`uv` exited 127 because image PATH lacks its directory; subsequent commands
use `/root/.local/bin/uv`.

All commands in this block exited 0:

```sh
uv run --frozen --no-sync --group native python tools/native_dependencies.py --python "$PWD/.venv/bin/python3" --output /private/tmp/bike-numerics-darwin-measured.json
uv run --frozen --no-sync --group native python tools/native_dependencies.py --python "$PWD/.venv/bin/python3" --write-numerics-record /private/tmp/bike-numerics-regenerated.lock.json
docker exec bike-dev-python-free-cpp-numerics /root/.local/bin/uv run --no-project --no-sync --python /opt/bike-venv/bin/python3 python tools/native_dependencies.py --python /opt/bike-venv/bin/python3 --output /src/linux-numerics-probe.json
docker exec bike-dev-python-free-cpp-numerics /root/.local/bin/uv run --no-project --python /opt/bike-venv/bin/python3 python tools/native_dependencies.py --python /opt/bike-venv/bin/python3 --write-numerics-record /src/native/numerics.lock.json
uv run --frozen --no-sync --group native python tools/native_dependencies.py --check-numerics-record native/numerics.lock.json --probe-document /private/tmp/bike-numerics-darwin-measured.json
docker exec bike-dev-python-free-cpp-numerics /root/.local/bin/uv run --no-project --python /opt/bike-venv/bin/python3 python tools/native_dependencies.py --check-numerics-record native/numerics.lock.json --probe-document /src/linux-numerics-probe.json
```

The existing four-operation lock was rejected against the real macOS probe
(exit 1, expected): `numerics.symbols must map exactly ('ddot', 'dgemv',
'dgemm', 'dgelsd', 'dgesv', 'dsyevd', 'ddot_ilp64', 'zdotu_sub')`.
The regenerated combined lock passes the same check, printing
`darwin-arm64` on macOS and `linux-aarch64` in the container.

The tool generated a fresh macOS file at the temporary path, then the
Linux writer preserved that entry and appended its measured record. The
combined generated file was copied back without hand-editing. JSON
comparison proves every pre-existing Darwin fact is unchanged after
removing only the four newly added symbol fields. The Linux record has
eight symbols, BLAS/LAPACK widths 64/64 and exactly the planned provider
SHA-256 `5dafca6786e1e366f29ff5e57a6b3beb66a4a02cadf899f51b38fa9fb22c5798`.

Generated `native/numerics.lock.json` SHA-256:
`19f542e9a581ef5b47379ab1420b6104f5bac72cf270860421c5ae5acd0539bc`.
Full machine-specific probe documents remain in ignored local evidence:

- `.work/evidence/numerics-2026-10-21/darwin-arm64.json`, SHA-256
  `b2124b4e70d181ed3499219d9e34f4e212e8d337be9fc5cd49c7f942f57ee517`;
- `.work/evidence/numerics-2026-10-21/linux-aarch64.json`, SHA-256
  `7b7913529dd7409012f9a1cd1291094e02b900a5b6d453c0eedf0943454ea358`.

Task 1.10's record-generation requirement is fulfilled. Its portability
pytest/build/configure checks, Task 1.03's L1 checks and all phase
checkpoints remain open; no task checkbox or qualification status is
advanced by these successful numerics controls alone.

## 2026-10-21 — Task 1.18 measured nonfinite sin/cos parity

Source revision: `36b0b0af5f386fa509179c6520d204f5fde89aad`, branch
`impl/python-free-cpp`. This is the explicitly authorized focused
measurement/regression slice for review item 3. No full project build,
native/full profile, simulation episode or phase checkpoint was run.

### Identities and measured decision

| Identity | macOS arm64 | Linux aarch64 |
|---|---|---|
| Host | macOS 27.0.1, build 26A434, Darwin 27.0.0 | Linux 7.0.14-linuxkit, aarch64 |
| Interpreter | CPython 3.13.12, checkout `.venv/bin/python3` | CPython 3.13.16, `/opt/bike-venv/bin/python3` |
| NumPy | 2.5.2, existing checkout venv | 2.5.2, image-owned venv |
| Native compiler | Apple Clang 21.0.0 (clang-2100.3.34.2) | Clang 18.1.3 (1ubuntu1); GCC 13.3.0-6ubuntu2~24.04.1 |
| Trig provider | libSystem.B.dylib 1359.0.0 | `/lib/aarch64-linux-gnu/libm.so.6`, Ubuntu glibc 2.39-0ubuntu8.9 |
| Focused linter | Homebrew clang-tidy/LLVM 23.1.2 | Ubuntu clang-tidy/LLVM 18.1.3 |

Reused running container `bike-dev-python-free-cpp-numerics`, image
`sha256:959091e9b453461be335eb31edadc63e36269b827755994f6b54d3796703ca69`.
Its `bike.checkout` label matches this worktree and `Mounts=[]`.
Only the probe, wrapper header, focused test and clang-tidy policy were
copied into `/tmp`; no host venv, bind mount, sync or image rebuild.

**Reproduced decision:** scalar and array NumPy results agree for all four
inputs and both operations on each platform (512 array output lanes per
platform). Linux libm already matches every oracle bit, so its existing
plain-libm rule stays unchanged. On macOS, libm sin preserves the negative
qNaN while libm cos canonicalizes it positive; the existing Apple
subtraction rule matches NumPy. No wrapper arithmetic, platform gate,
finite operation order or volatile anti-fusion load was changed.
No inference about other NaN payloads, signaling NaNs or other platforms
is made by this four-input measurement.

### Probe source and commands

The exact `/private/tmp/bike-sincos-probe.py` source (SHA-256
`1738b72ed70d45e53222d68c7d2a07cca5809973750b72b94432ebe9df83a8e1`):

```python
import platform
import struct
import sys

import numpy as np


def bits(value):
    return f"{struct.unpack('<Q', struct.pack('<d', value))[0]:016x}"


print(f"platform={platform.platform()} machine={platform.machine()}")
print(f"python={sys.version} executable={sys.executable}")
print(f"numpy={np.__version__} path={np.__file__}")
print("input scalar_sin scalar_cos array_sin array_cos lanes_equal")
for word in (0x7FF0000000000000, 0xFFF0000000000000,
             0x7FF8000000000000, 0xFFF8000000000000):
    value = struct.unpack('<d', struct.pack('<Q', word))[0]
    with np.errstate(invalid="ignore"):
        scalar = (np.sin(np.float64(value)), np.cos(np.float64(value)))
        array = (np.sin(np.full(64, value, dtype=np.float64)),
                 np.cos(np.full(64, value, dtype=np.float64)))
    equal = all(np.all(result.view(np.uint64) == result.view(np.uint64)[0])
                for result in array)
    print(f"{word:016x}", *(bits(item) for item in scalar),
          *(bits(result[0]) for result in array), equal)
```

These commands exited 0:

```sh
uv run --frozen --no-sync --group native python /private/tmp/bike-sincos-probe.py
docker inspect bike-dev-python-free-cpp-numerics --format '{{.Image}} {{.State.Status}} {{json .Mounts}} {{json .Config.Labels}}'
docker cp /private/tmp/bike-sincos-probe.py bike-dev-python-free-cpp-numerics:/tmp/bike-sincos-probe.py
docker exec bike-dev-python-free-cpp-numerics /root/.local/bin/uv run --no-project --python /opt/bike-venv/bin/python3 python /tmp/bike-sincos-probe.py
```

NumPy scalar/contiguous-array output on **both** measured platforms (each
`lanes_equal` covers all 64 lanes for both operations):

```text
input scalar_sin scalar_cos array_sin array_cos lanes_equal
7ff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 True
fff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 True
7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 True
fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 True
```

macOS Apple Clang native dump (before and after the test/comment change):

```text
input native_sin native_cos libm_sin libm_cos subtract
7ff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
fff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 7ff8000000000000 fff8000000000000
```

Linux aarch64 Clang 18 native dump (before and after), and GCC 13 final
dump (identical):

```text
input native_sin native_cos libm_sin libm_cos subtract
7ff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
fff0000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000 7ff8000000000000
fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000 fff8000000000000
```

### Focused regression and negative control

The final standalone TU is compiled with C++23, `-O2`,
`-ffp-contract=off -Wall -Wextra -Wpedantic -Werror`, and the shared
required safety flags
`-fstack-protector-strong -ftrivial-auto-var-init=zero
-fvisibility=hidden -fvisibility-inlines-hidden -fstrict-flex-arrays=3`.
Apple Clang, Linux Clang 18 and Linux GCC 13 each compile and run with
exit 0 and no test output: **24 exact-bit assertions per binary** (eight
measured nonfinite results, four signed-zero controls, twelve nonzero
finite np/math-versus-separate-libm controls). Final strict commands and
all exit details are in
`.superpowers/sdd/2026-10-21-sincos-golden-pull/task-3-report.md`.

The temporary negative-control TU changes only the +infinity row's
expected sin from `7ff8000000000000` to `fff8000000000000` in both
platform tables. Apple Clang and Linux Clang 18 compile it with the same
strict flags (exit 0); both runs exit **1**, reporting:

```text
np_sin nonfinite input=0x7ff0000000000000 expected=0xfff8000000000000 actual=0x7ff8000000000000
```

The production tree always retains the correct table. The three correct
binaries were run again after the isolated negative control and exit 0.
Because the original Linux rule already matches NumPy, this is an
expectation negative control, not evidence of a production behavior fix.

Focused clang-tidy, one selected TU plus its first-party header, uses
`native/.clang-tidy` and the same strict flags. Both final runs exit 0
with zero first-party diagnostics. macOS prints 1496 generated warnings
and suppresses 1500 (1496 non-user, four existing NOLINT); Linux prints
and suppresses 56118 non-user warnings. Earlier local helper diagnostics
(one swappable-parameter finding, then ten designated-initializer
findings on macOS) were corrected and rechecked; no suppression was added.

Two transient sandbox access failures occurred: uv could not initialize
its user cache (exit 2), and a Docker identity query could not access the
socket (exit 1). Each was retried with approved sandbox escalation. An
initial sequential negative-control preparation batch continued after
uv's failure, producing missing-source compile (1), missing binary (127)
and missing Docker-copy source (1); its Linux compile/run used the prior
analytic TU and exited 0. Those results are **not** RED evidence. The
subsequent successful preparation and final isolated negative controls
above establish RED; no failure is hidden.

### Verification limits

The existing cached native Makefile/compile database has no sincos target
entry, so this task uses the expressly permitted standalone compilation.
`native/CMakeLists.txt` already registers the test in the binding graph;
it is unchanged. Root-graph registration remains upstream Task 3.02.
No CMake configure, full graph rebuild, CTest/pytest suite, orchestrated
`check_tidy` sweep, sanitizer, RTSan, coverage or native/full gate was
run. Direct one-TU clang-tidy is not a claim that the full sweep passed.
Review item 5 owns full-project/phase verification. No phase status or
plan checkbox advances here. `native/numerics.lock.json` is unchanged,
SHA-256 `19f542e9a581ef5b47379ab1420b6104f5bac72cf270860421c5ae5acd0539bc`.
