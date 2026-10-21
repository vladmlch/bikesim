# Task 1.03: drop-oracle-parity source merge into orig

Date: 2026-10-21
Scope: D13 / Task 1.03 of [the C++ + Linux completion plan](../docs/superpowers/plans/2026-10-20-cpp-linux-completion.md), reconciled with D12 / Task 1.04 and [ADR-0003](../docs/adr/0003-native-authoritative-python-removal.md).

## Result and scope

The supplied `drop-oracle-parity.xml.zip` research and ADR payload is integrated into the supplied `orig.xml.zip` source tree. The destination already contains Phase-1 and Phase-2 source work. That implementation is retained. This delivery performs the planned source integration; it does not execute later phases or establish a Git merge commit.

The inputs are Repomix text snapshots, with no Git objects or usable branch ancestry. Their whole-tree differences are not the branch delta: the donor is older in native code, Linux support, Python runtime code, tests and build tooling. Task 1.03 explicitly requires no incoming native source changes, so those differences are not applied. Every original path remains present, and every original file outside the five reconciled documentation/evidence files retains its supplied bytes.

### Input identities

| Input | ZIP SHA-256 | Extracted text files |
|---|---|---:|
| `orig.xml.zip` | `d76f46a7745130aa8d3ad8c9c56b94acbaeecdf86b4657d4a531a05833f22037` | 2,004 |
| `drop-oracle-parity.xml.zip` | `d30493e3fbe353a16c6256a65346b12073e0470a317283934dd3dfefe0ea9092` | 1,010 |

The original ZIP attachments are the provenance of the inputs. The delivered `orig.xml.zip` is the updated Repomix snapshot and has a different hash.

## Imported material

- All 14 donor-only documents under `docs/`: ADR-0002, two reference summaries, four source research reports, five r2/track/compiler audits, and two historical research plans.
- All 130 supplied files under `tools/research/`: 22 Python research scripts, documentation, two DEM tile lists, and lateral-digitisation CSV/JSON measurements and summaries.
- One recovered expected statistics JSON, copied without recalculation from the donor's preserved audit result.
- This report and a machine-readable [merge manifest](cpp-linux-task103-merge-manifest.json).

The 22 imported Python files are unchanged from the extracted donor. They are registered under the exact Task 1.03 heading in the append-only [completion evidence](cpp-linux-completion-evidence.md). OD-1 remains for Task 3.02; no research-script deletion or relocation decision is made here.

The donor's temporary research working tree and unrelated October-3 progress record are excluded. Published research outputs and their original scientific text remain intact. Historical plans retain their supplied content, including earlier verification-policy statements; the completion plan and ADR-0003 are the current authority.

## Documentation reconciliation

| File | Resolution |
|---|---|
| `docs/adr/0001-native-port-mujoco-core.md` | Bring in the ADR-0002 status reference and qualify it with ADR-0003's transition. |
| `docs/adr/0002-drop-python-oracle-parity.md` | Import the actual donor text. Amend only its status line: permanent Python retention and immediate removal of live-oracle gates are superseded by ADR-0003. The historical body is preserved. |
| `docs/adr/0003-native-authoritative-python-removal.md` | Preserve its decision; replace the obsolete missing-payload paragraph with the current source-merge status. |
| `docs/TESTING.md` | Add a current ADR-0003 transition note. Retain the existing Linux, root-graph, checker, sanitizer, coverage and baseline documentation. |
| `CONTEXT.md` | Align only the order-changing optimization definition with the current per-file transition. Keep C++23, native model construction and the current golden-episode definition. |
| `migration/cpp-linux-completion-evidence.md` | Append source-merge evidence and the exact 22-script list. Earlier missing-input entries remain as dated history. |

`AGENTS.md`, `docs/agents/native-engineering.md`, all native implementation/build files, the root CMake graph, original Python code/tests/tools, dependency locks, oracle manifests and existing historical plans are byte-identical to `orig`. No baseline, qualification status, acceptance record or task checkbox is promoted.

## Recovering the expected statistics file

The imported package README explicitly identifies its expected reference as byte-identical to the audit result. The donor contains that audit result, so the merge copies it to the path the existing comparator consumes:

- Source: `.scratch/track-research/a-slope-profile/src/stats2_out.json` in the donor snapshot.
- Destination: `tools/research/alpine_osm_dem/expected/stats2_out.json`.
- Size: 15,391 bytes as extracted.
- SHA-256: `414e7f4b37309ce839d6cc3926dcb43570150afb9ff937b9a7f4ae902855bc30`.

No calculation, tolerance adjustment or statistics comparison was run. The package's `.gitignore` now uses `/stats2_out.json`, anchoring its generated-output exclusion to the package root so that the expected reference remains trackable.

## Portable research filename

One report's original basename is 303 UTF-8 bytes, beyond the Linux filesystem component limit encountered during extraction. Its content is unchanged and its destination is:

`docs/researches/gemini-alpine-trail-geomorphology.md`

The exact original-to-destination mapping and content hashes are retained in the merge manifest. No exact link requiring the long original filename was found among the imported documents.

## Inputs absent from the supplied snapshots

The following nine compressed research datasets named by the alpine package README are absent from both snapshots:

- `tools/research/alpine_osm_dem/data/zermatt_paths.json.gz`
- `tools/research/alpine_osm_dem/data/engadin_paths.json.gz`
- `tools/research/alpine_osm_dem/data/zermatt_ways_lv.json.gz`
- `tools/research/alpine_osm_dem/data/engadin_ways_lv.json.gz`
- `tools/research/alpine_osm_dem/data/zermatt_profiles.json.gz`
- `tools/research/alpine_osm_dem/data/engadin_profiles.json.gz`
- `tools/research/alpine_osm_dem/data/scree_crossings_at.slim.json.gz`
- `tools/research/alpine_osm_dem/data/scree_crossings_ch.slim.json.gz`
- `tools/research/alpine_osm_dem/data/scree_crossing_elev_at.json.gz`

The two tile lists and all supplied digitisation point/summary files are retained. The lateral-digitisation source PDFs and figure images are also absent. The package documentation records these limits next to the original workflow. The original offline-reproduction statements and dated verification tables describe the research author's earlier checkout; this delivery does not establish that those results can be regenerated from these text snapshots alone. No replacement dataset, binary fixture or new research output is fabricated or downloaded.

Restore the original pinned inputs from the actual research branch or its archive to run that research pipeline. These absent research inputs do not require changing the simulator or its build graph for this source merge.

## Static verification and remaining gates

Verification is limited to file operations and static inspection: archive extraction, exact source/destination comparison, hash checks, protected-path preservation, new-Python inventory, conflict-marker inspection in reconciled text, and patch application to temporary source copies. No project program, dependency acquisition, configure, compilation, CTest, pytest, analyzer, ShellCheck, container, viewer or simulation is run.

The patch is checked against both the exact Repomix-unmixed baseline and a normal LF-terminated baseline. Manifest hashes describe the exact extracted/source-archive bytes; the ordinary checkout variant preserves every new file's bytes and adapts EOF only in existing files. The delivered result is compared with the applied patch by file content. Git is used only to form and apply filesystem patches; this does not recover branch history. Input Repomix files do not encode the original Git executable modes, so the patch contains no changes to existing modes.

Task 1.03's missing text-payload prerequisite is resolved, and the ADR-0002 reconciliation required by Task 1.04 is implemented in source. Task 1.03's actual branch merge/ancestry and its L1 commands remain unverified. The existing Phase-1/Phase-2 execution gates and numerics-lock regeneration requirements remain open. The old Phase-2 source review and earlier evidence entries describe their original delivery date; this appended merge record supersedes only their missing-payload status.

## Applying this delivery

Use the ordinary checkout patch for a repository whose text files end in LF:

```sh
git apply --check merge-drop-oracle-parity.patch
git apply merge-drop-oracle-parity.patch
```

Use the Repomix patch when starting from files extracted literally with the Repomix-unmixer skill (that extractor omits the wrapper's final newline):

```sh
git apply --check merge-drop-oracle-parity-repomix.patch
git apply merge-drop-oracle-parity-repomix.patch
```

Apply one patch variant only. Both carry the same substantive changes. The source archive and the updated Repomix snapshot preserve the extraction convention, including unchanged original bytes. They contain the available text tree; Git history, dependencies and binary assets omitted by the input snapshots are not recreated.
