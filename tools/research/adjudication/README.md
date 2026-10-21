# Adjudication recalculation scripts (WP-V4)

Standalone, stdlib-only recomputations backing
`docs/superpowers/audits/2026-10-14-round-2-adjudication.md`. Each script prints
`claimed / recomputed / diff / OK-or-MISMATCH` per checked value; 1 % relative is the
default pass band (5 % where the audit itself round-trips coarse digits).

Run:

```bash
uv run python tools/research/adjudication/gemini_golden_cases.py
uv run python tools/research/adjudication/audit_golden_cases.py
```

- `gemini_golden_cases.py` — Gemini tyre report golden cases TC-M1 (Bekker plate,
  compaction work), TC-M2 (hard-bottom exponential hardening), TC-M3 (Janosi shear vs
  basal slab cap on a 30° slope).
- `audit_golden_cases.py` — audit angle-M golden cases: G1–G6 Bekker plate inversions,
  G11 compaction work, G7/G8/G9 Wong–Reece wheel (RoverDevKit and PINNeAPPle
  conventions), G12 Terzaghi N_c convention check.

Neither script touches `src/`, `native/`, tests, or specs; they exist to make the
verdicts in the adjudication registry reproducible.