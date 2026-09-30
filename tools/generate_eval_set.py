#!/usr/bin/env python3
"""Freeze a committed evaluation set of generated tracks.

A policy is compared across revisions on the same tracks, so the set is written
once as TOML (loadable with `bike-research --track-file`) plus a manifest of
seeds and the spec hash, instead of being regenerated on the fly.
"""
from datetime import date
from pathlib import Path
import argparse
import json
import sys
import tomllib
from bike_sim.terrain.generator import TerrainGenSpec, generate_track
from bike_sim.terrain.trackfile import save_track


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--n', type=int, default=10)
    p.add_argument('--seed0', type=int, default=1000)
    p.add_argument('--gen-spec', type=Path, help='TerrainGenSpec TOML (default: built-in ranges)')
    p.add_argument('--out', type=Path, default=Path('examples/research/eval'))
    p.add_argument('--date', default=None, help='manifest date (default: today)')
    args = p.parse_args(argv)
    if args.n < 1:
        p.error('--n must be positive')
    if args.out.exists() and any(args.out.iterdir()):
        print(f'ERROR: refusing to overwrite nonempty eval set: {args.out}', file=sys.stderr)
        return 2
    spec = TerrainGenSpec.from_dict(tomllib.loads(args.gen_spec.read_text())) if args.gen_spec else TerrainGenSpec()
    args.out.mkdir(parents=True, exist_ok=True)
    tracks = []
    for i in range(args.n):
        seed = args.seed0+i
        track = generate_track(spec, seed=seed, name=f'eval_{i:02d}')
        save_track(track, args.out/f'eval_{i:02d}.toml')
        tracks.append(dict(file=f'eval_{i:02d}.toml', seed=seed, length_m=track.length_m))
    manifest = dict(date=args.date or date.today().isoformat(), seed0=args.seed0, spec=spec.to_dict(),
                    spec_sha256=spec.sha256(), tracks=tracks)
    (args.out/'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
