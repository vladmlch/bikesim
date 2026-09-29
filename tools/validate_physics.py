#!/usr/bin/env python3
"""Run independent physics gates and write strict JSON; any mandatory failure exits 1."""
import argparse
from bike_sim.validation.benchmarks import REGISTRY,run_suite


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',default='output/physics-validation')
    parser.add_argument('--dt',nargs='+',type=float,default=[.0005,.00025,.000125])
    parser.add_argument('--cases',nargs='+',choices=tuple(REGISTRY))
    parser.add_argument('--jobs',type=int,default=1,help='independent worker processes, 1 to 32')
    parser.add_argument('--require-lock',action='store_true',help='also fail if the measured environment differs from uv.lock')
    args=parser.parse_args(argv)
    report=run_suite(args.out,args.dt,cases=args.cases,jobs=args.jobs)
    okay=report['passed'] and report['source_unchanged_during_run']
    if args.require_lock:
        okay=okay and report['environment']['locked_environment_verified']
    return 0 if okay else 1


if __name__=='__main__':raise SystemExit(main())
