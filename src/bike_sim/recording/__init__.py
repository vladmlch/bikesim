"""Opt-in whole-launch run recording (design 2026-10-12-run-recording).

Stage A provides the contracts, the in-RAM launch owner, the reviewed
field/coordinate catalog and typed-batch encoding. Storage writers and the
analysis CLI land in later stages; nothing here performs disk I/O.
"""
from bike_sim.recording.contracts import (
    Asset, EpisodeEvidence, InputEvent, PhysicalSource, RunMetadata,
    TableBatch,
)

__all__ = [
    'Asset', 'EpisodeEvidence', 'InputEvent', 'PhysicalSource', 'RunMetadata',
    'TableBatch',
]
