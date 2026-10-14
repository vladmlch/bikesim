"""Owned recording contracts; DuckDB, Arrow and engines are imported lazily.

Use RunCapture for final-only publication and recording.reader.Recording for
manifest-validated, data-only analysis. No work is performed on import.
"""
from bike_sim.recording.contracts import (
    Asset, EpisodeEvidence, InputEvent, PhysicalSource, ResearchEvidence,
    ResearchSource, RunMetadata, TableBatch,
)

__all__ = ['Asset', 'EpisodeEvidence', 'InputEvent', 'PhysicalSource',
           'ResearchEvidence', 'ResearchSource', 'RunMetadata', 'TableBatch']
