"""
sample_model.py
Stage 1A: Standardized internal data structure (SampleData) and RawFileData container
for historical LabVIEW files (.csv, .txt, .xlsx) and Python indenter exports.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import numpy as np


@dataclass
class RawFileData:
    """Holds the raw, unmodified table read directly from an uploaded file."""
    file_path: str
    file_name: str
    file_type: str
    has_header_row: bool
    column_names: List[str]
    rows: List[List[str]]
    total_rows: int
    total_cols: int


@dataclass
class SampleData:
    """
    Standardized internal representation of an indentation experiment.
    Decouples downstream analysis and plotting from raw file formats.
    """
    sample_id: str
    file_name: str
    file_type: str

    # Standardized measurement vectors (1D NumPy float arrays)
    time: Optional[np.ndarray] = None
    position: Optional[np.ndarray] = None
    force: Optional[np.ndarray] = None
    voltage: Optional[np.ndarray] = None

    # Physical units for each vector
    units: Dict[str, str] = field(default_factory=lambda: {
        "time": "s",
        "position": "mm",
        "force": "g",
        "voltage": "V",
    })

    # Provenance & experimental metadata
    metadata: Dict[str, Any] = field(default_factory=dict)
    transformations_applied: List[str] = field(default_factory=list)
    raw_file_data: Optional[RawFileData] = field(default=None, repr=False)

    def __post_init__(self) -> None:
        if self.time is not None:
            self.time = np.asarray(self.time, dtype=float)
        if self.position is not None:
            self.position = np.asarray(self.position, dtype=float)
        if self.force is not None:
            self.force = np.asarray(self.force, dtype=float)
        if self.voltage is not None:
            self.voltage = np.asarray(self.voltage, dtype=float)

    @property
    def num_samples(self) -> int:
        for arr in (self.force, self.position, self.time, self.voltage):
            if arr is not None:
                return int(len(arr))
        return 0