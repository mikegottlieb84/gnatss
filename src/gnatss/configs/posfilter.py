"""posfilter.py

The posfilter module containing base models for
position filtering configuration
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field, model_validator

from .io import InputData


class AtdOffset(BaseModel):
    """Antenna Transducer Offset base model."""

    forward: float = Field(..., description="forward offset")
    rightward: float = Field(..., description="rightward offset")
    downward: float = Field(..., description="downward offset")


class PositionFilterInputs(BaseModel):
    ahrs: InputData | None = Field(
        None, description="Externally extracted XSENS attitude CSV files."
    )
    roll_pitch_heading: InputData | None = Field(
        None, description="Roll Pitch Heading (RPH) data path specification."
    )
    novatel: InputData | None = Field(None, description="Novatel data path specification.")
    novatel_std: InputData | None = Field(None, description="Novatel STD data path specification.")
    gps_positions: InputData | None = Field(
        None, description="GPS positions data path specification."
    )
    travel_times: InputData | None = Field(
        None, description="Input travel times data path specification"
    )


class ExportObs(BaseModel):
    """Export observation data after filtering."""

    full: bool = Field(
        False,
        description="Only export the necessary data, including antenna covariance.",
    )


class AhrsOptions(BaseModel):
    """Explicit uncertainty assumptions and interpolation limits for AHRS attitude."""

    roll_std: float = Field(
        ..., gt=0, allow_inf_nan=False, description="Roll uncertainty, degrees."
    )
    pitch_std: float = Field(
        ..., gt=0, allow_inf_nan=False, description="Pitch uncertainty, degrees."
    )
    heading_std: float = Field(
        ..., gt=0, allow_inf_nan=False, description="Heading uncertainty, degrees."
    )
    max_gap_seconds: float = Field(0.5, gt=0, allow_inf_nan=False)
    heading_offset_deg: float = Field(
        0.0, allow_inf_nan=False, description="Correction added to exported heading, degrees."
    )


class PositionFilter(BaseModel):
    """
    Position filter base model for position filtering routine
    """

    export: ExportObs | None = Field(None, description="Export observation data after filtering.")
    input_files: PositionFilterInputs | None = Field(
        None, description="Input files for position filtering routine."
    )
    atd_offsets: AtdOffset = Field(..., description="Antenna Transducer Offset values.")
    attitude_source: Literal["NOV_INS", "AHRS"] = "NOV_INS"
    ahrs: AhrsOptions | None = None

    @model_validator(mode="after")
    def validate_ahrs_input(self):
        if self.attitude_source == "AHRS" and (
            self.input_files is None or self.input_files.ahrs is None or self.ahrs is None
        ):
            msg = (
                "AHRS requires posfilter.input_files.ahrs and posfilter.ahrs uncertainty settings."
            )
            raise ValueError(msg)
        return self
