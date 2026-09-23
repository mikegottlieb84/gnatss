from __future__ import annotations

import numpy as np
import pandas as pd
import typer
from scipy.interpolate import CubicSpline

from .. import constants
from ..configs.posfilter import AhrsOptions


def interpolate_ahrs(
    ahrs: pd.DataFrame, travel_times: pd.DataFrame, options: AhrsOptions
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Interpolate continuous AHRS segments, then discard whole unsupported pings.

    Cubic splines use elapsed seconds, with angular unwrapping before interpolation.
    Segments never cross invalid samples or excessive gaps, and never extrapolate.
    Two/three-sample segments use linear interpolation; isolated samples support
    only an exactly matching event timestamp.
    """
    data = ahrs.sort_values(constants.RPH_TIME)
    times = data[constants.RPH_TIME].to_numpy()
    angles = data[constants.RPH_LOCAL_TANGENTS].to_numpy(dtype=float)
    targets = travel_times[constants.TT_TIME].to_numpy()
    values = np.full((len(targets), 3), np.nan)
    valid = np.isfinite(angles).all(axis=1) & np.isfinite(times)
    indices = np.flatnonzero(valid)
    if len(indices):
        breaks = (
            np.flatnonzero(
                (np.diff(indices) != 1) | (np.diff(times[indices]) > options.max_gap_seconds)
            )
            + 1
        )
        for segment in np.split(indices, breaks):
            t = times[segment]
            mask = (targets >= t[0]) & (targets <= t[-1])
            if not mask.any():
                continue
            # Roll can wrap as well as heading; unwrap each angular coordinate.
            y = np.rad2deg(np.unwrap(np.deg2rad(angles[segment]), axis=0))
            x = t - t[0]
            query = targets[mask] - t[0]
            if len(t) >= 4:
                values[mask] = CubicSpline(x, y, extrapolate=False)(query)
            else:
                values[mask] = np.column_stack([np.interp(query, x, y[:, i]) for i in range(3)])

    values[:, 0] = (values[:, 0] + 180) % 360 - 180
    values[:, 2] = (values[:, 2] + options.heading_offset_deg) % 360
    ping_ids = travel_times[constants.DATA_SPEC.tx_time].fillna(travel_times[constants.TT_TIME])
    bad_pings = ping_ids[~np.isfinite(values).all(axis=1)].unique()
    keep = ~ping_ids.isin(bad_pings).to_numpy()
    typer.echo(f"Skipped {len(bad_pings)} pings without AHRS coverage at every transmit/reply.")
    if not keep.any():
        msg = "No pings remain within valid AHRS coverage; check timestamps and max_gap_seconds."
        raise ValueError(msg)

    retained = travel_times.loc[keep].reset_index(drop=True)
    attitude = retained.copy()
    attitude[constants.RPH_LOCAL_TANGENTS] = values[keep]
    attitude[constants.PLATFORM_COV_RPH] = 0.0
    attitude[constants.PLATFORM_COV_RPH_DIAG] = np.square(
        [options.roll_std, options.pitch_std, options.heading_std]
    )
    return retained, attitude
