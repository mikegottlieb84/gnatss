from __future__ import annotations

import importlib
from types import SimpleNamespace

import numpy as np
import pandas as pd
import pytest
from pydantic import ValidationError

from gnatss import constants
from gnatss.configs.io import InputData
from gnatss.configs.posfilter import AhrsOptions, PositionFilter
from gnatss.loaders import load_ahrs_csv
from gnatss.ops.io import load_files_to_dataframe
from gnatss.posfilter.ahrs import interpolate_ahrs
from gnatss.posfilter.posfilter import rotation
from gnatss.utilities.time import AstroTime, gps_ws_time_to_astrotime


def options(**kwargs):
    return AhrsOptions(roll_std=0.2, pitch_std=0.3, heading_std=2.0, **kwargs)


def series(times, roll=None, pitch=None, heading=None):
    n = len(times)
    return pd.DataFrame(
        {
            constants.RPH_TIME: times,
            "roll": np.zeros(n) if roll is None else roll,
            "pitch": np.zeros(n) if pitch is None else pitch,
            "heading": np.zeros(n) if heading is None else heading,
        }
    )


def events(times, tx=None):
    return pd.DataFrame(
        {
            constants.TT_TIME: times,
            constants.DATA_SPEC.tx_time: [np.nan] * len(times) if tx is None else tx,
            constants.DATA_SPEC.transponder_id: ["T_transmit"] * len(times),
            constants.DATA_SPEC.travel_time: np.zeros(len(times)),
        }
    )


def test_csv_headers_time_conversion_sorting_and_overlapping_files(tmp_path):
    path = tmp_path / "ahrs.csv"
    path.write_text(
        "Common Time, Instrument Time, Heading, Pitch, Roll, Magnetic field X\n"
        "1757894400.1,67814.4,-109.8,3.0,-1.8,0.1\n"
        "1757894400.0,67814.3,-110.0,2.0,-1.0,0.1\n"
    )
    data = load_ahrs_csv([str(path), str(path)])
    assert len(data) == 2
    assert data[constants.RPH_TIME].is_monotonic_increasing
    np.testing.assert_allclose(
        data[constants.RPH_LOCAL_TANGENTS], [[-1, 2, -110], [-1.8, 3, -109.8]]
    )
    expected = AstroTime([1757894400.0, 1757894400.1], format="unix", scale="tt").rangea_j2000
    np.testing.assert_allclose(data[constants.RPH_TIME], expected, rtol=0, atol=1e-7)
    loaded = load_files_to_dataframe("ahrs", InputData(path=str(path)), None)
    pd.testing.assert_frame_equal(data, loaded)


def test_legacy_swapped_timestamp_columns_are_explicit(tmp_path):
    path = tmp_path / "ahrs.csv"
    path.write_text("Common Time,Instrument Time,Heading,Pitch,Roll\n67814,1757894400,1,2,3\n")
    with pytest.raises(ValueError, match="swapped"):
        load_ahrs_csv([str(path)])
    inp = InputData(path=str(path), loader_kwargs={"time_column": "Instrument Time"})
    assert len(load_files_to_dataframe("ahrs", inp, None)) == 1


def test_csv_invalid_angles_remain_gaps_and_conflicts_fail(tmp_path):
    path = tmp_path / "ahrs.csv"
    path.write_text("Common Time,Heading,Pitch,Roll\n1757894400,,inf,ERR\n")
    data = load_ahrs_csv([str(path)])
    assert data[constants.RPH_LOCAL_TANGENTS].isna().all().all()
    path.write_text("Common Time,Heading,Pitch,Roll\n1757894400,1,2,3\n1757894400,4,2,3\n")
    with pytest.raises(ValueError, match="Conflicting"):
        load_ahrs_csv([str(path)])
    path.write_text("time,roll\n1,2\n")
    with pytest.raises(ValueError, match="requires columns"):
        load_ahrs_csv([str(path)])


def test_cubic_interpolation_uses_actual_irregular_timestamps():
    t = np.array([0.0, 0.1, 0.3, 0.5, 0.7])
    data = series(t, roll=t**3, pitch=2 * t, heading=20 + 3 * t)
    target = np.array([0.05, 0.2, 0.4])
    retained, result = interpolate_ahrs(data, events(target), options())
    np.testing.assert_allclose(result["roll"], target**3, atol=1e-12)
    np.testing.assert_allclose(result["pitch"], 2 * target)
    np.testing.assert_allclose(result["heading"], 20 + 3 * target)
    np.testing.assert_allclose(result[constants.PLATFORM_COV_RPH_DIAG], [[0.04, 0.09, 4]] * 3)
    pd.testing.assert_frame_equal(retained, events(target))


def test_wraparound_and_heading_correction():
    data = series([0, 0.1, 0.2, 0.3], heading=[358, 359, 0, 1], roll=[178, 179, -180, -179])
    _, result = interpolate_ahrs(data, events([0.15]), options(heading_offset_deg=10))
    assert result["heading"].iloc[0] == pytest.approx(9.5)
    assert result["roll"].iloc[0] == pytest.approx(179.5)


def test_common_time_aligns_with_novatel_clock():
    # Paired common/GPS times from a PIN observation; only receipt latency remains.
    week, seconds = divmod(1472180860, 604800)
    ins_time = gps_ws_time_to_astrotime(np.array([week]), np.array([float(seconds)])).unix_j2000[0]
    common_time = AstroTime(1788145642.0075219, format="unix", scale="tt").rangea_j2000
    assert common_time - ins_time == pytest.approx(0.0075219, abs=1e-6)


@pytest.mark.parametrize(
    "rph,offset,expected",
    [
        ([0, 0, 90], [1, 0, 0], [6378137, 1, 0]),  # Heading east.
        ([0, 90, 0], [1, 0, 0], [6378138, 0, 0]),  # Bow up.
        ([90, 0, 0], [0, 1, 0], [6378136, 0, 0]),  # Starboard down.
    ],
)
def test_ahrs_ned_conventions_match_existing_rotation(rph, offset, expected):
    attitude = series([0], roll=[rph[0]], pitch=[rph[1]], heading=[rph[2]])
    pos = pd.DataFrame({constants.TIME_J2000: [0], "ant_x": [6378137], "ant_y": [0], "ant_z": [0]})
    result = rotation(pos, attitude, np.array(offset), SimpleNamespace(lat=0, lon=0, alt=0))
    np.testing.assert_allclose(result[constants.GPS_GEOCENTRIC].iloc[0], expected, atol=1e-8)


@pytest.mark.parametrize("bad_time", [-0.01, 0.5, 1.21])
def test_gaps_and_extrapolation_drop_entire_ping(bad_time, capsys):
    data = series([0, 0.1, 0.2, 1.0, 1.1, 1.2])
    twtt = events([0.05, bad_time, 1.05, 1.15], [np.nan, 0.05, np.nan, 1.05])
    kept, _ = interpolate_ahrs(data, twtt, options())
    assert kept[constants.TT_TIME].tolist() == [1.05, 1.15]
    assert "Skipped 1 pings" in capsys.readouterr().out


def test_invalid_sample_breaks_segment():
    data = series([0, 0.1, 0.2, 0.3, 0.4], heading=[0, 1, np.nan, 3, 4])
    kept, _ = interpolate_ahrs(data, events([0.05, 0.15, 0.25, 0.35]), options())
    np.testing.assert_allclose(kept[constants.TT_TIME], [0.05, 0.35])


def test_no_coverage_fails_and_isolated_exact_sample_works():
    data = series([1.0])
    with pytest.raises(ValueError, match="No pings remain"):
        interpolate_ahrs(data, events([1.01]), options())
    _, result = interpolate_ahrs(data, events([1.0]), options())
    np.testing.assert_array_equal(result[constants.RPH_LOCAL_TANGENTS], [[0, 0, 0]])


def test_config_defaults_requirements_and_uncertainty(tmp_path):
    offsets = dict(forward=0, rightward=0, downward=1)
    assert PositionFilter(atd_offsets=offsets).attitude_source == "NOV_INS"
    with pytest.raises(ValidationError, match="AHRS requires"):
        PositionFilter(atd_offsets=offsets, attitude_source="AHRS")
    with pytest.raises(ValidationError):
        options(max_gap_seconds=0)
    path = tmp_path / "a.csv"
    path.touch()
    cfg = PositionFilter(
        atd_offsets=offsets,
        attitude_source="AHRS",
        ahrs=options(),
        input_files={"ahrs": {"path": str(path)}},
    )
    assert cfg.ahrs.heading_std == 2


@pytest.mark.parametrize("source", ["NOV_INS", "AHRS"])
def test_runner_selects_attitude_but_keeps_novatel_velocity_inputs(monkeypatch, source):
    run = importlib.import_module("gnatss.posfilter.run")
    novatel, novatel_std, gps = object(), object(), object()
    captured = {}

    def kalman(**kwargs):
        assert kwargs["inspvaa_df"] is novatel
        assert kwargs["insstdeva_df"] is novatel_std
        assert kwargs["gps_df"] is gps
        return kwargs["twtt_df"]

    def spline(**kwargs):
        assert source == "NOV_INS"
        captured["spline"] = True
        return "novatel attitude"

    def rotation(pos, attitude, *_):
        captured["attitude"] = attitude
        return pos

    monkeypatch.setattr(run, "kalman_filtering", kalman)
    monkeypatch.setattr(run, "spline_interpolate", spline)
    monkeypatch.setattr(run, "rotation", rotation)
    monkeypatch.setattr(run, "standardize_data", lambda x: x)
    monkeypatch.setattr(run, "ensure_monotonic_increasing", lambda x: x)
    cfg = SimpleNamespace(
        array_center=None,
        posfilter=SimpleNamespace(
            attitude_source=source,
            ahrs=options(),
            atd_offsets=None,
            export=None,
            input_files=SimpleNamespace(travel_times=SimpleNamespace(format="DFOP")),
        ),
    )
    data = dict(
        novatel=novatel,
        novatel_std=novatel_std,
        gps_positions=gps,
        travel_times=events([0.15]),
        ahrs=series([0, 0.1, 0.2, 0.3], heading=[10] * 4),
    )
    run.run_posfilter(cfg, data)
    if source == "AHRS":
        assert "spline" not in captured
        assert captured["attitude"]["heading"].iloc[0] == pytest.approx(10)
    else:
        assert captured["attitude"] == "novatel attitude"
