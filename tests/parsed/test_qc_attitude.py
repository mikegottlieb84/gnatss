from __future__ import annotations

import copy
import io
import json
import tarfile
from types import SimpleNamespace

import numpy as np
import pytest
from pydantic import ValidationError

from gnatss import constants
from gnatss.configs.parsed import Parsed
from gnatss.loaders import load_sv3_targz


def config(source="AHRS"):
    return SimpleNamespace(
        site_id="TEST",
        parsed=Parsed(
            attitude_source=source,
            atd_offsets={"forward": 0, "rightward": 0, "downward": 1},
        ),
    )


def ping():
    observations = {
        "GNSS": {"time": {"common": 1788145642.0}},
        "NOV_INS": {
            "latitude": 41.65,
            "longitude": -124.93,
            "hae": -30.0,
            "vx": 1.0,
            "vy": 2.0,
            "vz": 3.0,
            "r": 10.0,
            "p": 20.0,
            "h": 30.0,
        },
        "AHRS": {"r": 1.0, "p": 2.0, "h": 3.0},
    }
    result = {"interrogation": {"event": "interrogation", "observations": observations}}
    for i, address in enumerate(["IR5209", "IR5210", "IR5211"], 1):
        obs = copy.deepcopy(observations)
        obs["AHRS"] = {"r": float(i + 1), "p": float(i + 2), "h": float(i + 3)}
        result[address] = {
            "event": "range",
            "observations": obs,
            "time": {"common": 1788145642.0 + i},
            "range": {"range": float(i), "cn": address},
        }
    return result


def archive(tmp_path, records):
    path = tmp_path / "qc.tar.gz"
    payload = "\n".join(json.dumps(record) for record in records).encode()
    with tarfile.open(path, "w:gz") as tar:
        member = tarfile.TarInfo("qc.pin")
        member.size = len(payload)
        tar.addfile(member, io.BytesIO(payload))
    return [str(path)]


def test_default_and_invalid_source():
    assert Parsed(atd_offsets=config().parsed.atd_offsets).attitude_source == "NOV_INS"
    with pytest.raises(ValidationError):
        config("unknown")


def test_attitude_selection_preserves_positions_and_timing(tmp_path):
    paths = archive(tmp_path, [ping()])
    ahrs = load_sv3_targz(config(), paths)
    ins = load_sv3_targz(config("NOV_INS"), paths)
    columns = constants.RPH_LOCAL_TANGENTS
    np.testing.assert_allclose(ahrs[columns], [[1, 2, 3], [2, 3, 4], [3, 4, 5], [4, 5, 6]])
    np.testing.assert_allclose(ins[columns], [[10, 20, 30]] * 4)
    assert ahrs.drop(columns=columns).equals(ins.drop(columns=columns))


@pytest.mark.parametrize("event", ["interrogation", "IR5211"])
@pytest.mark.parametrize(
    "attitude",
    [
        None,
        "ERR",
        {"h_mag": 90},
        {"r": 1, "p": None, "h": 3},
        {"r": 1, "p": 2, "h": float("nan")},
        {"r": float("inf"), "p": 2, "h": 3},
        {"r": True, "p": 2, "h": 3},
    ],
)
def test_incomplete_attitude_skips_whole_ping(tmp_path, capsys, event, attitude):
    bad = ping()
    bad[event]["observations"]["AHRS"] = attitude
    result = load_sv3_targz(config(), archive(tmp_path, [bad, ping()]))
    assert len(result) == 4
    assert "Skipped 1 QC pings" in capsys.readouterr().out


def test_missing_source_and_all_pings_skipped(tmp_path):
    bad = ping()
    del bad["interrogation"]["observations"]["AHRS"]
    paths = archive(tmp_path, [bad])
    with pytest.raises(ValueError, match="No QC pings remain with complete AHRS"):
        load_sv3_targz(config(), paths)
    assert len(load_sv3_targz(config("NOV_INS"), paths)) == 4


def test_zero_attitude_valid_and_zero_range_ignored(tmp_path):
    record = ping()
    record["interrogation"]["observations"]["AHRS"] = {"r": 0, "p": 0, "h": 0}
    record["IR5211"]["range"]["range"] = 0
    del record["IR5211"]["observations"]["AHRS"]
    result = load_sv3_targz(config(), archive(tmp_path, [record]))
    assert len(result) == 3
    np.testing.assert_allclose(result[constants.RPH_LOCAL_TANGENTS].iloc[0], [0, 0, 0])
