from pathlib import Path


SCRIPT = Path("/opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace/run_autoracer_stage_b_headless.sh")


def test_stage_b_headless_defaults_to_first_500m_smoke_window():
    source = SCRIPT.read_text(encoding="utf-8")

    assert 'MIN_DISTANCE_M="${MIN_DISTANCE_M:-500}"' in source
    assert 'TSTOP="${TSTOP:-330}"' in source
