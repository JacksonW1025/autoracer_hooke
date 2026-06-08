from pathlib import Path


TESTRUN = Path("/opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace/Data/TestRun/AutoracerStageB_UrbanRoute271")
SENSORLESS_CAR = Path(
    "/opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace/Data/Vehicle/AutoracerStageB_SensorlessCar"
)
NDT_TESTRUN = Path(
    "/opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace/Data/TestRun/AutoracerStageB_UrbanRoute271_NDT"
)
NDT_SENSOR_CAR = Path(
    "/opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace/Data/Vehicle/AutoracerStageB_NDTSensorCar"
)
NDT_RUN_SCRIPT = Path(
    "/opt/ipg/carmaker/linux64-15.1/SimProject_TianmenRace/run_stage_b_ndt_realtime.sh"
)


def test_stage_b_testrun_uses_sensorless_autoracer_vehicle_for_planning_closed_loop():
    testrun = TESTRUN.read_text(encoding="utf-8")
    sensorless_car = SENSORLESS_CAR.read_text(encoding="utf-8")

    assert "Vehicle = AutoracerStageB_SensorlessCar" in testrun
    assert "SensorCluster.N = 0" in sensorless_car
    assert "Sensor.2.Active = 0" in sensorless_car
    assert "Sensor.2.Ref.Cluster =" in sensorless_car


def test_stage_b_ndt_testrun_uses_sensor_enabled_copy_without_mutating_sensorless_baseline():
    ndt_testrun = NDT_TESTRUN.read_text(encoding="utf-8")
    ndt_sensor_car = NDT_SENSOR_CAR.read_text(encoding="utf-8")
    sensorless_car = SENSORLESS_CAR.read_text(encoding="utf-8")

    assert "Vehicle = AutoracerStageB_NDTSensorCar" in ndt_testrun
    assert "SensorCluster.N = 1" in ndt_sensor_car
    assert "Sensor.2.Active = 1" in ndt_sensor_car
    assert "Sensor.2.Ref.Cluster = 0" in ndt_sensor_car
    assert "SensorCluster.N = 0" in sensorless_car
    assert "Sensor.2.Active = 0" in sensorless_car


def test_realtime_ndt_script_defaults_to_sensor_enabled_testrun():
    source = NDT_RUN_SCRIPT.read_text(encoding="utf-8")

    assert 'TESTRUN="${TESTRUN:-AutoracerStageB_UrbanRoute271_NDT}"' in source


def test_realtime_ndt_script_aliases_movie_road_cache_for_copied_testrun():
    source = NDT_RUN_SCRIPT.read_text(encoding="utf-8")

    assert "ensure_movie_road_cache_alias" in source
    assert "AutoracerStageB_UrbanRoute271.ubuntu" in source
    assert "AutoracerStageB_UrbanRoute271_NDT.ubuntu" in source


def test_stage_b_ndt_testrun_keeps_driver_route_generation_for_movie_gpusensor():
    ndt_testrun = NDT_TESTRUN.read_text(encoding="utf-8")
    sensorless_testrun = TESTRUN.read_text(encoding="utf-8")

    assert "DrivMan.Man.0.LongStep.0.Dyn = Driver 1 0" in ndt_testrun
    assert "DrivMan.Man.0.LatStep.0.Dyn = Driver 0" in ndt_testrun
    assert "DrivMan.Man.0.LongStep.0.Dyn = Manual" in sensorless_testrun
