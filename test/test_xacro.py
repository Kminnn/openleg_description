"""Description parsing tests."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro


def test_main_xacro_parses_for_rviz_and_gazebo():
    package_share = get_package_share_directory("openleg_description")
    model = os.path.join(package_share, "urdf", "main.urdf.xacro")
    controllers = os.path.join(package_share, "config", "controllers.yaml")

    for use_gazebo in ("true", "false"):
        document = xacro.process_file(
            model,
            mappings={"use_gazebo": use_gazebo, "controllers_file": controllers},
        )
        assert document.documentElement.getAttribute("name") == "openleg"


def test_gazebo_model_uses_physics_and_base_imu():
    package_share = get_package_share_directory("openleg_description")
    model = os.path.join(package_share, "urdf", "main.urdf.xacro")
    controllers = os.path.join(package_share, "config", "controllers.yaml")
    document = xacro.process_file(
        model,
        mappings={"use_gazebo": "true", "controllers_file": controllers},
    )
    robot = ET.fromstring(document.toxml())

    base_mass = robot.find("./link[@name='base_link']/inertial/mass")
    assert base_mass is not None
    assert float(base_mass.get("value")) > 0.0

    with open(controllers, encoding="utf-8") as controller_file:
        controller_config = controller_file.read()
    assert "position_proportional_gain: 0.2" in controller_config

    sensor = robot.find("./gazebo[@reference='base_link']/sensor[@name='base_imu']")
    assert sensor is not None
    assert sensor.get("type") == "imu"
    assert sensor.findtext("pose") == "0 0 0.070 0 0 0"
    assert sensor.findtext("topic") == "/imu"

    plugins = robot.findall("./gazebo/plugin")
    filenames = {plugin.get("filename") for plugin in plugins}
    assert "gz-sim-velocity-control-system" not in filenames
    assert "gz-sim-odometry-publisher-system" not in filenames

    world = ET.parse(os.path.join(package_share, "worlds", "openleg.sdf"))
    world_plugins = {
        plugin.get("filename") for plugin in world.findall("./world/plugin")
    }
    assert "gz-sim-imu-system" in world_plugins

    ground_collision = world.find(
        "./world/model[@name='ground_plane']/link/collision"
    )
    assert ground_collision is not None
    assert ground_collision.findtext("./surface/friction/ode/mu") == "1.2"
    assert ground_collision.findtext("./surface/friction/ode/mu2") == "1.2"
