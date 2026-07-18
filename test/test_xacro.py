"""Description parsing tests."""

import os

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
