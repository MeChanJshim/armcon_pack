# Contact runtime regressions

Source `/opt/ros/humble/setup.bash` and the workspace `install/setup.bash`.
Tests use localhost and separate ROS domains; no physical robot is required.

```bash
ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 python3 tests/motion_input_regression.py
```

This verifies invalid/missing/nonfinite startup messages, normal commands,
reordered joints, joint timeout, GForce execution and force feedback timeout.

The F/T node regression is built with `Y2FT_AQ` when `BUILD_TESTING` is enabled:

```bash
cd ~/armcon_ws/build/Y2FT_AQ
ctest -R '^ft_node_regression$' --output-on-failure
```

It sends gravity-only data from a localhost UDP sensor, delays the robot pose,
calibrates at a nonidentity orientation, changes orientation, and disconnects.
It checks gravity force/torque cancellation and absence of stale publication.

From the armcon_pack root, run the transport tests with sanitizers:

```bash
g++ -std=c++17 -Wall -Wextra -fsanitize=address,undefined -g -pthread \
  -I Y2FT_AQ/include -I Y2Matrix/include \
  tests/ft_transport_regression.cpp Y2FT_AQ/src/FT_EtherGet.cpp \
  Y2FT_AQ/src/FT_eCANGet.cpp -o /tmp/ft_transport_regression
/tmp/ft_transport_regression
```

This verifies fresh UDP samples, invalid-data recovery, unique-sample tare,
fragmented and coalesced TCP frames, paired force/moment data and disconnects.

With all three repositories built in the workspace, check the installed graph:

```bash
ROS_DOMAIN_ID=214 ROS_LOCALHOST_ONLY=1 python3 tests/contact_graph_smoke.py
```

This starts simulator, robot controller and contact estimator in an isolated
ROS domain and checks process health, finite robot feedback and sensor frames.
It is a connectivity smoke test, not a polishing performance measurement.
