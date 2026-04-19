# manus_ros2_msgs (stub)

**This is a stub.** Message interface files mirror the public [Manus ROS2
docs](https://docs.manus-meta.com/3.1.0/Plugins/SDK/ROS2/ros2%20package/)
so the workspace builds and can be tested without the commercial Manus SDK.

When deploying against a real Manus glove, replace this directory with the
official `manus_ros2_msgs` package shipped with the Manus SDK:

```
# from your freshly extracted Manus SDK:
rm -rf ~/hand_ws/src/manus_glove/manus_ros2_msgs
cp -r <manus_sdk>/ros2/manus_ros2_msgs ~/hand_ws/src/manus_glove/
cd ~/hand_ws && colcon build --symlink-install --packages-select manus_ros2_msgs
```

The fields defined here cover only what the retargeter / slider UI read
today. The official package may define additional fields — all of which
remain compatible with this subset.
