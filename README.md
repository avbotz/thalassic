# Thalassic

Build: `colcon build`

Install dependencies: `rosdep update --rosdistro=jazzy && rosdep -i install --from-paths src --rosdistro jazzy -y --skip-keys="pcl"`

Run sim: `ros2 launch sub_bringup sim_launch.py`

Run pool test: `ros2 launch sub_bringup pool_test_launch.py`


## Troubleshooting

If you are having issues with DDS (`rmw_create_node: failed to create domain`), run `pkill -f ros`.
