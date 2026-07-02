source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh

if [ -f .venv/bin/activate ]; then
    source .venv/bin/activate
fi
