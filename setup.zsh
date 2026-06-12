source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

export CYCLONEDDS_URI="file://${0:a:h}/cyclonedds.xml"

if [ -f .venv/bin/activate ]; then
    source .venv/bin/activate
fi
