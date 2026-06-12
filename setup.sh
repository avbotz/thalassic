source /opt/ros/jazzy/setup.bash
source install/setup.sh

export CYCLONEDDS_URI="file://$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/cyclonedds.xml"

if [ -f .venv/bin/activate ]; then
    source .venv/bin/activate
fi
