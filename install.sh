
ROS_WS_DIR=$(pwd)

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update

sudo apt install -y build-essential cmake cppcheck curl git gnupg libeigen3-dev libgles2-mesa-dev lsb-release pkg-config protobuf-compiler python3-dbg python3-pip python3-venv qtbase5-dev ruby software-properties-common sudo wget

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update

# Set versions to install
DIST=jazzy

sudo apt install -y python3-rosdep python3-rosinstall-generator python3-vcstool ros-${DIST}-desktop

git submodule update --init --recursive

vcs import src < src/sub_drivers/waterlinked_dvl/ros2.repos

# Stonefish
mkdir /tmp/stonefish
cd /tmp/stonefish
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y python3-pip libglm-dev libsdl2-dev libfreetype6-dev
git clone https://github.com/kethan1/stonefish
cd stonefish
git switch fixes-merged
mkdir build && cd build
cmake ..
make -j4
sudo make install
cd $ROS_WS_DIR

sudo rosdep init
rosdep update
rosdep install -y -i --from-paths src --skip-keys="pcl nlohmann_json"

colcon build
