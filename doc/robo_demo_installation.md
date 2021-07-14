# Installation – Use ROS MoveIt on a real Panda robot
###### David Yackzan

### Versions
  * Desktop OS: Ubuntu 18.04
  * ROS: Melodic
  * Panda system: v3.0.0
  * Libfranka: 0.7.1
    - Compatable Libfranka and Panda system versions: https://frankaemika.github.io/docs/libfranka_changelog.html
  * MoveIt: 1
  * Franka Ros: melodic-devel branch

*replace .zsh extension with whatever shell you are using (ex: bash $\rightarrow$ .bash)*

### Install ROS Melodic (assuming Ubuntu 18.04)
Source: https://wiki.ros.org/melodic/Installation/Ubuntu
1. Set up sources.list: \
`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
2. Set up keys: \
`sudo apt install curl # if you haven't already installed curl` \
`curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
3. Install \
`sudo apt update` \
`sudo apt install ros-melodic-desktop-full`
4. Set up environment \
`echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc` \
`source ~/.zshrc`
5. Install dependencies and initialize rosdep \
`sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential` \
`sudo apt install python-rosdep` \
`sudo rosdep init` \
`rosdep update`

### Install MoveIt 1 from Source (Assuming ROS Melodic)
Source: https://moveit.ros.org/install/source/
1. Update packages \
`rosdep update` \
`sudo apt update` \
`sudo apt dist-upgrade`
2. Install tools \
`sudo apt install python3-wstool python3-catkin-tools clang-format-10 python3-rosdep`
3. Create a new workspace \
`mkdir ~/ws_moveit` \
`cd ~/ws_moveit`
4. Source installed ros version \
`source /opt/ros/melodic/setup.zsh`
5. Pull down repositories and build from root directory of local workspace \
`wstool init src` \
`wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall` \
`wstool update -t src` \
`rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}` \
`catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release`

### Install libfranka and franka_ros
Source: https://frankaemika.github.io/docs/installation_linux.html
##### libfranka
1. Uninstall existing installations to avoid conflicts \
`sudo apt remove "*libfranka*"`
2. Install dependencies \
`sudo apt install build-essential cmake git libpoco-dev libeigen3-dev`
3. Download source code (downloaded in home directory here) \
`cd ~` \
`git clone --recursive https://github.com/frankaemika/libfranka` \
`cd libfranka`
4. Checkout version and update \
`git checkout 0.7.1` \
`git submodule update`
5. In the libfranka root directory, create a build directory and run CMake \
`mkdir build` \
`cd build` \
`cmake -DCMAKE_BUILD_TYPE=Release ..` \
`cmake --build .`
##### franka_ros
1. Clone the franka_ros repository into the MoveIt workspace src folder \
`cd ~/ws_moveit/src` \
`git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros` \
2. Checkout the melodic version of franka ros \
`cd franka_ros` \
`git checkout melodic-devel`
3. Install any missing dependencies and build the packages \
`cd ..` \
`rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka` \
`catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build`\
`source devel/setup.zsh`

*Follow instructions in the source link to install real-time kernel if it has not been installed already*

### (Next): clone github repository into src file with code for demo
