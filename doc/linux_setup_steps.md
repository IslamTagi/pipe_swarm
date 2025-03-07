# WSL ROS Package Setup

## Setup ssh key

You do not need a password. Click `enter` when prompted.

```bash
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
```

In Azure, open `User Settings` -> `SSH public keys` -> `Add`. Copy and paste the public key then save. To view your public key, run the following in WSL.

```bash
cat ~/.ssh/id_rsa.pub
```

## Cloning Repo

Create a `source_code` directory and move into it

```bash
mkdir ~/source_code
cd ~/source_code
```

Clone the git repository

```bash
git clone --recurse-submodules git@ssh.dev.azure.com:v3/islamtagi/Pipe%20Swarm%20Project/Pipe%20Swarm%20Project pipe_swarm --branch=develop
```

Once cloned, navigate into the ros_ws, build and source the workspace

```bash
cd ~/source_code/pipe_swarm/ros_ws
colcon build --symlink-install
colcon build
source install/setup.bash
```

## Test Build

Once successfully built, run the spawn launch script

```bash
ros2 launch pipe_swarm spawn_multi_agent.launch.py
```

## Sourcing Packages on WSL Start-up

Make sure the following lines are at the end of your `~/.bashrc` file

```bash
source /opt/ros/humble/setup.bash
source ~/source_code/pipe_swarm/ros_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$(whoami)/source_code/pipe_swarm/ros_ws/src/pipe_swarm/models
```

<br>

## Debugging tips

If a package is not built due to missing dependencies use

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r
```

Once installed and working, add any missing packages to the `~/source_code/pipe_swarm/ros_ws/src/pipe_swarm/package.xml` file
