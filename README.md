See `main` for training bot instructions

# Setup for zed (because zed is stupid and lame) 
`https://github.com/stereolabs/zed-ros2-wrapper#build-the-package`
```
mkdir -p ~/ros2_ws/src/ # create your workspace if it does not exist
cd ~/ros2_ws/src/ #use your current ros2 workspace folder
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc # automatically source the installation in every new bash (optional)
source ~/.bashrc
```
