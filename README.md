# openrave_test


```
# cd to your catkin ws
wstool set --git openrave_test https://github.com/YuOhara/openrave_test.git
wstool set --git or_urdf https://github.com/personalrobotics/or_urdf
wstool update openrave_test or_urdf
cd openrave_test
rosdep install --from-paths . --ignore-src -y -r -n
catkin bt
source sample/install_sample.sh
# source rosbash
cd scripts
ipython grasp_finder_with_load.py
```