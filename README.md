# openrave_test

```
# cd to your catkin ws
wstool set --git openrave_test https://github.com/YuOhara/openrave_test.git -y
# wstool set --git or_urdf https://github.com/personalrobotics/or_urdf
# wstool update openrave_test or_urdf
cd openrave_test
rosdep install --from-paths . --ignore-src -y -r -n
catkin bt
# install sample
source sample/install_sample.sh
# install external
wget http://lmb.informatik.uni-freiburg.de/resources/binaries/pami2013_MoSeg.tar.gz
tar xfvz pami2013_MoSeg.tar.gz
# source rosbash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)
roscd openrave_test/scripts/
ipython grasp_finder_with_load.py
```