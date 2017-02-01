rm ~/.ros/mesh.ply
rm ~/.ros/mesh_estimated.ply
rm ~/.ros/temp_box.txt
wget http://www.jsk.t.u-tokyo.ac.jp/~ohara/graspfinder_sample/mesh.ply -P ~/.ros/
wget http://www.jsk.t.u-tokyo.ac.jp/~ohara/graspfinder_sample/mesh_estimated.ply -P ~/.ros/
wget http://www.jsk.t.u-tokyo.ac.jp/~ohara/graspfinder_sample/temp_box.txt -P ~/.ros/
mkdir ~/.ros/grasps
