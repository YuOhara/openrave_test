for file in `\find ~/Result/ -maxdepth 1 -mindepth 1 -type d`; do
    # TODO
    echo $file
    cp $file/* ~/.ros/ -r
    rosrun openrave_test grasp_finder_with_load.py > $file/result.txt
done
