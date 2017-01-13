mkdir ~/Result/pictures
cd ~/Result/pictures
for file in `\find ~/Result/ -maxdepth 1 -mindepth 1 -type d`; do
    # TODO
    echo $file
    num=$(echo $file | sed -e 's,/home/leus/Result/result,,g')
    cp $file/* ~/.ros/ -r
    rosrun openrave_test grasp_finder_with_load_show.py
    cp tmp.png $num.png
done
