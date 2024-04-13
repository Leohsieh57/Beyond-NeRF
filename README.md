
## Beyond-NeRF
### Dependencies 
 1. `sudo apt install libgoogle-glog-dev`
 2. ROS noetic (Ubuntu 20.04)

### KITTI Visualization Test 
 1. Download one sequence of [KITTI Raw Data](https://www.cvlibs.net/datasets/kitti/raw_data.php) ([calibration] & [synced+rectified data])
 2. unzip them under the same directory
 3. `roslaunch scan_filter test_kitti.launch raw_data:=[your-kitti-dir] date:=[see-pic] id:=[see-pic]`
    
 ![enter image description here](https://i.imgur.com/9gVLkiF.png)
