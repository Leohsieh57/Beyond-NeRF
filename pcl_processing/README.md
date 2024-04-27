## Registration Package
1. Under `catkin_ws`, run`rm -rf build/ devel/` then `catkin_make` 
2. For visualization check, run the following (change `/data/kitti/raw_data` into your kitti dir)
	`roslaunch pcl_processing test_regist.launch raw_data:=/data/kitti/raw_data`

2.  Modify `pcl_processing/scripts/pcl_server.py` to incorporate registration. The current visualization uses GPS pose to cheat. 