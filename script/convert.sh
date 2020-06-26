#!/usr/bin/env python
DAY="2019-06-14"
bag_folder="/mnt/disk0/ILIAD_remote/rosbags/Orkla/2019-06_Campaign_2/${DAY}_processesd/"
bag_name="Orkla_${DAY}_001.bag_edited.bag"
txt_folder="/mnt/disk0/ILIAD_remote/rosbags/Orkla/2019-06_Campaign_2/${DAY}_processed_corrections/"
finished_folder="/mnt/disk0/ILIAD_remote/rosbags/Orkla/2019-06_Campaign_2/${DAY}-final/"
rosrun tf_workshop insertTxtToBag.py $bag_name $bag_folder $txt_folder $finished_folder
#                                   bag file                           
