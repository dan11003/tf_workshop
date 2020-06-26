#!/usr/bin/env python
DAY="2019-06-14"
bag_folder="/mnt/disk0/ILIAD_remote/rosbags/Orkla/2019-06_Campaign_2/${DAY}_processed/"
#bag_name="Orkla_2019-06-13_001.bag_edited.bag"
txt_folder="/mnt/disk0/ILIAD_remote/rosbags/Orkla/2019-06_Campaign_2/${DAY}_processed_corrections/"
finished_folder="/mnt/disk0/ILIAD_remote/rosbags/Orkla/2019-06_Campaign_2/${DAY}_final/"
python insertbag.py $bag_name $bag_folder $txt_folder $finished_folder
#                                   bag file
for bag_name in `ls $bag_folder`; do
    echo "${bag_name} will be converted into folder ${finished_folder}"
    python insertbag.py $bag_name $bag_folder $txt_folder $finished_folder
done

