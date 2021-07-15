# EAO-SLAM-Improve 
# VERSION 1.0 
1> Reference: \
                  https://github.com/yanmin-wu/EAO-SLAM \
                  https://github.com/xiaoxifuhongse/ORB-SLAM-RGBD-with-Octomap \
                  https://github.com/bianjingshan/MOT-deepsort 
              
2> Done:  \
            1. Opencv4 Support \
            2. YOLO Model support for online \
            3. Fix some bugs \
            4. Add 2d-object-tracker for data association[for Mono mode] \
            5. Add RGB-D mode for pointcloud viewer and octomap \
            6. Other TUM Sequences support

3> To Do: \
            1. Add RGB-D Depth Info to fix uncertainty of mono depth \
            2. Use segment model to get better box \
            3. Get better Tracker 
          
          
4> Results: \
            Only compare with data association here: \
            1. EAO results with track 
            ![EAO-Only](https://github.com/Be997398715/EAO-SLAM-Improve/blob/v1.0/EAO-SLAM-master-improve/figures/EAO-only.png) EAO-Only 
            ![EAO-With-Track](https://github.com/Be997398715/EAO-SLAM-Improve/blob/v1.0/EAO-SLAM-master-improve/figures/eao-with-track.png) EAO-With-Track \
            2. IOU results with track 
            ![Iou-With-Track](https://github.com/Be997398715/EAO-SLAM-Improve/blob/v1.0/EAO-SLAM-master-improve/figures/iou-with-track.png) Iou-With-Track  \
            3. FULL results with track 
            ![Full-Only](https://github.com/Be997398715/EAO-SLAM-Improve/blob/v1.0/EAO-SLAM-master-improve/figures/full.png) Full-Only
            ![Full-With-Track](https://github.com/Be997398715/EAO-SLAM-Improve/blob/v1.0/EAO-SLAM-master-improve/figures/full-with-track.png) Full-With-Track \
            These results test online and show 2d-track is useful for data association, while using better tracker, results getting better.
            
            


5> Usage: \
            1. for mono: build/mono_tum Full data/rgbd_dataset_freiburg3_long_office_household/ Vocabulary/ORBvoc.bin Examples/Monocular/TUM3.yaml online  \
            2. for rgbd: build/rgbd_tum Vocabulary/ORBvoc.bin Examples/RGB-D/TUM3.yaml data/rgbd_dataset_freiburg3_long_office_household/ Examples/RGB-D/associations.txt Full online   

          
          
