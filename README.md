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
            2. Use segment model to get better box
          
          
4> Results: \



5> Usage: \
            1. for mono: build/mono_tum Full data/rgbd_dataset_freiburg3_long_office_household/ Vocabulary/ORBvoc.bin Examples/Monocular/TUM3.yaml online  \
            2. for rgbd: build/rgbd_tum Vocabulary/ORBvoc.bin Examples/RGB-D/TUM3.yaml data/rgbd_dataset_freiburg3_long_office_household/ Examples/RGB-D/associations.txt Full online   

          
          
