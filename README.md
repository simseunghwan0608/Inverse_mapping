# Inverse_mapping
Change lane_point in image to baselink by using Inverse_mapping




# Table of Contents

1. Find extrinsic matrix of camera. (your should have intrinsic matrix in purpose!)
2. Change parameters in the file
3. Subscribe image from yolo and you can find out points on baselink

***baselink is not rear shaft of the car, it is z = 0 place of the cameralink** 


# Get extrinsic matrix

Go into BEV_MATLAB and run 

get_extrinsic_baselink_camer_camera.m

you can find how to use it at

https://nl.mathworks.com/help/driving/ug/calibrate-a-monocular-camera.html
