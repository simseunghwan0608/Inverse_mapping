% STEP 1
% capture image 1

camList = webcamlist;
cam =  webcam(1);
preview(cam);
img_captured = snapshot(cam);
imwrite(img_captured,'images/calibration_baselink_camera_img_1.png');
clear cam;

% STEP 2

img = imread('images/calibration_baselink_camera_img1.png');
[imagePoints, boardSize] = detectCheckerboardPoints(img);
squareSize = 0.025; % Square size in meters (our teams default is 0.025);
worldPoints = generateCheckerboardPoints(boardSize,squareSize);
patternOriginHeight = 0; % Pattern is on ground
[pitch, yaw, roll, height] = estimateMonoCameraParameters(intrinsic, ...
                            imagePoints, worldPoints, patternOriginHeight);

% check out the pitch, yaw, roll, height if height is correct and
%  yaw and roll will be close at 0 and pitch will be positive, check out
%  it is degrees or radians.

%STEP 3
% use imageToVehicle and check out it is correct

%
% if using a matlab the pitch is with a degree!!!
%

