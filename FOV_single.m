clear all
close all

FOV_degrees = [54.14,41.67];
plane_of_stitching = [0,0,1,0]';
R_bounds = [-pi/3,1,pi/3];
Rz_bounds = [-pi/2,pi/2,pi];
T_bounds = [2,5,7.5];
scene_depth = 16.5;

FOV_rads = pi/180*FOV_degrees;



%% Naive Approach
cc = 1;

t = [2.0000    7.0000    7.0000    2.0000    2.0000];
R = [-0.0472   -1.0472   -1.0472   -1.0472   -1.0472];
Rz = [1.5708         0    3.1416         0    3.1416];

for j = 1:5
    camera_t(j,:) = [t(j)*sin(Rz(j)),t(j)*cos(Rz(j)),-scene_depth];
end

camera_R(:,:,1) = rotateMat(R(1)*cos(Rz(1)),R(1)*-sin(Rz(1)),Rz(1));
camera_R(:,:,2) = rotateMat(R(2)*cos(Rz(2)),R(2)*-sin(Rz(2)),Rz(2));
camera_R(:,:,3) = rotateMat(R(3)*cos(Rz(3)),R(3)*-sin(Rz(3)),Rz(3));
camera_R(:,:,4) = rotateMat(R(4)*cos(Rz(4)),R(4)*-sin(Rz(4)),Rz(4));
camera_R(:,:,5) = rotateMat(R(5)*cos(Rz(5)),R(5)*-sin(Rz(5)),Rz(5));

[area,full_poly] = array_area(FOV_rads, camera_R,camera_t,plane_of_stitching);

area

figure
fill(full_poly(:,1),full_poly(:,2),'b')


