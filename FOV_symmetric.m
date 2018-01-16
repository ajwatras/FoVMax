clear all
close all

FOV_degrees = [54.14,41.67];
plane_of_stitching = [0,0,1,0]';
R_bounds = [-pi/3,1,pi/3];
Rz_bounds = [-pi/2,pi/2,pi];
T_bounds = [2,5,7.5];
scene_depth = 16.5;

FOV_rads = pi/180*FOV_degrees;
%% Symmetric approach


cc = 1;
for t = T_bounds(1):T_bounds(2):T_bounds(3)
    for Rang = R_bounds(1):R_bounds(2):R_bounds(3)
        
        %t = 3.5;
        %Rang = -pi/6;
        
        camera_t(2,:) = [-t,0,-scene_depth];
        camera_t(3,:) = [0,t,-scene_depth];
        camera_t(4,:) = [t,0,-scene_depth];
        camera_t(5,:) = [0,-t,-scene_depth];
        camera_t(1,:) = [1,0,-scene_depth];
        
        %camera_t = [camera1_t;camera2_t;camera3_t;camera4_t;camera5_t];
        
        camera_R(:,:,2) = rotateMat(0,Rang,-pi/2);
        camera_R(:,:,3) = rotateMat(Rang,0,0);
        camera_R(:,:,4) = rotateMat(0,-Rang,pi/2);
        camera_R(:,:,5) = rotateMat(-Rang,0,pi);
        camera_R(:,:,1) = rotateMat(0,0,0);
        
        [area,full_poly] = array_area(FOV_rads, camera_R,camera_t,plane_of_stitching);
        %cc, t, Rang, area,overlap_areas
        symmetric_output(cc,:) = [cc,area,t,Rang*180/pi];
        symmetric_drawings{cc} = full_poly;
        cc = cc + 1;
    end
end
symmetric_sorted_out = sortrows(symmetric_output,2);
max_id = symmetric_sorted_out(end,1);
symmetric_poly = symmetric_drawings{max_id};

symmetric_Optimal = symmetric_sorted_out(end,3:end)
symmetric_area = symmetric_sorted_out(end,2)
figure
fill(symmetric_poly(:,1),symmetric_poly(:,2),'b')

x = T_bounds(1):T_bounds(2):T_bounds(3);
x_shape = size(x);
x_shape = x_shape(2);
y = R_bounds(1):R_bounds(2):R_bounds(3);
y_shape = size(y);
y_shape = y_shape(2);
z = reshape(symmetric_output(:,2),[y_shape,x_shape]);

figure
surf(x,180*y/pi,z)

