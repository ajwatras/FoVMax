%% Naive optimization
% This script will attempt to optimize camera placement for a set of cameras placed on a 
% cross shaped trocar camera array. It follows a naive approach where the cameras are only
% optimized over arm length on the cross shaped array. 

% Set up workspace
clear all
close all

% Tunable Parameters
FOV_degrees = [54.14,41.67];
plane_of_stitching = [0,0,1,0]';
R_bounds = [-pi/3,1,pi/3];
Rz_bounds = [-pi/2,pi/2,pi];
T_bounds = [2,5,7.5];
scene_depth = 16.5;
thresh = 25;

FOV_rads = pi/180*FOV_degrees;



%% Naive Approach
cc = 1;
for t = T_bounds(1):T_bounds(2):T_bounds(3)
    
    %t = 3.5;
    %Rang = -pi/6;
    
    camera_t(2,:) = [-t,0,-scene_depth];
    camera_t(3,:) = [0,t,-scene_depth];
    camera_t(4,:) = [t,0,-scene_depth];
    camera_t(5,:) = [0,-t,-scene_depth];
    camera_t(1,:) = [1,0,-scene_depth];
    
    %camera_t = [camera1_t;camera2_t;camera3_t;camera4_t;camera5_t];
    
    camera_R(:,:,2) = rotateMat(0,0,-pi/2);
    camera_R(:,:,3) = rotateMat(0,0,0);
    camera_R(:,:,4) = rotateMat(0,0,pi/2);
    camera_R(:,:,5) = rotateMat(0,0,pi);
    camera_R(:,:,1) = rotateMat(0,0,0);
    
    [area,full_poly] = array_area(FOV_rads, camera_R,camera_t,plane_of_stitching,thresh);;
    [con_area,cut_poly] = peelPotato(full_poly');
    %cc, t, Rang, area,overlap_areas
    naive_output(cc,:) = [cc,area,con_area,t,0];
    naive_drawings{cc} = full_poly;
    naive_cut{cc} = cut_poly';
    cc = cc + 1;
end

%Sort for max area
naive_sorted_out = sortrows(naive_output,2);
max_id = naive_sorted_out(end,1);
naive_poly = naive_drawings{max_id};

% Display max area
naive_max_area_opt = naive_sorted_out(end,4:end)
naive_area = naive_sorted_out(end,2)

figure
fill(naive_poly(:,1),naive_poly(:,2),'b')

%Sort for max convex region
naive_sorted_out = sortrows(naive_output,3);
max_id = naive_sorted_out(end,1);
naive_poly_con = naive_drawings{max_id};
naive_cut_drawn = naive_cut{max_id};

% Display max area
naive_con_opt = naive_sorted_out(end,3:end)
naive_area_con = naive_sorted_out(end,2)

figure
hold on
fill(naive_poly_con(:,1),naive_poly_con(:,2),'b')
fill(naive_cut_drawn(:,1),naive_cut_drawn(:,2),'r')


