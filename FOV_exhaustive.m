%% Exhaustive Optimization
% This script runs an exhaustive optimization over all possible camera placements for the 
% cross shaped trocar array. 

% Set up the workspace
clear all
close all

% Tunable Parameters
FOV_degrees = [54.14,41.67];
plane_of_stitching = [0,0,1,0]';
R_bounds = [-pi/3,1,pi/3];
Rz_bounds = [-pi/2,pi/2,pi];
T_bounds = [2,5,7.5];
scene_depth = 16.5;


FOV_rads = pi/180*FOV_degrees;

% Optimization
num_cameras = 5;
cc = 0;
for Rz = Rz_bounds(1):Rz_bounds(2):Rz_bounds(3)
    for t = T_bounds(1):T_bounds(2):T_bounds(3)
        cc = cc+1;
        positionlist(:,cc) = [Rz, t]; % Choose arm, then translation
    end
end

camera_poses = combnk(1:cc,num_cameras);
cc = 0;
for i = 1:length(camera_poses(:,1))
    for j = 1:num_cameras
        pose = positionlist(:,camera_poses(i,j));
        Rz(j) = pose(1);
        t(j) = pose(2);
        camera_t(j,:) = [t(j)*sin(Rz(j)),t(j)*cos(Rz(j)),-scene_depth];
    end
    for R1 = R_bounds(1):R_bounds(2):R_bounds(3)
        for R2 = R_bounds(1):R_bounds(2):R_bounds(3)
            for R3 = R_bounds(1):R_bounds(2):R_bounds(3)
                for R4 = R_bounds(1):R_bounds(2):R_bounds(3)
                    for R5 = R_bounds(1):R_bounds(2):R_bounds(3)
                        cc = cc + 1
                        camera_R(:,:,1) = rotateMat(R1*cos(Rz(1)),R1*-sin(Rz(1)),Rz(1));
                        camera_R(:,:,2) = rotateMat(R1*cos(Rz(2)),R1*-sin(Rz(2)),Rz(2));
                        camera_R(:,:,3) = rotateMat(R1*cos(Rz(3)),R1*-sin(Rz(3)),Rz(3));
                        camera_R(:,:,4) = rotateMat(R1*cos(Rz(4)),R1*-sin(Rz(4)),Rz(4));
                        camera_R(:,:,5) = rotateMat(R1*cos(Rz(5)),R1*-sin(Rz(5)),Rz(5));
                        
                        [area,full_poly] = array_area(FOV_rads, camera_R,camera_t,plane_of_stitching);
                        %cc, t, Rang, area,overlap_areas
                        exhaustive_output(cc,:) = [cc,area,t,R1*180/pi,R2*180/pi,R3*180/pi,R4*180/pi,R5*180/pi,Rz];
                        exhaustive_drawings{cc} = full_poly;
                        
                        
                    end
                end
            end
        end
    end
end

exhaustive_sorted_out = sortrows(exhaustive_output,2);
max_id = exhaustive_sorted_out(end,1);
exhaustive_poly = exhaustive_drawings{max_id};

exhaustive_Optimal = exhaustive_sorted_out(end,3:end)
exhaustive_area = exhaustive_sorted_out(end,2)

figure
fill(exhaustive_poly(:,1),exhaustive_poly(:,2),'r')