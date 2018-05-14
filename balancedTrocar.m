%% Balanced optimization
% This script will attempt to optimize camera placement for a set of cameras placed on a
% cross shaped trocar camera array. It follows a balanced approach where
% the cameras are not allowed to be placed with more than 2 cameras on any
% arm of the trocar, but are otherwise free to be placed.

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
bal_thresh = 2;

%% Generate Allowable Camera Positions

Rotations = R_bounds(1):R_bounds(2):R_bounds(3);
Translations = T_bounds(1):T_bounds(2):T_bounds(3);
Rz = Rz_bounds(1):Rz_bounds(2):Rz_bounds(3);

Positions = combvec(Rotations, Translations, Rz)';
[n_pos, n_params] = size(Positions);
n_trans = length(Rotations);

%% Choose and Evaluate Camera Positions
cc = 1; % Counter for number of evaluated arrays.

%Saved parameters for maximal area
max_area = -1;
max_cameras = zeros(5,3);
max_FoV = [0;0];

% Saved parameters for maximal convex shape.
max_peeled_area = -1;
max_peeled_area_nonconvex = -1;
max_peeled_cameras = zeros(5,3);
max_peeled_FoV = [0;0];
max_peeled_FoV_nonconvex = [0;0];

% Loop over possible camera arrays.
for cam1 = 1:n_pos - 4*n_trans
    for cam2 = cam1 + n_trans:n_pos - 3*n_trans
        for cam3 = cam2 + n_trans:n_pos - 2*n_trans
            for cam4 = cam3 + n_trans:n_pos - n_trans
                for cam5 = cam4 + n_trans:n_pos
                    % Grab the parameters for the chosen camera positions
                    Cameras = [Positions(cam1,:);Positions(cam2,:);Positions(cam3,:);Positions(cam4,:);Positions(cam5,:)];

                    % Evaluate whether Cameras satisfies the required
                    % constraints.
                    if isBalanced(Cameras,bal_thresh)
                        [Area,FOV] = FOV_eval(Cameras, scene_depth,FOV_rads,plane_of_stitching,thresh); % Evaluate FOV
                        
                        % Evaluate Maximum Area
                        if Area > max_area
                            max_area = Area;
                            max_cameras = Cameras;
                            max_FoV = FOV;
                        end
                        
                        [peeled_area,peeled_FoV] = peelPotato(FOV');
                        % Evaluate Maximal convex shape.
                        if peeled_area > max_peeled_area
                            max_peeled_area = peeled_area;
                            max_peeled_area_nonconvex = Area;
                            max_peeled_cameras = Cameras;
                            max_peeled_FoV = peeled_FoV;
                            max_peeled_FoV_nonconvex = FOV;
                        end
                        
                    end
                    
                    
                    
                end
            end
        end
    end
end
%% Display results
disp(['Maximal Coverage Area: ', num2str(max_area),'\n'])
disp(['Area of Maximal Convex Region: ', num2str(max_peeled_area),'\n'])
disp(['Total Area for Convex Optimization: ', num2str(max_peeled_area_nonconvex),'\n'])

figure
hold on
fill(max_FoV(1,:),max_FoV(2,:),'r')




%% Extra Functions

function flag = isBalanced(cameras,bal_thresh)
% Checks to see that in the camera poses given by "cameras", there are at
% most "bal_thresh" cameras on any given arm.

    Rz = cameras(:,3);                          % Isolates arm parameter
    UniqueRz = unique(Rz);                      % Gives unique arms used
    num_cameras_per_arm = hist(Rz,UniqueRz);    % Counts cameras per arm.
    if max(num_cameras_per_arm) > bal_thresh
        flag = false;
    else
        flag = true;
    end
end