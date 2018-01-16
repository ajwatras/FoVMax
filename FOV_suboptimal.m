clear all
close all

FOV_degrees = [54.14,41.67];
FOV_rads = pi/180*FOV_degrees;
plane_of_stitching = [0,0,1,0]';
scene_depth = 16.5;
num_cameras = 5;
thresh = 25;
Rz_bounds = [-pi/2,pi/2,pi];
R_bounds = [-pi/3,pi/6,pi/3];
T_bounds = [2.5,2.75,7.5];

%% Greedy Approach

%A better coverage region might work better. 
coverage_dim = [0,0]; % Define the region which must be covered by the cameras
coverage_reg = [-coverage_dim(1),-coverage_dim(2);-coverage_dim(1),...
    coverage_dim(2);coverage_dim(1),coverage_dim(1);coverage_dim(1),-coverage_dim(2)];
%coverage_reg = [-20,-10;-20,10;-10,10;-10,20;10,20;10,10;20,10;20,-10;10,-10;10,-20;-10,-20;-10,-10;-20,-10];

n = 5;
%camera_R = repmat(eye(3),[1,1,n]); % initialize camera rotations
%camera_t = repmat([0,0,-16],[n,1]); % initialize camera translations
covered_poly = [0];
camera_pose = [];

for i = 1:n
    [camera_pose(i,:),covered_poly,cameras] = maxCoverage(FOV_rads,plane_of_stitching, coverage_reg, covered_poly,T_bounds,R_bounds,scene_depth,thresh,camera_pose);
    figure
    hold on
    fill(coverage_reg(:,1),coverage_reg(:,2),'r');
    fill(covered_poly(:,1),covered_poly(:,2),'b');
    
end

camera_pose
greedy_area = polyarea(covered_poly(:,1),covered_poly(:,2))
greedy_poly = covered_poly;


%figure
%fill(covered_poly(:,1),covered_poly(:,2),'b');
