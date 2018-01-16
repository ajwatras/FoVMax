clear all
close all

% Current settings should take approx 5 hours.
FOV_degrees = [54.14,41.67];
FOV_rads = pi/180*FOV_degrees;
plane_of_stitching = [0,0,1,0]';
scene_depth = 16.5;
thresh = 25;

num_cameras = 5;
Rz_bounds = [-pi/2,pi/2,pi];
R_bounds = [-pi/8,pi/16,pi/8];
T_bounds = [-4,1,4];

%Slow, Approx 3 Days
%R_bounds = [-pi/3,pi/6,pi/3];
%T_bounds = [2,2.75,7.5];

%Medium, Approx 1 Days
%R_bounds = [-pi/3,pi/6,pi/3];
%T_bounds = [2,2.75,7.5];

%Fast settings should take 40 min.
%R_bounds = [-pi/3,1,pi/3];
%T_bounds = [2,5,7.5];



%% Bounds
min_bound = -1;
max_bound = -1;
max_thresh = -1;

cc = 1;
for tx = T_bounds(1):T_bounds(2):T_bounds(3)
    for ty = T_bounds(1):T_bounds(2):T_bounds(3)
        for Rx = R_bounds(1):R_bounds(2):R_bounds(3)
            for Ry = R_bounds(1):R_bounds(2):R_bounds(3)
                
                %t = 3.5;
                %Rang = -pi/6;
                
                camera_t = [tx,ty,-scene_depth];
                camera_R = rotateMat(Ry,Rx,0);
                bound_poly = FOVproject(FOV_rads,plane_of_stitching,camera_R,camera_t);
                bound_area = polyarea(bound_poly(:,1),bound_poly(:,2));
                
                if (min_bound == -1) || (bound_area < min_bound)
                    min_bound = bound_area;
                end
                if (max_bound < bound_area)
                    max_bound = bound_area;
                end
            end
        end
    end
end
[min_bound, max_bound]
thresh_bound = num_cameras*max_bound - (num_cameras - 1)*thresh

%% Naive Approach
cc = 1;
for tx = T_bounds(1):T_bounds(2):T_bounds(3)
    for ty = T_bounds(1):T_bounds(2):T_bounds(3)
        
        %t = 3.5;
        %Rang = -pi/6;
        
        camera_t(2,:) = [tx,ty,-scene_depth];
        camera_t(3,:) = [tx,ty,-scene_depth];
        camera_t(4,:) = [tx,ty,-scene_depth];
        camera_t(5,:) = [tx,ty,-scene_depth];
        camera_t(1,:) = [tx,ty,-scene_depth];
        
        %camera_t = [camera1_t;camera2_t;camera3_t;camera4_t;camera5_t];
        
        camera_R(:,:,2) = rotateMat(0,0,-pi/2);
        camera_R(:,:,3) = rotateMat(0,0,0);
        camera_R(:,:,4) = rotateMat(0,0,pi/2);
        camera_R(:,:,5) = rotateMat(0,0,pi);
        camera_R(:,:,1) = rotateMat(0,0,0);
        
        [area,full_poly] = array_area(FOV_rads, camera_R,camera_t,plane_of_stitching,thresh);
        %cc, t, Rang, area,overlap_areas
        naive_output(cc,:) = [cc,area,tx,ty,0];
        naive_drawings{cc} = full_poly;
        cc = cc + 1;
    end
end

naive_sorted_out = sortrows(naive_output,2);
max_id = naive_sorted_out(end,1);
naive_poly = naive_drawings{max_id};

naive_Optimal = naive_sorted_out(end,3:end)
naive_area = naive_sorted_out(end,2)

%% Symmetric approach


cc = 1;
for t = T_bounds(1):T_bounds(2):T_bounds(3)
    for R = R_bounds(1):R_bounds(2):R_bounds(3)
        
        
        %t = 3.5;
        %Rang = -pi/6;
        
        camera_t(2,:) = [-t,0,-scene_depth];
        camera_t(3,:) = [0,t,-scene_depth];
        camera_t(4,:) = [t,0,-scene_depth];
        camera_t(5,:) = [0,-t,-scene_depth];
        camera_t(1,:) = [0,0,-scene_depth];
        
        %camera_t = [camera1_t;camera2_t;camera3_t;camera4_t;camera5_t];
        
        camera_R(:,:,2) = rotateMat(0,-R,0);
        camera_R(:,:,3) = rotateMat(R,0,0);
        camera_R(:,:,4) = rotateMat(0,R,0);
        camera_R(:,:,5) = rotateMat(-R,0,0);
        camera_R(:,:,1) = rotateMat(0,0,0);
        
        [area,full_poly] = array_area(FOV_rads, camera_R,camera_t,plane_of_stitching,thresh);
        %cc, t, Rang, area,overlap_areas
        symmetric_output(cc,:) = [cc,area,t,R*180/pi];
        symmetric_drawings{cc} = full_poly;
        cc = cc + 1;
    end
end

symmetric_sorted_out = sortrows(symmetric_output,2);
max_id = symmetric_sorted_out(end,1);
symmetric_poly = symmetric_drawings{max_id};

symmetric_Optimal = symmetric_sorted_out(end,3:end)
symmetric_area = symmetric_sorted_out(end,2)

%% Greedy Approach

%A better coverage region might work better.
coverage_dim = [15,15]; % Define the region which must be covered by the cameras
coverage_reg = [-coverage_dim(1),-coverage_dim(2);-coverage_dim(1),...
    coverage_dim(2);coverage_dim(1),coverage_dim(1);coverage_dim(1),-coverage_dim(2)];
%coverage_reg = [-20,-10;-20,10;-10,10;-10,20;10,20;10,10;20,10;20,-10;10,-10;10,-20;-10,-20;-10,-10;-20,-10];

n = 5;
%camera_R = repmat(eye(3),[1,1,n]); % initialize camera rotations
%camera_t = repmat([0,0,-16],[n,1]); % initialize camera translations
covered_poly = [0];
camera_pose = [];

greedy_fig = figure;
title('Greedy Camera Placement')
for i = 1:n
    camera_pose;
    %[camera_pose(i,:),covered_poly,cameras] = maxCoverage(FOV_rads,plane_of_stitching, coverage_reg, covered_poly,T_bounds,R_bounds,scene_depth,thresh,camera_pose);
    [camera_pose(i,:),covered_poly,cameras] = greedyBox(FOV_rads,plane_of_stitching, coverage_reg, covered_poly,T_bounds,R_bounds,scene_depth,thresh,camera_pose);
    
    subplot(2,3,i)
    hold on
    fill(coverage_reg(:,1),coverage_reg(:,2),'r');
    fill(covered_poly(:,1),covered_poly(:,2),'b');
    
end
subplot(2,3,6)
fill(covered_poly(:,1),covered_poly(:,2),'b');

print(greedy_fig,'greedy_grid','-dpng')
%figure
%fill(covered_poly(:,1),covered_poly(:,2),'b');
greedy_area = polyarea(covered_poly(:,1),covered_poly(:,2))
greedy_poly = covered_poly;

%% Exhaustive Approach

%Evaluate how long exhaustive will take.
% num_R = floor((R_bounds(3) - R_bounds(1))/R_bounds(2) + 1);
% num_T = floor((T_bounds(3) - T_bounds(1))/T_bounds(2) + 1);
% num_Arms = floor((Rz_bounds(3) - Rz_bounds(1))/Rz_bounds(2) + 1);
% loop_time = 100; % in ms
% 
% runtime = (nchoosek(num_T*num_Arms,num_cameras)*num_R^num_cameras)*loop_time/1000/60/60;
% 
% disp(sprintf('Running exhaustive search will take approximately %f hours to complete',runtime));
% yn =input('Continue with exhaustive search? y/n:','s');
% 
% if (yn ~= 'n')
%     % If we run exhaustive approach, we get here.
%     cc = 0;
%     for tx = T_bounds(1):T_bounds(2):T_bounds(3)
%         for t = T_bounds(1):T_bounds(2):T_bounds(3)
%             cc = cc+1;
%             positionlist(:,cc) = [tx, ty]; % Choose arm, then translation
%         end
%     end
%     
%     camera_poses = combnk(1:cc,num_cameras);
%     cc = 0;
%     for i = 1:length(camera_poses(:,1))
%         for j = 1:num_cameras
%             pose = positionlist(:,camera_poses(i,j));
%             Rz(j) = pose(1);
%             t(j) = pose(2);
%             camera_t(j,:) = [t(j)*sin(Rz(j)),t(j)*cos(Rz(j)),-scene_depth];
%         end
%         for Rx1 = R_bounds(1):R_bounds(2):R_bounds(3)
%             for Rx2 = R_bounds(1):R_bounds(2):R_bounds(3)
%                 for Rx3 = R_bounds(1):R_bounds(2):R_bounds(3)
%                     for Rx4 = R_bounds(1):R_bounds(2):R_bounds(3)
%                         for Rx5 = R_bounds(1):R_bounds(2):R_bounds(3)
%                             for Ry1 = R_bounds(1):R_bounds(2):R_bounds(3)
%                                 for Ry2 = R_bounds(1):R_bounds(2):R_bounds(3)
%                                     for Ry3 = R_bounds(1):R_bounds(2):R_bounds(3)
%                                         for Ry4 = R_bounds(1):R_bounds(2):R_bounds(3)
%                                             for Ry5 = R_bounds(1):R_bounds(2):R_bounds(3)
%                                                 cc = cc + 1;
%                                                 camera_R(:,:,1) = rotateMat(Rx1,Ry1,0);
%                                                 camera_R(:,:,2) = rotateMat(Rx2,Ry2*-sin(Rz(2)),Rz(2));
%                                                 camera_R(:,:,3) = rotateMat(Rx3,Ry3*-sin(Rz(3)),Rz(3));
%                                                 camera_R(:,:,4) = rotateMat(Rx4,Ry4,Rz(4));
%                                                 camera_R(:,:,5) = rotateMat(Rx5,Ry5,Rz(5));
%                                                 
%                                                 [area,full_poly] = array_area(FOV_rads, camera_R,camera_t,plane_of_stitching,thresh);
%                                                 %cc, t, Rang, area,overlap_areas
%                                                 exhaustive_output(cc,:) = [cc,area,t,R1*180/pi,R2*180/pi,R3*180/pi,R4*180/pi,R5*180/pi,Rz];
%                                                 exhaustive_drawings{cc} = full_poly;
%                                                 
%                                                 
%                                             end
%                                         end
%                                     end
%                                 end
%                             end
%                         end
%                         
%                         exhaustive_sorted_out = sortrows(exhaustive_output,2);
%                         max_id = exhaustive_sorted_out(end,1);
%                         exhaustive_poly = exhaustive_drawings{max_id};
%                         
%                         exhaustive_Optimal = exhaustive_sorted_out(end,3:end)
%                         exhaustive_area = exhaustive_sorted_out(end,2)
%                     end
%                     
                    %% Results
                    yn = 'n';
                    if (yn ~= 'n')
                        Areas = [naive_area, symmetric_area, greedy_area, exhaustive_area,thresh_bound]
                    else
                        Areas = [naive_area, symmetric_area, greedy_area,-1,thresh_bound]
                    end
                    res_fig = figure;
                    subplot(2,2,1), fill(naive_poly(:,1),naive_poly(:,2),'r')
                    title('Naive')
                    xlim([-30,30])
                    ylim([-30,30])
                    subplot(2,2,2), fill(symmetric_poly(:,1),symmetric_poly(:,2),'r')
                    title('Symmetric')
                    xlim([-30,30])
                    ylim([-30,30])
                    subplot(2,2,3), fill(greedy_poly(:,1),greedy_poly(:,2),'r')
                    title('Greedy')
                    xlim([-30,30])
                    ylim([-30,30])
                    print(res_fig,'grid_result','-dpng')
                    if (yn ~= 'n')
                        subplot(2,2,4), fill(exhaustive_poly(:,1),exhaustive_poly(:,2),'r')
                        title('Exhaustive')
                        xlim([-20,20])
                        ylim([-20,20])
                    end
                    
                    