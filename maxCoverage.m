function [camera_pose,covered_poly,camera_views] = maxCoverage(FOV_rads,plane_of_stitching, coverage_reg, covered_poly,T_bounds,R_bounds,scene_depth,thresh,camera_pose)
%   MAXCOVERAGE calculates the calculates the optimal camera pose for a camera to be added to a cross shaped camera array. 

%   Output Variables:
%   camera_pose - The set of camera parameters that define the best camera
%   covered_poly - The total field of view of the camera with 
%   camera_views - The set of polygons denoting the coverage region for each camera in the array, including the new one. 
%
%   Input Variables:
%   FOV_rads -  A 1x2 array giving the maximum angle visible from the cameras along the x axis or y axis.
%   plane_of_stitching - The coefficients defining the plane where the scene lies. This should be in the format 
%   [A,B,C,D] where A,B,C, and D fulfill the plane equation Ax + By + Cz + D = 0.
%   coverage_reg -
%   covered_poly - The polygon detailing what region the camera array can currently see. The union of all previous camera FoV 
%   T_bounds - T_bounds[1] holds the minimum arm length for the cross shape, T_bounds[2] holds the increment size for the 
%   discrete camera placement, and T_bounds[3] holds the maximum arm length.  
%   R_bounds - R_bounds[1] holds the minimum angle of rotation for the camera, R_bounds[2] holds the increment size for the 
%   discrete camera placement, and R_bounds[3] holds the maximum angle of rotation. 
%   scene_depth - A scalar denoting the distance from the camera array to the scene along the axis perpendicular to the spread of the array. 
%   camera_pose - The set of camera poses for cameras already placed in the array. 
%T_bounds = [2,.05,7.5];
%scene_depth = 16.5;
%thresh = 100;
[n,m] = size(camera_pose);
for k = 1:n
    camera_R = rotateMat(camera_pose(k,1)*cos(camera_pose(k,2)),camera_pose(k,1)*-sin(camera_pose(k,2)),camera_pose(k,2));
    camera_t = [camera_pose(k,3)*sin(camera_pose(k,2)),camera_pose(k,3)*cos(camera_pose(k,2)),scene_depth];
    camera_views{k} = FOVproject(FOV_rads,plane_of_stitching,camera_R,camera_t);
end

if (length(covered_poly) > 1)
    current_area = polyarea(covered_poly(:,1),covered_poly(:,2));
else
    current_area = 0;
end

comp_array = [];
cc = 1;
for Rang = R_bounds(1):R_bounds(2):R_bounds(3)
    for R_z = [-pi/2,0 pi/2,pi]
        for t_xy = T_bounds(1):T_bounds(2):T_bounds(3)
            %check to ensure we don't place camera in same place.
            flag = 0;
            for k = 1:n
                if (camera_pose(k,2) == R_z) && (camera_pose(k,3) == t_xy)
                    flag = 1;
                end
            end
            if (flag == 0)
                %generate the camera, and find it's projection
                camera_R = rotateMat(Rang*cos(R_z),Rang*-sin(R_z),R_z);
                camera_t = [t_xy*sin(R_z),t_xy*cos(R_z),scene_depth];
                new_poly{cc} = FOVproject(FOV_rads,plane_of_stitching,camera_R,camera_t);
                new_poly{cc} = new_poly{cc}(:,1:2);
                new_cam_area = polyarea(new_poly{cc}(:,1),new_poly{cc}(:,2));
                
                
                %add camera to existing views
                if (length(covered_poly) > 1)
                    [flag,combined_area, combined_poly{cc}] = combinePoly(covered_poly,new_poly{cc},'uni');
                    %[flag, overlap_area, overlap_poly{cc}] = combinePoly(new_poly, covered_poly,'int');
                    overlap_area = 0;
                    for i = 1:n
                        [flag, overlap_area(i),temp] = combinePoly(new_poly{cc}, camera_views{i},'int');
                        
                    end
                    if (max(overlap_area) - thresh > 0)
                        [temp_clip_x,temp_clip_y] = polyclip(coverage_reg(:,1),coverage_reg(:,2),combined_poly{cc}(:,1),combined_poly{cc}(:,2),'int');
                        
                        if ~isempty(temp_clip_x)
                            new_covered = [temp_clip_x{1},temp_clip_y{1}];
                            covered_area = polyarea(new_covered(:,1),new_covered(:,2));
                        else
                            new_covered = 0;
                            covered_area = 0;
                            %combined_area = 0;
                        end
                    else
                        covered_area = 0;
                        combined_area = 0;
                    end
                else
                    combined_poly{cc} = new_poly{cc};
                    combined_area = new_cam_area;
                    [temp_clip_x,temp_clip_y] = polyclip(coverage_reg(:,1),coverage_reg(:,2),new_poly{cc}(:,1),new_poly{cc}(:,2),'int');
                    
                    if ~isempty(temp_clip_x)
                        
                        new_covered = [temp_clip_x{1},temp_clip_y{1}];
                        %new_covered{cc}
                        covered_area = polyarea(new_covered(:,1),new_covered(:,2));
                    else
                        new_covered = [0];
                        covered_area = 0;
                    end
                end
                
                comp_array(cc,:) = [cc,covered_area,combined_area,Rang,R_z,t_xy];
                cc = cc + 1;
            end
        end
    end
end




sorted_out = sortrows(comp_array,[2,3]);
camera_pose = sorted_out(end,4:end);
covered_poly = combined_poly{sorted_out(end,1)};
camera_views{n+1} = new_poly{sorted_out(end,1)};
