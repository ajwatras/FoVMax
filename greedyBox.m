function [camera_pose,covered_poly,camera_views] = greedyBox(FOV_rads,plane_of_stitching, coverage_reg, covered_poly,T_bounds,R_bounds,scene_depth,thresh,camera_pose)
%R_bounds = [-pi/3,.05,pi/3];
%T_bounds = [2,.05,7.5];
%scene_depth = 16.5;
%thresh = 100;
[n,m] = size(camera_pose);
for k = 1:n
    camera_R = rotateMat(camera_pose(k,1),camera_pose(k,2),0);
    camera_t = [camera_pose(k,3),camera_pose(k,4),scene_depth];
    camera_views{k} = FOVproject(FOV_rads,plane_of_stitching,camera_R,camera_t);
end

if (length(covered_poly) > 1)
    current_area = polyarea(covered_poly(:,1),covered_poly(:,2));
else
    current_area = 0;
end

comp_array = [];
cc = 1;
for Rx = R_bounds(1):R_bounds(2):R_bounds(3)
    for Ry = R_bounds(1):R_bounds(2):R_bounds(3)
        for tx = T_bounds(1):T_bounds(2):T_bounds(3)
            for ty = T_bounds(1):T_bounds(2):T_bounds(3)
                %check to ensure we don't place camera in same place.
                flag = 0;
                for k = 1:n
                    if (camera_pose(k,3) == tx) && (camera_pose(k,4) == ty)
                        flag = 1;
                    end
                end
                if (flag == 0)
                    %generate the camera, and find it's projection
                    camera_R = rotateMat(Rx, Ry, 0);
                    camera_t = [tx,ty,scene_depth];
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
                    
                    comp_array(cc,:) = [cc,covered_area,combined_area,Rx,Ry,tx,ty];
                    cc = cc + 1;
                end
            end
        end
    end
end



cc
sorted_out = sortrows(comp_array,[2,3]);
camera_pose = sorted_out(end,4:end);
covered_poly = combined_poly{sorted_out(end,1)};
camera_views{n+1} = new_poly{sorted_out(end,1)};
