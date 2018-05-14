function [Area, FOV] = FOV_eval(Positions, scene_depth,FOV_rads,plane_of_stitching,thresh)
% Evaluate the Field of view of a given camera array.
Area = -1;
FoV = [0;0];
[n_cams,n_params] = size(Positions);

% Check to ensure Positions uses proper format. 
if n_params == 3
    % Generate the camera translation and Rotation matrix for each camera.
    for j = 1:n_cams
        camera_t(j,:) = [Positions(j,2)*sin(Positions(j,3)),Positions(j,2)*cos(Positions(j,3)),-scene_depth];
        camera_R(:,:,j) = rotateMat(Positions(j,1)*cos(Positions(j,3)),Positions(j,1)*-sin(Positions(j,3)),Positions(j,3));
    end
    
    % Evaluate the array area. 
    [Area,FOV] = array_area(FOV_rads, camera_R,camera_t,plane_of_stitching,thresh);
end
