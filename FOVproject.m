function [corners] = FOVproject(FOV_rads,plane_of_stitching,camera_R,camera_t)
%	FOVPROJECT calculates the projection of the FoV cone onto the plane of 
%	stitching and returns the resulting polygon vertices.
%
%	Output Variables:
%	corners -
%
%	Input Variables:
% 	FOV_rads -  A 1x2 array giving the maximum angle visible from the cameras along the x axis or y axis.
%	plane_of_stitching - The coefficients defining the plane where the scene lies. This should be in the format 
%  [A,B,C,D] where A,B,C, and D fulfill the plane equation Ax + By + Cz + D = 0.
%	camera_R - A 3x3 Rotation Matrix
%	camera_t - A 3x1 vector specifying the camera translation. 


% Calculate the FoV cone
rays = FOVcone(FOV_rads,camera_R,camera_t,1);

% Intersect each ray with the plane of stitching. 
[ret1, point1] = ray_plane_intersect(plane_of_stitching,rays(:,1)',camera_t);
[ret2, point2] = ray_plane_intersect(plane_of_stitching,rays(:,2)',camera_t);
[ret3, point3] = ray_plane_intersect(plane_of_stitching,rays(:,3)',camera_t);
[ret4, point4] = ray_plane_intersect(plane_of_stitching,rays(:,4)',camera_t);

% Format output
corners = [point1;point2;point3;point4];

