function [corners] = FOVproject(FOV_rads,plane_of_stitching,camera_R,camera_t)

rays = FOVcone(FOV_rads,camera_R,camera_t,1);

[ret1, point1] = ray_plane_intersect(plane_of_stitching,rays(:,1)',camera_t);
[ret2, point2] = ray_plane_intersect(plane_of_stitching,rays(:,2)',camera_t);
[ret3, point3] = ray_plane_intersect(plane_of_stitching,rays(:,3)',camera_t);
[ret4, point4] = ray_plane_intersect(plane_of_stitching,rays(:,4)',camera_t);

corners = [point1;point2;point3;point4];

