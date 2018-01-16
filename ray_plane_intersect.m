function [ error_flag, point ]= ray_plane_intersect(plane_normal,ray_normal,ray_translation)
% Intersects a plane and a ray. 

numerator = [ray_translation,1]*plane_normal;
denominator = [ray_normal,0]*plane_normal;

if (denominator == 0)
    error_flag = 1;
    point = 0;
else
    error_flag = 0;
    t = -numerator./denominator;
    point = ray_translation + t*ray_normal;
    point = point(1:3);
end

