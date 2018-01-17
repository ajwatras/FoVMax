function [ error_flag, point ]= ray_plane_intersect(plane_normal,ray_normal,ray_translation)
%	RAY_PLANE_INTERSECT computes the intersection point of a ray and a plane. 

%	Output Variables:
%	error_flag - Denotes whether the intersection was possible. 
%	point - The resulting point of intersection. 

%	Input Variables:
%	plane_normal -	The vector defining the plane. 
%	ray_normal - The vector defining the direction of the ray.
%	ray_translate - The vector defining the translation applied to the ray. 

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

