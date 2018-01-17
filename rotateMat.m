function [R] = rotateMat(xang,yang,zang)
% ROTATEMAT generates a rotation matrix

%	Output Variables:
%	R - The resulting 3x3 rotation matrix. 
%	Input Variables:
%	xang - The magnitude of rotation around the x axis in radians.
%	yang - The magnitude of rotation around the y axis in radians.
%	zang - The magnitude of rotation around the z axis in radians.

Rx = [1 0 0; 0 cos(xang) -sin(xang); 0 sin(xang) cos(xang)];
Ry = [cos(yang) 0 sin(yang); 0 1 0; -sin(yang) 0 cos(yang)];
Rz = [cos(zang) -sin(zang) 0; sin(zang) cos(zang) 0; 0 0 1];

R = Rx*Ry*Rz;