function [R] = rotateMat(xang,yang,zang)

Rx = [1 0 0; 0 cos(xang) -sin(xang); 0 sin(xang) cos(xang)];
Ry = [cos(yang) 0 sin(yang); 0 1 0; -sin(yang) 0 cos(yang)];
Rz = [cos(zang) -sin(zang) 0; sin(zang) cos(zang) 0; 0 0 1];

R = Rx*Ry*Rz;