function [output] = FOVcone(FOV_rads,R,t,scale)
%	FOVCONE calculates the bounding rays for the viewable region given the camera FoV and pose. 
% 	
%	Output Variables:
%	output - The four resulting rays combined into a matrix. 

%	Input Variables:
%	FoV_rads - A 1x2 array giving the maximum angle visible from the cameras along the x axis or y axis.
%	R - A 3x3 camera rotation matrix
% 	t - A 3x1 camera translation vector
%	scale - A parameter used to change the length of each of the rays. 

direction = sin(FOV_rads./2);

ray1 = scale.*R*[direction(1),direction(2),1]';
ray2 = scale.*R*[-direction(1),direction(2),1]';
ray3 = scale.*R*[-direction(1),-direction(2),1]';
ray4 = scale.*R*[direction(1),-direction(2),1]';

output = [ray1,ray2,ray3,ray4];