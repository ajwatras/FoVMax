function [output] = FOVcone(FOV_rads,R,t,scale)
%	FOVCONE calculates the bounding rays for the viewable region given the camera FoV and pose. 
% 	
%	Output Variables:
%	output - The four resulting rays combined into a matrix. 

%	Input Variables:
%	FoV_rads -
%	R - 
% 	t -
%	scale - 

direction = sin(FOV_rads./2);

ray1 = scale.*R*[direction(1),direction(2),1]';
ray2 = scale.*R*[-direction(1),direction(2),1]';
ray3 = scale.*R*[-direction(1),-direction(2),1]';
ray4 = scale.*R*[direction(1),-direction(2),1]';

output = [ray1,ray2,ray3,ray4];