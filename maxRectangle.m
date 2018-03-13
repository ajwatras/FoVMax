function [out_corners, area] = maxRectangle(in_corners)
% A function for approximating the maximal convex shape inside the viewing
% area of our camera array. This checks several cases related to the cross
% shape of the surgical array

reflex = findReflex(in_corners);

if(length(reflex)  == 1)
    % construct butterfly
    
end
% case 1
d1 = norm(in_corners(:,reflex(1)) - in_corners(:,reflex(2)));
d2 = norm(in_corners(:,reflex(3)) - in_corners(:,reflex(4)));
dy = min([d1,d2]);
dx = max(in_corners(1,:)) - min(in_corners(1,:));
area = dy*dx;


% case 2
d1 = norm(in_corners(:,reflex(1)) - in_corners(:,reflex(4)));
d2 = norm(in_corners(:,reflex(2)) - in_corners(:,reflex(3)));
dy = min([d1,d2]);
dx = max(in_corners(2,:)) - min(in_corners(2,:));
if (area < dy*dx)
    area = dy*dx;
end

% case 3


[m,n] = size(in_corners);
out_corners = [];