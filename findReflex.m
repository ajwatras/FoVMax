function reflex = findReflex(poly)
% Finds the reflex corners of the polygon poly
[m,n] = size(poly);

for i = 2:n-1
    v1 = poly(:,i) - poly(:,i-1);
    v2 = poly(:,i+1) - poly(:,i);
    angle(i) = det([v1,v2]);
end
v1 = poly(:,n) - poly(:,n-1);
v2 = poly(:,1) - poly(:,n);
angle(n) = det([v1,v2]);

v1 = poly(:,1) - poly(:,n);
v2 = poly(:,2) - poly(:,1);
angle(1) = det([v1,v2]);

sang = sign(angle) > 0;
reflex = find(sang);


