function [success,area, poly] = combinePoly(p1,p2,method)

[a,b] = polyclip(p1(:,1),p1(:,2),p2(:,1),p2(:,2),method);

if length(a) == 1
    area = polyarea(a{1},b{1});
    poly = [a{1},b{1}];
    success = 1;
else
    area = 0;
    poly = p1;
    success = 0;
end

end