function pivots = findPivots(chain)
%given the end points in a chain. returns the midpoints of each line. 

[m,n] = size(chain);

for i = 1:n-1
    pivots(:,i) = (chain(:,i+1) + chain(:,i))/2;
end
