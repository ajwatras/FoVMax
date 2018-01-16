function [area,full_poly] = array_area(FOV_rads, camera_R,camera_t,plane_of_stitching,thresh)
%thresh = 50;
n = length(camera_t(:,1));

%poly{1} = FOVproject(FOV_rads,plane_of_stitching,camera_R(:,:,1),camera_t(1,:));
%A{1} = polyarea(poly{1}(:,1),poly{1}(:,2));
%ind = convhull(poly{1}(:,1:2)); %assumes polygon is parallel to the xy plane.
%poly{1} = poly{1}(ind,:);

%figure
%fill(full_poly(:,1),full_poly(:,2),'r')
for k=1:n
    poly{k} = FOVproject(FOV_rads,plane_of_stitching,camera_R(:,:,k),camera_t(k,:));
    ind = convhull(poly{k}(:,1:2)); % assumes polygon is parallel to xy plane.
    poly{k} = poly{k}(ind,:);
    
    %figure
    %fill(full_poly(:,1),full_poly(:,2),'r')
    %fill(poly{k}(:,1),poly{k}(:,2),'r')
end

overlap_area = zeros(n,n);
for i = 1:(n-1)
    for j = i+1:n
        [flag,overlap_area(i,j),temp] = combinePoly(poly{i},poly{j},'int');
        %overlap_area(i,j) = max(overlap_area(i,j) - thresh,0);
        overlap_area(j,i) = overlap_area(i,j);
    end
end
%overlap_area
[sorted,idx] = sort(sum(overlap_area > 0),'descend');
for i = 1:n
    temp_poly{i} = poly{idx(i)};
end 
poly = temp_poly;
%idx
[flag,area, full_poly] = combinePoly(poly{1},poly{1},'uni');
successes = zeros([1,n]);
for i = 1:n
    for j = 1:n
        % Find union with existing FOV
        if (area > 0) || (i ~= n)
            [flag, hold_area, temp_poly] = combinePoly(full_poly, poly{j},'uni');
            if flag
                area = hold_area;
                full_poly = temp_poly;
                successes(j) = 1;
                
                %figure
                %fill(full_poly(:,1),full_poly(:,2),'r')
                
            end
        end
    end
end

if (prod(successes) == 0)
    area = 0;
    return;
end

%for i = 1:n
%    if ((max(overlap_area(:,i)) - thresh) < 0)
%        area = 0; 
%    end
%end
if ~isConnected((overlap_area - thresh) > 0)
    area = 0;
end
end
