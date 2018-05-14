function [area,out_corners] = peelPotato(in_corners)
% A function for approximating the maximal convex shape inside the viewing
% area of our camera array. This solves optimally if the number of reflex
% corners is 0 or 1. It returns the best result using single length chains
% if the number of reflex corners is greater than 1.

reflex = findReflex(in_corners);

if isempty(reflex)
    area = polyarea(in_corners(1,:),in_corners(2,:));
    out_corners = in_corners;
    return;
end

if(length(reflex)  == 1)
    [m,n] = size(in_corners);
    area = -1;
    out_corners = [];
    % construct butterflies.
    for i = 1:n
        if (i ~= reflex)
            C = [];                                     % Reset C
            C2 = [];                                    % Reset C2
            
            % Calculate butterfly chord 1
            [C,Cx,Cy] = maxChord(in_corners,[in_corners(:,i)',in_corners(:,reflex)']);
            
            if ~isempty(C)
                % Find adjacent chord.
                if i == n                                   % Wrap around polygon
                    if (reflex == 1)                        % Skip reflex point
                        i2 = 2;
                    else
                        i2 = 1;
                    end
                elseif (i+1 == reflex)                      % Skip reflex point
                    if (i == n-1)                           % wrap around polygon
                        i2 = 1;
                    else
                        i2 = i+2;
                    end
                else
                    i2 = i+1;
                end
                
                % Calculate butterfly chord 2
                [C2,Cx,Cy] = maxChord(in_corners,[in_corners(:,i2)',in_corners(:,reflex)']);
                
                if ~isempty(C2)
                    % Compute Supporting lines
                    if ((i == n) && (i2 == 2)) || ((i2 == 1) && (i == n-1)) || (i2 == i + 2)
                        line1 = findLine(C(1:2),C2(3:4));
                        line2 = findLine(C(3:4),C2(1:2));
                    else
                        line1 = findLine(C(1:2),C2(1:2));
                        line2 = findLine(C(3:4),C2(3:4));
                    end
                    
                    % Balance Chord
                    bal_cut = balanceChord(line1,line2,in_corners(:,reflex));
                    
                    % Evaluate each cut
                    [area1,corners1] = cutPolygon(in_corners,C);
                    [area2,corners2] = cutPolygon(in_corners,C2);
                    [area3,corners3] = cutPolygon(in_corners,bal_cut);
                    
                    % Update best cut.
                    if (area1 > area)
                        area = area1;
                        out_corners = corners1;
                    end
                    if (area2 > area)
                        area = area2;
                        out_corners = corners2;
                    end
                    if (area3 > area)
                        area = area3;
                        out_corners = corners3;
                    end
                end
            end
        end
    end
    %fill(out_corners(1,:),out_corners(2,:),'r');
    return;
end

if length(reflex > 1)
    
    [m,n] = size(in_corners);
    area = -1;
    out_corners = [];
    temp = in_corners';
    % construct butterflies.
    for j = 1:length(reflex)
        temp_area = -1;
        for i = 1:n
            if (i ~= reflex(j))
                C = [];                                     % Reset C
                C2 = [];                                    % Reset C2
                
                % Calculate butterfly chord 1
                [C,Cx,Cy] = maxChord(in_corners,[in_corners(:,i)',in_corners(:,reflex(j))']);
                
                if ~isempty(C)
                    % Find adjacent chord.
                    if i == n                                   % Wrap around polygon
                        if (reflex(j) == 1)                        % Skip reflex point
                            i2 = 2;
                        else
                            i2 = 1;
                        end
                    elseif (i+1 == reflex(j))                      % Skip reflex point
                        if (i == n-1)                           % wrap around polygon
                            i2 = 1;
                        else
                            i2 = i+2;
                        end
                    else
                        i2 = i+1;
                    end
                    
                    % Calculate butterfly chord 2
                    [C2,Cx,Cy] = maxChord(in_corners,[in_corners(:,i2)',in_corners(:,reflex(j))']);
                    
                    if ~isempty(C2)
                        % Compute Supporting lines
                        if ((i == n) && (i2 == 2)) || ((i2 == 1) && (i == n-1)) || (i2 == i + 2)
                            line1 = findLine(C(1:2),C2(3:4));
                            line2 = findLine(C(3:4),C2(1:2));
                        else
                            line1 = findLine(C(1:2),C2(1:2));
                            line2 = findLine(C(3:4),C2(3:4));
                        end
                        
                        % Balance Chord
                        bal_cut = balanceChord(line1,line2,in_corners(:,reflex(j)));
                        
                        % Evaluate each cut
                        [area1,corners1] = cutPolygon(in_corners,C);
                        [area2,corners2] = cutPolygon(in_corners,C2);
                        [area3,corners3] = cutPolygon(in_corners,bal_cut);
                        
                        % Update best cut.
                        if (area1 > temp_area)
                            temp_area = area1;
                            out_corners = corners1;
                            cut = C;
                        end
                        if (area2 > temp_area)
                            temp_area = area2;
                            out_corners = corners2;
                            cut = C2;
                        end
                        if (area3 > temp_area)
                            temp_area = area3;
                            out_corners = corners3;
                            cut = bal_cut;
                        end
                    end
                end
            end
        end
        
        % Update our estimate for best polygon now that one reflex
        % corner has been cut.
        [a,b,temp] = combinePoly(temp,out_corners','int');
    end
    
    %if isempty(findReflex(temp'))
    out_corners = temp';
    area = polyarea(out_corners(1,:),out_corners(2,:));
    %else
    %    out_corners = convhull(in_corners(:,reflex)');
    %    area = polyarea(out_corners(:,1),out_corners(:,2));
    %end
    
    %fill(out_corners(1,:),out_corners(2,:),'r');
    return;
end
end


%% Functions used by script.
function line = findLine(x1,x2)
m1 = (x2(2) - x1(2));
m2 = (x2(1) - x1(1));
b = m2*x1(2) - m1*x1(1);
line = [m1,m2,b];

end

function [out_chord,Cx,Cy] = maxChord(poly,chord)
% Extends chord so that it is maximal. Note, this function may need to be
% improved to handle the case where there are multiple reflex points. In
% this case, we instead need to ensure that the portion between the stated
% chords lies in the polygon and then extend it to the entire connected
% region that lies in the polygon.

line = findLine(chord(1:2),chord(3:4));                 % Find the line on which the chord lies.
% Extend line so that the ends lie outside the polygon.
if abs(line(1)) < abs(line(2))
    xmax = max(poly(1,:));
    xmin = min(poly(1,:));
    
    out_chord(1) = xmin - 1;
    out_chord(2) = (line(1)*out_chord(1) + line(3))/line(2);
    out_chord(3) = xmax + 1;
    out_chord(4) = (line(1)*out_chord(3) + line(3))/line(2);
    
else
    ymax = max(poly(2,:));
    ymin = min(poly(2,:));
    
    out_chord(2) = ymin - 1;
    out_chord(1) = (line(2)*out_chord(2) - line(3))/line(1);
    out_chord(4) = ymax + 1;
    out_chord(3) = (line(2)*out_chord(4) - line(3))/line(1);
    
end
% Intersect extended chord with the polygon.
[Cx,Cy] = polyxpoly(poly(1,:),poly(2,:),[out_chord(1),out_chord(3)],[out_chord(2),out_chord(4)],'unique');

% Check to see that maximal chord is contained in poly.
flag = zeros(length(Cx) - 1,1);
for i = 1:length(Cx)-1
    mid = ([Cx(i),Cy(i)] + [Cx(i+1),Cy(i+1)])/2;
    flag(i) = inpolygon(mid(1),mid(2),poly(1,:),poly(2,:));
end
% Set maximal chord as output.
if all(flag)
    out_chord(1) = Cx(1);
    out_chord(2) = Cy(1);
    out_chord(3) = Cx(end);
    out_chord(4) = Cy(end);
else
    out_chord = [];
end

end

function [area,corners] = cutPolygon(poly,chord)
% Bisects a polygon via a chord and returns the region of the bisected
% polygon with the largest area. Uses cutpolygon.m script by Dominik Brands
% and Jasper Menger to perform the cutting.

% Placeholders. If function does not succeed, return these.
area = -1;
corners = [];

if any(isnan(chord)) || isempty(chord) || any(isinf(chord))
    return;
end
if all(chord(1:2) == chord(3:4))
    return;
end

if chord(1) ~= chord(3)
    poly1 = cutpolygon(poly',[chord(1:2);chord(3:4)],1)';
    poly2 = cutpolygon(poly',[chord(1:2);chord(3:4)],2)';
else
    poly1 = cutpolygon(poly',[chord(1:2);chord(3:4)],3)';
    poly2 = cutpolygon(poly',[chord(1:2);chord(3:4)],4)';
end

area1 = polyarea(poly1(1,:),poly1(2,:));
area2 = polyarea(poly2(1,:),poly2(2,:));

if area1 > area2
    area = area1;
    corners = poly1;
else
    area = area2;
    corners = poly2;
end



end
