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
    
    if isempty(findReflex(temp'))
        out_corners = temp';
        area = polyarea(out_corners(1,:),out_corners(2,:));
    else
        if length(reflex) > 2
            K = convhull(in_corners(:,reflex)');
            out_corners = in_corners(:,reflex)';
            out_corners = out_corners(K,:);
            [a,b,out_corners] = combinePoly(out_corners,in_corners','int');
            if ~isempty(out_corners) && isempty(findReflex(out_corners'))
                area = polyarea(out_corners(:,1),out_corners(:,2));
            else
                out_corners = [];
                area = -1;
            end
        else
            out_corners = [];
            area = -1;
        end
    end
    
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
if all(flag) && ~isempty(flag)
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

function out_vals = balanceChord(line1,line2,pivot)
% Finds the chord through pivot such that the end points lie on line1 and
% line2 and the chord is balanced.
% line1 = [m1, c1]
% line2 = [m2, c2]
% pivot = [px,py;qx,qy]'
% out_vals has format [x_0,y_0,x_1,y_1]
% Could change output format to [m,c0]
out_vals = [];
[n,pivot_size] = size(pivot);

% need to develop for double pivot chord
if (pivot_size == 2)
    if ~all(pivot(:,1) == pivot(:,2))
        
        % Determine chord passing through pivots
        m1 = (pivot(2,1) - pivot(2,2));
        m2 = (pivot(1,1) - pivot(1,2));
        b = m2*pivot(2,2) - m1*pivot(1,2);
        line = [m1,m2,b];
        
        % Calculate intercepts with each line.
        x0 = [line(1), -line(2);line1(1), -line1(2)]\[-line(3);-line1(3)];
        x1 = [line(1), -line(2);line2(1), -line2(2)]\[-line(3);-line2(3)];
        
        
        % Calculate line midpoint and determine which line is nearest each
        % pivot
        ref = (x0 + x1)/2;
        if norm(pivot(:,1) - x0,2) < norm(pivot(:,2) - x0,2)
            p = pivot(:,1);
            q = pivot(:,2);
        else
            q = pivot(:,1);
            p = pivot(:,2);
        end
        
        %Check if ref is in between the pivots
        if (norm(ref - x0,2) >= norm(p - x0,2)) && (norm(ref - x1,2) >= norm(p - x0,2))
            % Check for bracketing property
            if norm( 2*p - x0 - x1,2) <= norm(2*(q - x1))
                out_vals = [x0;x1]';
                return;
            end
            
            
        end
    else
        pivot = pivot(:,1);
    end
    
end

% works if we have a single pivot chord and our lines are not parallel.
if (pivot_size == 1)
    % Adjust coordinate system so pivot is (0,0)
    c1 = line1(3) - line1(2)*pivot(2) + line1(1)*pivot(1);
    c2 = line2(3) - line2(2)*pivot(2) + line2(1)*pivot(1);
    
    % Assume pivot = (0,0)
    A = [line1(1), -line1(2);-line2(1), line2(2)];
    b = [-c1;-c2];
    if det(A) == 0
        out_vals = [];
        return;
    end
    if abs(det(A)) < 0.00001
        out_vals = [];
        return
    else
        
        x0 = A\b;
        % d = norm(x0,2)                        % The distance, use for checking balance.
        
        out_vals = [x0 + pivot; pivot - x0]';
    end
end

end

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

sang = sign(angle) < 0;
reflex = find(sang);
end

