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
    x0 = A\b;
    % d = norm(x0,2)                        % The distance, use for checking balance.
    
    out_vals = [x0 + pivot; pivot - x0]';
end

end
