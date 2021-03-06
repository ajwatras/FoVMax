function [flag,chain] = genChain(C1,C2,R,len)
% Finds the unique balanced chain from chord C1 to C2.
% Tunable Parameters
DISPLAY_FLAG = 0;

flag = 0;
chain = 0;
[m,n] = size(R);

% convert chords to lines (format = [m,b], mx + b = y)
m1 = (C1(4) - C1(2));
m2 = (C1(3) - C1(1));
b = m2*C1(2) - m1*C1(1);
line1 = [m1,m2,b];

m1 = (C2(4) - C2(2));
m2 = (C2(3) - C2(1));
b = m2*C2(2) - m1*C2(1);
line2 = [m1,m2,b];

% For Chains of length 1 we can solve
if (len == 1)
    % Check for parallel lines.
    if (line1(1)/line1(2)) == (line2(1)/line2(2))
        % May be able to solve in this case.
        
        return;
    %end
    else
    % find intersection point of line1 and line 2.
    isect = [line1(1), -line1(2);line2(1), -line2(2)]\[-line1(3);-line2(3)];
    end
    % find all double pivot balances
    for i = 1:n-1
        for j = i+1:n

            c_out = balanceChord(line1,line2,R(:,[i,j]));
            % check if all points enclosed
            if ~isempty(c_out)
                tri_reg = [c_out(1),isect(1),c_out(3);c_out(2),isect(2),c_out(4)];
                % if all points enclosed, generate chain and return
                if all(inpolygon(R(1,:),R(2,:),tri_reg(1,:),tri_reg(2,:)))
                    flag = 1;
                    chain = [c_out(1),c_out(2);c_out(3),c_out(4)]';
                    if DISPLAY_FLAG
                        drawChain(line1,line2,R,c_out);
                    end
                    return;
                end
            end
        end
    end
    
    % find all single pivot balances
    for i = 1:n
        c_out = balanceChord(line1,line2,R(:,i));
        % check if all points enclosed
        tri_reg = [c_out(1),isect(1),c_out(3);c_out(2),isect(2),c_out(4)];
        % If all points enclosed, generate chain and return.
        
        if all(inpolygon(R(1,:),R(2,:),tri_reg(1,:),tri_reg(2,:)))
            flag = 1;
            chain = [c_out(1),c_out(2);c_out(3),c_out(4)]';
            
            if DISPLAY_FLAG
                drawChain(line2,line2,R,c_out);
            end
            
            return;
        end
        
    end
    
    
end
if (len > 1)

end


