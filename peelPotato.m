function  [out_corners, area] = peelPotato(potato)
% implements the potato peeling algorithm from J.S. Chang's 1986 paper A
% polynomial solution for the potato peeling problem.

%Tunable Params
DISPLAY_CHORDS = 1;
DISPLAY_CHAINS = 1;
%% Set Up
if isempty(potato)
    %potato = [-1.5,-1,-2,-2,-1,-1.5,1.5,1,2,2,1,1.5;,-2,-1,-1.5,1.5,1,2,2,1,1.5,-1.5,-1,-2];
     %potato = [0,1,.5,0,-.5,-1;1,0,0,.5,0,0];
     potato = [0,0;0,2;2,4;3,3;4,4;6,2;5,1;6,0;4,-2;2,-2]'

end
% Determine the number of vertices in the potato
[m,n] = size(potato);

% Determine the set of reflex corners
reflex = findReflex(potato);

% Determine the total set of extremal chords from the potato.
for i = 1:n
    for j = 1:n
        [Cx,Cy] = polyxpoly(potato(1,:),potato(2,:),potato(1,[i,j]),potato(2,[i,j]));
        mid = (potato(:,i) + potato(:,j))/2;
        if (length(Cx) == 2) && inpolygon(mid(1,:),mid(2,:),potato(1,:),potato(2,:))
            xi(j+n*(i-1),:) = [Cx(1),Cy(1),Cx(2),Cy(2)];
        end
    end
end

if DISPLAY_CHORDS
    figure(1)
    hold on
        fill([potato(1,:),potato(1,1)],[potato(2,:),potato(2,1)],'-p')
    for i=1:length(xi(:,1))
        plot(xi(i,[1,3]),xi(i,[2,4]),'k')
    end
end


%% Step 1
% Calculate A.

% Set initial size and values of A.
A = -1*ones([length(xi),length(xi)]);

if DISPLAY_CHAINS
    figure(2)
    hold on
    fill([potato(1,:),potato(1,1)],[potato(2,:),potato(2,1)],'-p')
end
% Stage 1 Calculate all length 0 or 1 chains for each pair
for j = 1:length(xi)
    for k = 1:length(xi)
        C = xi(j,:);
        C2 = xi(k,:);
        % If C and C2 are valid chords.
        if any(C) && any(C2)
            % Generate set of R as corners moving clockwise from C to C2.
            vj = mod(j,n);
            if (vj == 0), vj = n; end
            vk = ceil((k)/n);
            if vk >= vj
                R = potato(:,vj:vk);
            else
               R = [potato(:,vk:end),potato(:,1:vj)];
            end
            % Generate the chain of length 1 for this set of points and chords.
            [flag,chain] = genChain(C,C2,R,1);
            
            % Store the chain.
            if flag && all(inpolygon(chain(1,:),chain(2,:),potato(1,:),potato(2,:)))
                
                chains{j,k,1} = chain;
                if DISPLAY_CHAINS
                    figure(2)
                    chain
                    plot(chain(1,:),chain(2,:),'k')
                end
            end
        end
    end
end
%Stage 2

% Stage 2:m
for i = 2:n
    for j = 1:length(xi)
        for k = j:length(xi)
            for l = 1:length(xi)
                % Try to combine previously calculated chains.
                if ~(j == l || k == l)
                    for m = 1:i-1
                        m2 = i-m-1;
                        if (m2 > 0)
                            chain1 = chains{i,l,m};
                            chain2 = chains{l,k,m2};
                            
                            if ~(isempty(chain1) || isempty(chain2))
                                p1 = findPivots(chain1);
                                p2 = findPivots(chain2);
                                % Set p and q as elements of R defining l.
                                p = xi(l,1:2);
                                q = xi(l,3:4);
                                
                                % Check for possibility A
                                
                                %Compute combined pivots
                                p_total(1:m,:) = p1;
                                p_total(m+2:m+2+m2,:) = p2;
                                if (mod(m2,2) == 0 )
                                    p_total(m,:) = p;
                                else
                                    p_total(m,:) = q;
                                end
                                
                                % Extend chain1
                                chain1(:,m+1) = 2*p_total(m+1) - chain(:,m);
                                
                                % Find critical point for the length i chain.
                                
                                % Check that the exteded chain1 and chain2 lie
                                % on the same critical interval.
                                
                                % Check for Possibility B
                            end
                        end
                    end
                end
            end
        end
        
    end
end

%First iterate over all C and C' in xi.
for j = 1:length(xi)
    for k = 1:length(xi)
        C = xi(j,:);
        C2 = xi(k,:);
        
        % Generate matrix A(C,C') which is the area of the uniquely balanced (C,C')
        % chain for all C and C' in xi.
        
        % Generate chain
        chain = genChain(C,C2,R,2);
        
        % Check for admissability
        if chainAdmissable(chain)
            % Calculate area
        else
            %Do not set area.
        end
        
        % Generate matrix A(C,C') which is the area of the uniquely balanced (C,C')
        % chain for all C and C' in xi.
        
    end
end


%% Step 2
% Calculate M.

% Set initial values of M
M = -1*ones(length(xi),length(xi));

% For C in xi_i, C' in xi_j, C'' in xi_k, we find
% max(A(C,C'), max_C''(M(C,C'')+M(C'',C)+Area(triangle(c,c'',c'))
for i = 1:n
    for j = i+1:n
        for k = j+1:n
            %identify xi_a ranges
            
            for i2 = 1:n
                for j2 = 1:n
                    for k2 = 1:n
                        %identify chord indexes
                        id0 = n*i+i2; % C
                        id1 = n*j+j2; % C'
                        id2 = n*k+k2; % C''
                        
                        %Choose chords for C, C', C''
                        C = xi(n*i+i2,:);
                        C1 = xi(n*j+j2,:);
                        C3 = xi(n*k+k2,:);
                        
                        % Set up values for M(C,C')
                        tmpM = -1;  % tmpM = max_C''(M(C,C'')+M(C'',C)+Area(triangle(c,c'',c'))
                        
                        M(id0,id1) = max(A(id0,id(1)),tmpM);    % max(A(C,C'), max_C''(M(C,C'')+M(C'',C)+Area(triangle(c,c'',c'))
                        
                    end
                end
            end
        end
    end
end


%% Step 3
% Calculate the resulting area.

% Find the best chain.
[max_val,j] = max(M+M');
[max_val,i] = max(max_val);
j = j(i);


% Using the chord indexes i and j, find the polygon associated with the
% balanced chains (C1,C2),(C2,C1).

%% results
area = max_val;
out_corners = xi;

