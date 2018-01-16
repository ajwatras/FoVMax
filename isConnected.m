function flag = isConnected(A)
%A should be a square adjacency matrix. 
[m,n] = size(A);
if (m ~= n)
    flag = 0;
    return;
end

C = zeros([m,n]);
for i = 0:n
    C =  C + A^i;
end

%A is connected iff C has no zero entries.
flag = not(any(any(C == 0)));