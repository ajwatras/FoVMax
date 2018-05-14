octangle = [0,0;0,2;2,4;4,4;6,2;5,1;6,0;4,-2;2,-2;0,0]';
square2 = [0,0;0,2;1,1;2,2,;2,0]';
potato = [-1.5,-1,-2,-2,-1,-1.5,1.5,1,2,2,1,1.5;-2,-1,-1.5,1.5,1,2,2,1,1.5,-1.5,-1,-2];

for i = 1:100
    tic()
    maxRectangle(octangle);
    t(i) = toc();
end
ave_oct = mean(t);

for i = 1:100
    tic()
    maxRectangle(square2);
    t(i) = toc();
end
ave_squ = mean(t);

for i = 1:100
    tic()
    maxRectangle(potato);
    t(i) = toc();
end
ave_pot = mean(t(1:100));

ave_oct
ave_squ
ave_pot