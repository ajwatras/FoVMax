function drawChain(line1,line2,R,Cx)
minx = min(R(1,:));
maxx = max(R(1,:));
miny = min(R(2,:));
maxy = max(R(2,:));

x = linspace(minx-5,maxx+5);
y = linspace(miny-5,maxy+5);

figure(1)
hold on
scatter(R(1,:),R(2,:))
%xlim([minx - 1,maxx+1])
%ylim([miny - 1,maxy+1])

if line1(2) == 0
    L0 = repmat(-line1(3)/line1(1),[1,length(y)]);
    plot(L0,y);
else
    L0 = (line1(1)*x + line1(3))/line1(2);
    plot(x,L0,'r')
end

if line2(2) == 0
    L1 = repmat(-line2(3)/line2(1),[1,length(y)]);
    plot(L1,y);
else
    L1 = (line2(1)*x + line2(3))/line2(2);
    plot(x,L1,'b')
end

plot(Cx([1,3]),Cx([2,4]),'k');