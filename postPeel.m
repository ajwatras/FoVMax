%% Load the prior workspace
load tempspace.mat

for i = 1:length(exhaustive_drawings)
    [area(i),out_corners{i}] = peelPotato(exhaustive_drawings{i}');
end
[con_area, sortIdx] = sort(area);
exaustive_con_area = exhaustive_sorted_out(sortIdx(end),2);
exhaustive_con = out_corners{sortIdx(end)}';
exhaustive_con_poly = exhaustive_drawings{sortIdx(end)};

save post.mat

%% Display outputs
if (yn ~= 'n')
    Areas = [naive_area, symmetric_area, greedy_area, exhaustive_area,thresh_bound]
    con_Areas = [naive_con_area,symmetric_con_area,greedy_area,exhaustive_con_area,thresh_bound]
else
    Areas = [naive_area, symmetric_area, greedy_area,-1,thresh_bound]
    con_Areas = [naive_con_area, symmetric_con_area, greedy_area,-1,thresh_bound]
end
res_fig = figure;
subplot(2,2,1), fill(naive_poly(:,1),naive_poly(:,2),'r')
title('Naive')
xlim([-100,100])
ylim([-100,100])
subplot(2,2,2), fill(symmetric_poly(:,1),symmetric_poly(:,2),'r')
title('Symmetric')
xlim([-100,100])
ylim([-100,100])
subplot(2,2,3), fill(greedy_poly(:,2),-greedy_poly(:,1),'r')
title('Greedy')
xlim([-100,100])
ylim([-100,100])
if (yn ~= 'n')
    subplot(2,2,4), fill(exhaustive_poly(:,1),exhaustive_poly(:,2),'r')
    title('Exhaustive')
    xlim([-100,100])
    ylim([-100,100])
end
print(res_fig,'comp_result','-dpng')

% Convex Results
res_fig_con = figure;

subplot(2,2,1), hold on, fill(naive_con_poly(:,1),naive_con_poly(:,2),'b'), fill(naive_con(:,1),naive_con(:,2),'r')
title('Naive')
xlim([-100,100])
ylim([-100,100])
subplot(2,2,2), hold on, fill(symmetric_con_poly(:,1),symmetric_con_poly(:,2),'b'), fill(symmetric_con(:,1),symmetric_con(:,2),'r')
title('Symmetric')
xlim([-100,100])
ylim([-100,100])
subplot(2,2,3), fill(greedy_poly(:,2),-greedy_poly(:,1),'b')
title('Greedy')
xlim([-100,100])
ylim([-100,100])
if (yn ~= 'n')
    subplot(2,2,4),hold on, fill(exhaustive_con_poly(:,1),exhaustive_con_poly(:,2),'b'), fill(exhaustive_con(:,1),exhaustive_con(:,2),'r')
    title('Exhaustive')
    xlim([-100,100])
    ylim([-100,100])
end
print(res_fig_con,'comp_result_con','-dpng')