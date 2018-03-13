function area = chainArea(chain)

a_nodes = chain(1,1:end-1).*(chain(2,2:end) - chain(2,1:end-1));

area = sum(a_nodes);