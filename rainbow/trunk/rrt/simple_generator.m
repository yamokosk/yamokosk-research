function node = simple_generator(tree,opts)

test = binornd(1,0.05);

if test == 1
    node = 10*rand(2,1);
else
    % use goal as the node point
    node = opts.goal;
end
