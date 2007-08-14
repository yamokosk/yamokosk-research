function [cc,ind] = find_set(ds,c)
cc = [];
for n=1:length(ds)
    vec = ds.sets{n};
    if ~isempty( find(vec==c) ) cc = vec; ind = n; break; end
end