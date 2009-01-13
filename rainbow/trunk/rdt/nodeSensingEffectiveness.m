function neff = nodeSensingEffectiveness(node, targets, neval)
nt=size(targets,2); 
neff=zeros(1,nt); 
for n=1:nt 
    neff(n)=neval(node,targets(:,n)); 
end