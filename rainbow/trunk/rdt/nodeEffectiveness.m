function neff = nodeSensingEffectiveness(feval,x,targets,udata)
nt=size(targets,2); 
neff=zeros(1,nt); 
for n=1:nt 
    neff(n)=feval(x,targets(:,n),udata); 
end