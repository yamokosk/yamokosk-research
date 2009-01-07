% selsus - stochastic universal selection
%
%
%	Description:
%	The function selects from the old P a required number of strings using
%	the "stochastic universal sampling" method. Under this selection method the number of a 
%       parent copies in the selected new P is proportional to its fitness. 
%	
% 
%	Syntax:
%
%	Newpop=selsus(Oldpop,Oldfit,Num);
%	[Newpop,Newfit]=selsus(Oldpop,Oldfit,Num);
%
%	       Newpop - new selected P
%	       Newfit - fitness vector of Newpop
%	       Oldpop - old P
%	       Oldfit - fitness vector of Oldpop
%	       Num    - required number of selected strings
%

% I.Sekaj, 5/2000

function [Newpop,Newfit,I] = selsus(P,Wp,n)

[dimp,Np] = size(P);
epsilon = 1e-5;

% Allocate some temporary storage
w = zeros(1,Np+1);
w0 = zeros(1,Np+1);
Newpop = zeros(dimp,n);
Newfit = zeros(1,n);
I = zeros(1,n);

% Shift fitnesses so that min(Wp)=1
Lp = Wp - min(Wp) + 1;

% Store total fitness of entire population
sumLp = sum(Lp);

% Create a roulette wheel on the interval (0,100]
w0(1,1:Np) = Lp*inv(sumLp);

for i = Np:-1:1
    w(i) = w(i+1) + w0(i);
end

maxw = max(w);
if maxw == 0 % guard against divide by zero
    maxw = epsilon;
end

w = (w/maxw)*100; % rescale to (0,1] then multiply by 100

% Create randomly sized, equally spaced bins
del=100/n;
b0 = rand * del - epsilon; % bin size
b = (0:n-1)*del + b0;

for i=1:n			
    bin_index = intersect( find( w < b(i) )-1, find( w > b(i) ) );
    Newpop(:,i)=P(:,bin_index);
    Newfit(i)=Wp(bin_index);
    I(i) = bin_index;
end
