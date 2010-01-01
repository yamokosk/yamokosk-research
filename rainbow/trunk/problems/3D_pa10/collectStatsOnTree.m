close all;
clear all;

solveProblem;

alpha = [0 0.5 1];


% raw_data = {};
% 
% clear G opts;
% opts = plannerSolve('defaults');
% opts.Display = 'off';
% opts.MaxIter = 200;
% opts.TimeStep = 1/25;
% 
% allLengths = {};
% allEffectiveness = {};
% allEfficiencies = {};
% supLength = -inf;
% supEffectiveness = -inf;
% supEfficiency = -inf;
% 
% for n = 1:length(alpha)
%     opts.SkewFactor = alpha(n);
%     G = plannerSolve(X0, target, f, opts, udata);
%     [stats, allLengths{n,1}, allEffectiveness{n,1}, allEfficiencies{n,1}, nodesExtendedCount, h]  = compute_search_tree_stats(G,false);
%     
%     maxLength = max(allLengths{n,1});
% 	if ( maxLength > supLength )
% 		supLength = maxLength;
% 	end
% 	
% 	maxEffectiveness = max(allEffectiveness{n,1});
% 	if ( maxEffectiveness > supEffectiveness )
% 		supEffectiveness = maxEffectiveness;
% 	end
% 	
% 	maxEfficiency = max(allEfficiencies{n,1});
% 	if ( maxEfficiency > supEfficiency )
% 		supEfficiency = maxEfficiency;
% 	end
% 	disp(['alpha=' num2str(alpha(n))]);
% end
% 
% effectivenessBinCenters = linspace(0,supEffectiveness,30);
% efficiencyBinCenters = linspace(0,supEfficiency,30);
% lengthBinCenters = linspace(0,supLength,30);
% 
% supEffectivenessFreq = -inf;
% supEfficiencyFreq = -inf;
% supLengthFreq = -inf;
% for n = 1:length(alpha)
% 	figure(1);
% 	subplot(1,3,n);
% 	N = hist(allEfficiencies{n,1},efficiencyBinCenters);
% 	bar(efficiencyBinCenters,N);
% 	title(['\alpha=' num2str(alpha(n)) ]);
% 	drawnow;
% 
% 	if ( max(N) > supEfficiencyFreq )
% 		supEfficiencyFreq = max(N);
% 	end
% 
% 	figure(2);
% 	subplot(1,3,n);
% 	N = hist(allEffectiveness{n,1},effectivenessBinCenters);
% 	bar(effectivenessBinCenters,N);
% 	title(['\alpha=' num2str(alpha(n)) ]);
% 	drawnow;
% 	
% 	if ( max(N) > supEffectivenessFreq )
% 		supEffectivenessFreq = max(N);
% 	end
% 	
% 	figure(3);
% 	subplot(1,3,n);
% 	N = hist(allLengths{n,1},lengthBinCenters);
% 	bar(lengthBinCenters,N);
% 	title(['\alpha=' num2str(alpha(n)) ]);
% 	drawnow;
% 	
% 	if ( max(N) > supLengthFreq )
% 		supLengthFreq = max(N);
% 	end
% end
% 
% for n = 1:length(alpha)
% 	figure(1);
% 	subplot(1,3,n);
% 	axis([0,supEffectiveness,0,supEffectivenessFreq]);
% 	
% 	figure(1);
% 	subplot(1,3,n);
% 	axis([0,supEfficiency,0,supEfficiencyFreq]);
% 	
% 	figure(1);
% 	subplot(1,3,n);
% 	axis([0,supLength,0,supLengthFreq]);
% end

K = 3;
M = length(alpha);
for n = 1:M
	figure(1);
    set(1,'Color',[1 1 1]);
    
	clear G opts;
	opts = plannerSolve('defaults');
	%opts.Display = 'off';
	opts.MinTimeStep = 2e-3;
  	opts.MaxTimeStep = 0.04;
	opts.TimeStep = 1/25;

    for k = 1:K
		opts.SkewFactor = alpha(n);
		
		G = plannerSolve(X0, target, f, opts, udata);
		opts.PriorSearchTree = G;
		
		weights = G.Wv;
		ind = find(weights > 1);
		if (~isempty(ind))
			fprintf(1,'node weight >1: %d\n',ind(1));
		end
		figure(1);
		subplot(K,M,sub2ind([K,M],n,k));
		hist(weights,20);
		title(['\alpha=' num2str(alpha(n)) ', iter=' num2str(k*opts.MaxIter)]);
		drawnow;
		disp(['alpha=' num2str(alpha(n)) ', iter=' num2str(k*opts.MaxIter)]);
	end
end

% raw_data = {};
% 
% timestep = linspace(0.04,.2,10);
% M = length(timestep);
% N = length(alpha);
% siz = [M,N];
% ind = 1;
% 
% for m = 1:length(timestep)
% 	clear G opts;
% 	opts = plannerSolve('defaults');
% 	opts.Display = 'off';
% 	opts.MaxIter = 200;	
% 	opts.TimeStep = timestep(m);
% 	
% 	for n = 1:length(alpha)
% 	
% 		opts.SkewFactor = alpha(n);
% 		
% 		G = plannerSolve(X0, target, f, opts, udata);
% 		[stats, branchLengths, branchSensingEfficiencies, nodesExtendedCount, h]  = compute_search_tree_stats(G,false);
% 
% 		raw_data{n,m} = {stats, branchLengths, branchSensingEfficiencies, nodesExtendedCount};
% 		
% 		figure(1);
% 		subplot(M,N,ind);
% 		hist(branchSensingEfficiencies,30);
% 		title(['\alpha=' num2str(alpha(n)) ', ts=' num2str(timestep(m))]);
% 		drawnow;
% 		
% 		figure(2);
% 		subplot(M,N,ind);
% 		hist(branchLengths,30);
% 		title(['\alpha=' num2str(alpha(n)) ', ts=' num2str(timestep(m))]);
% 		drawnow;
% 		disp(['alpha=' num2str(alpha(n)) ', ts=' num2str(timestep(m))]);
% 		
% 		ind = ind + 1;
% 	end
% end
