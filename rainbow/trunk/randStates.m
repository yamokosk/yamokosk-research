function states = randStates(scene, pp, t)
NF = length(t);
states = cell(NF,1);
numValid = zeros(NF,1);
[src,sen] = loadRobots;

% RLG params
target.del = .010;
target.dbar = .100;
target.sigma_d = .05;
target.Lbar = 1;
target.sigma_L = 0.3;
target.sigma_theta = pi/6;

% Num tries
N = 1000;
giveup = 1000;

totaltime = 0;
for n = 98:98 %NF
    goodStates = zeros(N*36, 12);
    target.gvars = interpGaitVariables(pp,t(n));
    
    for m = 1:N
        [c,elapsed] = rlg(scene,src,sen,target);
        totaltime = elapsed + totaltime;
        %c = rlg_old(scene,src,sen,target);
        
        if ~isempty(c)
            nGoodStates = size(c,1);
            goodStates(numValid(n)+1:numValid(n)+nGoodStates,:) = c;
            numValid(n) = numValid(n) + nGoodStates;
        end
        
        if mod(m,giveup) == 0
            if numValid(n) == 0
                break;
            end
        end
    end
    
    if numValid(n) > 0
        states{n} = goodStates(1:numValid(n),:);
    end
    fprintf(1, 'Frame %d of %d completed. Found %d states.\n', n, NF, numValid(n));
end

fprintf(1,'Mean run-time: %f\n',totaltime/N);