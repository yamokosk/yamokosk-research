function [S,cost,S_ordered,cost_ordered] = preProcessStates(scene, pp, t, states, costfun)
statesPerStep = cellfun(@length, states);
totalStates = sum(statesPerStep );

ptr = 1;
S_ordered = zeros(totalStates, 12+1);
cost_ordered = zeros(totalStates, 1);
q_names = {'src1';'src2';'src3';'src4';'src5';'src6';'sen1';'sen2';'sen3';'sen4';'sen5';'sen6'};
q = [q_names, num2cell(zeros(12,1))];

[srcRbt,senRbt] = loadRobots();
ptr = 1;
for s = 1:length(states)
    statesInStep = states{s};
    
    g = interpGaitVariables(pp,t(s));
    scene = buildScene(scene, [q; g]);
    targetBody = getBody(scene,'target');
    T_wcs_d = targetBody.T_wcs_obj;
    
    for n = 1:statesPerStep(s)
        T_lab_src = fkine(srcRbt,statesInStep(1:6));
        T_lab_sen = fkine(senRbt,statesInStep(7:12));

        S_ordered(ptr,:) = [statesInStep(n,:), t(s)];
        cost_ordered(ptr,1) = costfun(statesInStep(n,:), T_wcs_d, T_lab_src, T_lab_sen);
        ptr = ptr + 1;
    end
    fprintf(1, 'Frame %d of %d completed. States %d of %d completed.\n', s, length(states), sum(statesPerStep(1:s)), totalStates);
end

fprintf(1, 'Randomly permutating state and cost vectors...');
p = randperm(totalStates)';
S = intrlv(S_ordered, p);
cost = intrlv(cost_ordered, p);
fprintf(1, ' complete.\n');