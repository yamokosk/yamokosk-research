clear all;



for n=1:3
	data = raw_data{n};
	sensingEff = data{3};
end

pairs = [1,2; 2, 3; 1, 3];

for n=1:size(pairs,1)
	h = ttest2(sensingEff{pairs(n,1)}, sensingEff{pairs(n,1)});
end