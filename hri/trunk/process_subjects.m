clear;
subjects = {'as'; 'cy'; 'dw'; 'bg'; 'bc'; 'ze'};

ap = [];
ml = [];
vert = [];

for n = 1:length(subjects)
    disp(['Processing subject ' subjects{n} '... ']);
    force_bandwidth = process_subject(subjects{n});
    
    ap_band = [force_bandwidth(:,1,1); force_bandwidth(:,1,2)];
    ml_band = [force_bandwidth(:,2,1); force_bandwidth(:,2,2)];
    vert_band = [force_bandwidth(:,3,1); force_bandwidth(:,3,2)];
    
    ap = [ap, ap_band];
    ml = [ml, ml_band];
    vert = [vert, vert_band];
end    

[p_ap, tbl_ap, stats_ap] = anova2(ap,5);
[p_ml, tbl_ml, stats_ml] = anova2(ml,5);
[p_vert, tbl_vert, stats_vert] = anova2(vert,5);