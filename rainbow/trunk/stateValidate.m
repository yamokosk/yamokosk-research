function val = stateValidate(c, opts)
g = interpGaitVariables(opts.userdata.pp,c(13));
q_names = {'src1';'src2';'src3';'src4';'src5';'src6';'sen1';'sen2';'sen3';'sen4';'sen5';'sen6'};
q = [q_names, num2cell(c(1:12))];
opts.userdata.scene = buildScene(opts.userdata.scene, [q; g]);

if ( ~dSimpleSpaceCollide2(opts.userdata.scene.space{1}.id,opts.userdata.scene.space{2}.id) && ... % Environment with source 
     ~dSimpleSpaceCollide2(opts.userdata.scene.space{1}.id,opts.userdata.scene.space{3}.id) && ... % Environment with sensor
     ~dSimpleSpaceCollide2(opts.userdata.scene.space{2}.id,opts.userdata.scene.space{4}.id) && ... % source with human
     ~dSimpleSpaceCollide2(opts.userdata.scene.space{3}.id,opts.userdata.scene.space{4}.id) ) % sensor with human
	val = true;
else
    val = false;
end
end