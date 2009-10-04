clear function

source_files = { ...
    'mex_coriolis.cpp'; ...
    'mex_inv_kin.cpp'; ...
    'mex_kine.cpp'; ...
    'mex_delta_torque.cpp'; ...
    'mex_jacobian.cpp'; ...
    'mex_roboop.cpp'; ...
    'mex_gravity.cpp'; ...
    'mex_jacobian_DLS_inv.cpp'; ...
    'mex_torque.cpp'; ...
    'mex_inertia.cpp'; ...
    'mex_jacobian_dot.cpp'; ...
    'mex_acceleration.cpp'};

num_files = size(source_files,1);

for n = 1:num_files
    %if (ismac)
    %    str = ['mex ' source_files{n} ' -I/usr/local/include -I/usr/local/include/roboop -I/usr/local/include/newmat -L/usr/local/lib -lnewmat -lroboop'];
    %else
        str = ['mex ' source_files{n} ' -IC:\roboop-1.31.0\include -IC:\roboop-1.31.0\newmat -LC:\roboop-1.31.0\lib -lnewmat -lroboop'];
    %end
    fprintf('Compiling %s... ', source_files{n});
    eval(str);
    fprintf(' done.\n');
end