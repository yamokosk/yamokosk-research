clear function

source_files = ls('*.cpp');
num_files = size(source_files,1);

for n = 1:num_files
    str = ['mex ' strtrim(source_files(n,:)) ' -IC:\roboop-1.31.0\include -IC:\roboop-1.31.0\newmat -LC:\roboop-1.31.0\lib -lnewmat -lroboop'];
    fprintf('Compiling %s... ', strtrim( source_files(n,:) ));
    eval(str);
    fprintf(' done.\n');
end