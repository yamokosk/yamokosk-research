function data = loadOSMIFile(filename)
fid = fopen(filename,'r');
line = fgetl(fid);

n = 1;
data = [];
while ischar(line)
    C = textscan(line,'%s %d %f %f %f');
    if ~isempty(C{3})
        data(n,:) = [C{3},C{4},C{5}];
        n = n + 1;
    end
    line = fgetl(fid);
end
fclose(fid);