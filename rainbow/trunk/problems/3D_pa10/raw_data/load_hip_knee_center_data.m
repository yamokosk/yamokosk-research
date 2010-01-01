function [p_knee, p_hip] = load_hip_knee_center_data(filename)

fid = fopen(filename);

% First 5 lines are junk.
for n = 1:5
    line = fgetl(fid);
end

base_format = '%f%f%f';
format = '%*d';
for n = 1:10
    format = [format base_format];
end

p_hip = zeros(101,3,5);
p_knee = zeros(101,3,5);

n = 0;
while ( ~feof(fid) )
    n = n + 1;
    line = fgetl(fid);
    
    A = sscanf(line, format);
    
    p_hip(n,:,1) = [A(1) A(2) A(3)];
    p_hip(n,:,2) = [A(7) A(8) A(9)];
    p_hip(n,:,3) = [A(13) A(14) A(15)];
    p_hip(n,:,4) = [A(19) A(20) A(21)];
    p_hip(n,:,5) = [A(25) A(26) A(27)];
    
    p_knee(n,:,1) = [A(4) A(5) A(6)];
    p_knee(n,:,2) = [A(10) A(11) A(12)];
    p_knee(n,:,3) = [A(16) A(17) A(18)];
    p_knee(n,:,4) = [A(22) A(23) A(24)];
    p_knee(n,:,5) = [A(28) A(29) A(30)];
end

fclose(fid);