function [p_knee, v_knee, p_hip, v_hip] = prepare_raw_data(filename)

fid = fopen(filename);

% First 5 lines are junk.
for n = 1:5
    line = fgetl(fid);
end

base_format = '%*f%f%f';
format = '%*d';
for n = 1:12
    format = [format base_format];
end

p_hip = zeros(101,6);
v_hip = zeros(101,6);

p_knee = zeros(101,6);
v_knee = zeros(101,6);

n = 0;
while ( ~feof(fid) )
    n = n + 1;
    line = fgetl(fid);
    
    A = sscanf(line, format);
    
    p_hip(n,:) = [A(1) A(3) A(5) A(2) A(4) A(6)];
    p_knee(n,:) = [A(7) A(9) A(11) A(8) A(10) A(12)];
    
    v_hip(n,:) = [A(13) A(15) A(17) A(14) A(16) A(18)];
    v_knee(n,:) = [A(19) A(21) A(23) A(20) A(22) A(24)];
end

fclose(fid);