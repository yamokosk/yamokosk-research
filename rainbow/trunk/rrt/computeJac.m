syms tf t0 X0 real
N = 6;
for i = 1:6
    for j = 1:6
        str = ['syms D' num2str(i) num2str(j) ' real'];
        eval(str);
    end
    str = ['syms Db' num2str(i) ' X' num2str(i) ' real'];
    eval(str);
end

%c = [];
for i = 1:N
    str = ['c' num2str(i) ' = 2/(tf - t0) * (Db' num2str(i) '*X0 '];
    for k = 1:N
        str = [str ' + D' num2str(i) num2str(k) '*X' num2str(k)];
    end
    str = [str ');'];
    eval(str);
end
