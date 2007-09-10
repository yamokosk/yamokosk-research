function [a1, a2] = acos2(x)

if x > 0
    a1 = acos(x);
    a2 = -a1;
else
    a1 = acos(x);
    a2 = -a1;
end