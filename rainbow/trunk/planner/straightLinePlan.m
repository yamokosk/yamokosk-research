function val = straightLinePlan(c,n,opts)

validate = opts.Validate;

% Sample a straight line between the two configurations and check to see if
% those samples are valid
alpha = linspace(0,1,9);
for k = [5,3,7,2,6,4,8]
    xi = alpha(k)*(n-c) + c;
    if ~validate(xi,opts)
        val = false;
        break;
    end
end

val = true;