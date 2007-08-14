function val = straightLinePlan(c,n,opts)

validate = opts.Validate;

% Sample a straight line between the two configurations and check to see if
% those samples are valid
alpha = linspace(0,1,10);
for n = 1:length(alpha)
    xi = alpha(n)*(n-c) + c;
    if ~validate(xi,opts)
        val = false;
        break;
    end
end

val = true;