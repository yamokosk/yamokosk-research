function sol = solve_bvp()
K = 2;
solinit = bvpinit([1,0], [0 0]);
sol = bvp4c(@odefun,@bcfun,solinit);

    function dudx = odefun(x,u)
        dudx = [-x^2 - K*x; -2*x - K];
    end

    function res = bcfun(ya,yb)
        res = [ya(1); yb(1)];
    end

    function u = ufun(x)
        u = -x^2 - K*x;
    end

end
        