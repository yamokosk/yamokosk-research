function [N_matrix] = nmatrix(Points, Order, numberPout)
% BSPLINE B-spline data generation.
%   [POUT, N] = BSPLINE(Points, numberPoints, numberDegree) generates the
%   original curve (Pout) from a set of nodes (Points). It also outputs the
%   N-matrix so that you could generate the same curve with:
%
%       Pout(:,j)= Points*N(1, 1:numberPoints)'
tf = 2*pi;
numberPoints = max(size(Points));

% Check if the degree of the curve is greater than number of control points minus one.
if Order > (numberPoints-1),
	error(sprintf('Error: the maximum order of the B-spline cannot exceed %d', numberPoints-1));
end

% Define the parameters n (allowing indices to start at zero for n+1 control points) 
% and d (controlling degree of polynomial in u and curve continuity).
n = numberPoints - 1;
d = Order + 1;

% Define parametric knots (or knot values) Uj for open curve B-spline,
% which relate the parametric variable u to the Pj control points.
for tempJ = 1:(n + d + 1),
	j = tempJ - 1;
	if j < d
		Uj(tempJ) = 0;
    elseif j >= d & j <= n
		Uj(tempJ) = j - d + 1;
    elseif j > n
		Uj(tempJ) = n - d + 2;
	end
end

Ujmax = max(Uj);
Uj = Uj * (tf/Ujmax);

% Define the range of the parametric variable u.
umax = n - d + 2;
u_range = 0:(umax/(numberPout-1)):umax;

u_range = u_range * (tf/umax);
umax = tf;

% temp = u_range;
% u_range = [linspace(temp(1), temp(3), 5) temp(4:(length(temp)-3)) linspace(temp(length(temp)-2), temp(length(temp)), 5)];

% Allocate memory for the output points matrix Pout and B-spline basis blending functions N (a weighting function.
Pout = zeros(3, (size(u_range, 2)));
N = zeros(1, (n + d));
Np = zeros(1, (n + d));

% Instantiate the column index for the output points matrix Pout.
column = 1;

% Instantiate the N matrix for the shape functions.
N_matrix = [];

% Loop through the entire range of the parametric variable u.
for u = u_range,
    
    % Loop through each normalized polynomial degree in u.
    for k = 1:d
        
        % Check if the normalized polynomial degree in u is equal to one.
        if k == 1,
            
            % Loop through each parametric knot (or knot values).
            for i = 1:numberPoints,
                
                % Calculate the first order basis functions.
                if (u >= Uj(i)) & (u < Uj(i+1)),
                    N(i) = 1;
                elseif (abs(u - umax) < 1e-8) & (abs(u - Uj(i) - (tf/Ujmax)) < 1e-8) & (u - Uj(i+1) <= 1e-8),
                    N(i) = 1;
                else,
                    N(i) = 0;
                end
                
            end	% End loop through each parametric knot (or knot values).
            
        % Otherwise if the normalized polynomial degree in u is not equal to one.    
        else
            
            % Loop through each parametric knot (or knot values).
            for i=1:numberPoints
                
                % Calculate the higher order basis functions.
                if ((Uj(i+k-1) - Uj(i)) == 0) & ((Uj(i+k) - Uj(i+1)) == 0),
                    N(i) = 0;
                elseif ((Uj(i+k-1) - Uj(i)) == 0) & ((Uj(i+k) - Uj(i+1)) ~= 0),
                    N(i) = ((Uj(i+k) - u)*N(i+1)) / (Uj(k+i) - Uj(i+1));
                elseif ((Uj(i+k-1) - Uj(i)) ~= 0) & ((Uj(i+k) - Uj(i+1)) == 0),
                    N(i) = ((u - Uj(i))*N(i)) / (Uj(i+k-1) - Uj(i));
                else,
                    N(i) = ((u - Uj(i))*N(i) / (Uj(i+k-1) - Uj(i))) + ((Uj(i+k) - u)*N(i+1) / (Uj(k+i) - Uj(i+1)));
                end
                
            end % End loop through each parametric knot (or knot values).
            
        end % End check if the normalized polynomial degree in u is equal to one or not.
        
    end % End of for loop through each polynomial degree in u.
    
    % Compute (or generate) the B-spline curve.
    Pout(:, column) = Points*N(1, 1:numberPoints)';
    column = column + 1;
    
    % Compile all time frames of the shape functions.
    N_matrix = [N_matrix; N(1, 1:numberPoints)];
    
end % End of for loop through the entire range of the parametric variable u.







