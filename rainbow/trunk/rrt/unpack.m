function [tf, U, X] = unpack(z, dim, N)
nstates = dim*2;
tf = z(1); U = reshape(z(2:dim*N + 1),dim,N); X = reshape(z(dim*N + 2:end),nstates,N);
end