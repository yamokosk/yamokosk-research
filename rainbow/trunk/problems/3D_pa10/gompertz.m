function y = gompertz(a,b,c,x)
% gompertz(a,b,c,x) - The gompertz function.
%
% A Gompertz curve or Gompertz function, named after Benjamin Gompertz, is
% a sigmoid function. It is a type of mathematical model for a time series, 
% where growth is slowest at the start and end of a time period.
%
%   y(x)=ae^{be^{cx}} 
%
%   * a is the upper asymptote
%   * c is the growth rate
%   * b, c are negative numbers
y = a*exp(b*exp(c*x));