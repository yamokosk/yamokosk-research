close all;
open('data for paper/sensing_eff_different_fixed_time.fig');
figure(1)
set(1,'Color',[1 1 1]);

% Reset everything
for n = 1:(M*N)
	axishandle = subplot(M,N,n);
	%pos = get(axishandle, 'Position');
	set(axishandle, 'XTickLabel', [], ...
					'YTickLabel', [], ...
					'XLabel', [], ...
					'YLabel', [], ...
					'Title', []);
end

% Setup left column
leftcol = 1:3:M*N;
for n = 1:length(leftcol)
	axishandle = subplot(M,N,leftcol(n));
	label = sprintf('T_s = %0.3g', ts(n));
	set(get(axishandle,'YLabel'), 'String', label, 'FontSize',14);
	set(axishandle, 'YTickLabelMode', 'auto', ...
					'FontSize', 12);
end

% Setup bottom row
bottomrow = [28, 29, 30];
for n = 1:length(bottomrow)
	axishandle = subplot(M,N,bottomrow(n));
	set(axishandle, 'XTickLabelMode', 'auto', ...
					'FontSize', 12); 
end

% Setup top row
toprow = [1, 2, 3];
alpha = [0, 0.5, 1.0];
for n = 1:length(toprow)
	axishandle = subplot(M,N,toprow(n));
	label = sprintf('\alpha = %0.1g', alpha(n));
	set(get(axishandle,'Title'), 'String', label, 'FontSize',14);	
end