function myerrorregion(X,Y,E,lw,color)

figure(gcf);
N = length(X);
for n = 1:N
    line([X(n); X(n)],[Y(n)+E(n); Y(n)-E(n)],'LineWidth',2,'Color',[0.6 0.6 0.6]);
end

% errorbar(X,Y,E,'LineStyle','none','LineWidth',0.5,'Color',[0.6 0.6 0.6]);