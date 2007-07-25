function dist = maxToRect(pt, lb, ub)
%dist = sum( max([(pt-lb).^2, (ub-pt).^2], [], 2) );
dist = 0;
for ii = 1:length(pt)
    if (pt(ii) >= ((lb(ii) + ub(ii))/2) ) dist = dist + (pt(ii)-lb(ii))^2; 
    else dist = dist + (ub(ii)-pt(ii))^2; end
end