function dist = minToRect(pt, lb, ub)
%dist = sum( min([(pt-lb).^2, (ub-pt).^2], [], 2) );
dist = 0;
for ii = 1:length(pt)
    if (pt(ii) < lb(ii)) dist = dist + (lb(ii)-pt(ii))^2; end
    if (pt(ii) > ub(ii)) dist = dist + (pt(ii)-ub(ii))^2; end
end