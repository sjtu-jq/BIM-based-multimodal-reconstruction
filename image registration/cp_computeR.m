function [rmse,entro,S] = cp_computeR(affmat, IgrayAff, P1, P2)
%CP_COMPUTER compute the criteria R
len = length(P1);
P2pro = [P1 ones(len,1)] * affmat.T;
P2pro = P2pro(:,1:2) ./ P2pro(:,3);
rmse = sqrt( sum((P2pro(:,1)-P2(:,1)).^2 + (P2pro(:,2)-P2(:,2)).^2) / length(P2) );
entro = entropy(IgrayAff);
GLevel = 1:1:255;
histbin = histcounts(IgrayAff, GLevel);
S = sum(histbin) / ( size(IgrayAff,1) * size(IgrayAff,2) );
end

