function correctIndex = cp_Ransac(p1,p2,maxErr)
%CP_RANSAC use built-in RANSAC function to obtain the fine matching points
%   使用内置的RANSAC函数获取精配准匹配点
% correctIndex = cp_Ransac(p1,p2,edge1,edge2,maxErr)
% p1: coarse matches of image 1
% p2: coarse matches of image 2
% edge1: the edge image of the image 1
% edge2: the edge image of the image 2
% maxErr: the maximum error that distinguishs the outliers with inliers

[NoUsed, maskIndex] = estimateGeometricTransform2D( p1(:,1:2), p2(:,1:2), 'projective','MaxNumTrials', 2000, 'MaxDistance', maxErr);
correctIndex = find(maskIndex);

end

