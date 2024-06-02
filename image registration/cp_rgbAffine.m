function IrgbAff = cp_rgbAffine(I1rgb,I2, affmat)
%CP_RGBAFFINE 
IrgbAff = I1rgb;
IrgbAff = imresize(IrgbAff, [size(I2, 1) size(I2, 2)]);
IrgbAff(:,:,1) = imwarp(I1rgb(:,:,1),affmat,'OutputView',imref2d(size(I2)));
IrgbAff(:,:,2) = imwarp(I1rgb(:,:,2),affmat,'OutputView',imref2d(size(I2)));
IrgbAff(:,:,3) = imwarp(I1rgb(:,:,3),affmat,'OutputView',imref2d(size(I2)));
end

