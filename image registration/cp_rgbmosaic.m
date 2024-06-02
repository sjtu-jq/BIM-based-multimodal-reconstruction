function Imosaic = cp_rgbmosaic(I1,I2,affmat,format)
% CP_MOSAIC
% input: a pair of source rgb images
% output: a mosaic image based on liniear interplation transformation

if nargin == 3
    format = 'crop';
end

Imosaic(:,:,1) = cp_graymosaic(I1(:,:,1),I2(:,:,1),affmat,format);
Imosaic(:,:,2) = cp_graymosaic(I1(:,:,2),I2(:,:,2),affmat,format);
Imosaic(:,:,3) = cp_graymosaic(I1(:,:,3),I2(:,:,3),affmat,format);

end

