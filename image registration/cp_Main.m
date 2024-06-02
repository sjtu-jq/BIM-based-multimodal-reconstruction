% CFO-C2F registration for infrared and visible images of power equipment
% Author：Qian Jiang (in chinese:姜骞)
% First version：2019-04-29
% Current version：2020-02-28
%% section I: Read source images
close all;
clear;
clc;

set(0, 'defaultfigurecolor', 'w');

[I1gray, I2gray, I1rgb, I2rgb, f1, f2, path] = cp_readImage(0);
%% section II: Resize images based on the minimum imaclosege height
height = min( size(I1gray,1), 800);
[I1, I2, scale] = cp_resizeImage(I1gray,I2gray,height);
%% section III: Registrate iteratively & Coarse matching
WarningInfo = input('记得针对输入的图像修改迭代次数、可见光图像采样层数!(输入任意字符退出提醒)\n');
try
    [P1, P2] = cp_registration(I1gray, I2gray, 1, false);
catch ME
    error('Registration failed!');
end
%% section IV: Fine matching
P3 = cp_subpixelFine(P1,P2);
%% section V: Evaluation critea of the registration results
[IgrayAff,affmat] = cp_getAffine(I1gray, I2gray, P1, P3);
IrgbAff = cp_rgbAffine(I1rgb, I2gray, affmat);
Igraymosaic = cp_graymosaic(I1gray, I2gray, affmat, 'loose');
Irgbmosaic = cp_rgbmosaic(I1rgb, I2rgb, affmat, 'loose');
% imwrite(Irgbmosaic,[path f1(1:end-4) '_' f2(1:end-4) '_Mosaic.jpg']);
cp_showMatch(I1rgb, I2rgb, P1, P3, [], 'After Subpixel Fineing');
figure, imshow(Igraymosaic)
figure, imshow(Irgbmosaic);
figure, imshow(IrgbAff);
% imwrite(IrgbAff, [path(1:end-7) 'infraredImages\' f2(1:end-4) '_infrared.jpg'])
%% Mannually compute projective matrix
% %%
% [I1gray, I2gray, I1rgb, I2rgb, f1, f2, path] = cp_readImage(0);
% [I1_aff,refaffmatT] = cp_manuallyTrans(I1rgb,I2rgb);
% cp_showResult(I1rgb,I2rgb,I1gray,I2gray,refaffmatT,1);
% refTrans = refaffmatT.T
% Iaffine = I2rgb;
% Iaffine(:,:,1) = imwarp(I1rgb(:,:,1),refaffmatT,'OutputView',imref2d(size(I2rgb(:,:,1))));
% Iaffine(:,:,2) = imwarp(I1rgb(:,:,2),refaffmatT,'OutputView',imref2d(size(I2rgb(:,:,1))));
% Iaffine(:,:,3) = imwarp(I1rgb(:,:,3),refaffmatT,'OutputView',imref2d(size(I2rgb(:,:,1))));
% Igraymosaic = cp_graymosaic(I1gray, I2gray, refaffmatT, 'crop');
% Irgbmosaic = cp_rgbmosaic(I1rgb, I2rgb, refaffmatT, 'crop');
% figure, imshow(Igraymosaic)
% figure, imshow(Irgbmosaic);
% figure, imshow(Iaffine);
% imwrite(Iaffine, [path(1:end-7) 'infraredImages\' f2(1:end-4) '_infrared.jpg'])
% imwrite(Irgbmosaic, [path(1:end-7) 'infraredImages\' f2(1:end-4) '_infrared_2.jpg'])
