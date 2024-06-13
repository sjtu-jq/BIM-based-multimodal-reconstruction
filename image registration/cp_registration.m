function [P1, P2, tform] = cp_registration(input_I1, input_I2, iterationNum, showflag)
% [P1, P2, tform] = cp_registration(I1, I2, iterationNum, showflag)

if nargin == 2
    iterationNum = 1;
    showflag = false;
elseif nargin == 3
    showflag = false;
end

I1gray = input_I1;
I2gray = input_I2;

if size(input_I1, 3) == 3
    I1gray = rgb2gray(input_I1);
end
if size(input_I2, 3) == 3
    I2gray = rgb2gray(input_I2);
end

height = min( [size(I1gray,1), size(I2gray,1), 800]);
[I1, I2, scale] = cp_resizeImage(I1gray, I2gray, height);


I1_itea = I1;
iteration = 0;
maxRMSE = 12; 

AffineTrans = zeros([3 3 iterationNum]);
while  iteration < iterationNum
    % fprintf('\n%d(th) iteration of registration...\n',iteration);
    % for infrared matching, I recommend:
    % [P1, P2, ~] = cp_singleRegis(I1_itea, I2, 20,   maxRMSE, iteration,  2,    2,      6,    showflag    , I2gray);
    % otherwise:
    % [P1, P2, ~] = cp_singleRegis(I1_itea, I2, 20,   maxRMSE, iteration,  0,    0,      6,    showflag    , I2gray);
    
    [P1, P2, ~] = cp_singleRegis(I1_itea, I2, 20,   maxRMSE, iteration,  2,    2,      6,    showflag    , I2gray);
               % cp_registration(I1,      I2, theta,maxRMSE, iteration, zoom+,zoom-,   Lc,showflag, I2gray)
    [I1_itea, affmat] = cp_getAffine(I1_itea, I2, P1, P2); % [v1,u1]==[v2,u2]
    iteration = iteration + 1;
    AffineTrans(:,:,iteration) = affmat.T;
end
% Points of I1gray after resize 
P1  = [P1 ones([length(P1) 1])];
for iteration = iteration:-1:2
    P1 = P1 / AffineTrans(:,:,iteration-1);
    P1(:,1:2) = P1(:,1:2) ./ P1(:,3);
    P1(:,3) = ones(length(P1),1);
end
P1 = P1(:,1:2);
P2 = P2(:,1:2);
% Correct matches in the source images
P1(:,2) = size(I1gray,1) / 2 + scale(1) * ( P1(:,2)-size(I1,1)/2);
P1(:,1) = size(I1gray,2) / 2 + scale(1) * ( P1(:,1)-size(I1,2)/2);

P2(:,2) = size(I2gray,1) / 2 + scale(2) * ( P2(:,2)-size(I2,1)/2);
P2(:,1) = size(I2gray,2) / 2 + scale(2) * ( P2(:,1)-size(I2,2)/2);

[tform, InlierInd, status_code] = estimateGeometricTransform2D(P2, P1, 'projective', 'MaxNumTrials', 3000, ...
    'MaxDistance', 2*max(size(input_I1,1), size(input_I2,1)) / min(size(input_I1, 1), size(input_I2,1)));
    
end

