function [regis_points1, regis_points2, cor12] = cp_singleRegis(I1,I2,maxtheta,maxErr,iteration,zoomascend,zoomdescend, Lc,showflag,I2ori)
%========= CP_REGISTRATION 图像配准主代码=========%
% fprintf('\n【1】Image registrating...\n\tplease wait for few seconds...\n')

%% Corner detection and orientation computed
%                                                 I  C  ang sigm  H  L  endp  gap
[cor1, orientation1, I1edge] = cp_cornerDetection(I1,[], [], [], [], 0,  1,  1,    Lc,iteration==0); % Length of curvature
[cor2, orientation2, I2edge] = cp_cornerDetection(I2,[], [], [], [], [], 1,  1,    Lc,iteration==0);
cor12 = [cor1(:,2) cor1(:,1); 0 0; cor2(:,2) cor2(:,1)];
I1 = double(I1);
I2 = double(I2);
I2ori = double(I2ori);
% fprintf('\tNumber of corner1: %d || Number of corner2: %d \n',length(cor1),length(cor2));
%% show orientation of each corner
% figure,
% subplot(121),imshow(I1edge);
% subplot(122),imshow(I2edge);
%% extract descriptor of Image 1
scale12 = 36; % square width is 6(pixels) * 4(squares) = 24 pixels totally
cols1=cor1(:,1); % swap row and col
rows1=cor1(:,2);  % row is x axis | col is y axis
s1 = scale12 * ones(size(cols1));
if iteration > 0% no need to caculate orientation
    o1 = zeros(size(cols1));    
else
    o1 = orientation1;
end
key1 = [rows1';cols1';s1';o1'];
des1 = cp_descriptor(I1,key1);
%% extract descriptors of Image 2 under multi scales
cols2 =cor2(:,1); % swap row and col
rows2  =cor2(:,2);  % row is x axis | col is y axis
if iteration > 0  % no need to caculate orientation
    o2 = zeros(size(cols2));
else
    o2 = orientation2;
end
s2 = scale12 * ones(size(cols2));
key2 = [rows2';cols2';s2';o2'];
% zoom transformation parameters
zoomstepup = 0.35;
zoomstepdown = 0.25;
des2 = zeros([128 length(cor2) zoomascend+zoomdescend+1]);
des2(:,:,1) = cp_descriptor(I2,key2);
level = 1;
scale = size(I2ori,1) / size(I2,1);
if iteration == 0
    for i=1:zoomascend
        level = level + 1;
        key2zoom = key2;
        I2zoom = imresize(I2ori,(1+zoomstepup*i) / scale);
        key2zoom(1:2,:) = floor( (1+zoomstepup*i) * key2zoom(1:2,:) );
        des2(:,:,level) = cp_descriptor(I2zoom,key2zoom);
    end
    for i=1:zoomdescend
        level = level + 1;
        key2zoom = key2;
        I2zoom = imresize(I2ori,(1-zoomstepdown*i)/scale);
        key2zoom(1:2,:) = floor( (1-zoomstepdown*i) * key2zoom(1:2,:) );
        des2(:,:,level) = cp_descriptor(I2zoom,key2zoom);
    end
end
%% *----Matchings the keys from two images---* 
%% coarsely match with BBF method
[matchIndex1, matchIndex2, zoom] = cp_match(des1',des2, 0.97);

zoomscale = (zoom == 1)*1 + (1<zoom & zoom <= (1+zoomascend))*(1+(zoom-1)*zoomstepup) +...
    (zoom > (1+zoomascend))*(1-(zoom-1-zoomascend)*zoomstepdown);
regis_points111 = [cor1(matchIndex1,2) cor1(matchIndex1,1) o1(matchIndex1)]; % (u,v,orientation)
regis_points222 = [cor2(matchIndex2,2) cor2(matchIndex2,1) o2(matchIndex2)];
% fprintf('\tNumber of features1: %d || Number of features2: %d \n',length(regis_points111),length(regis_points222));
if size(regis_points111,1) < 2
    error('ERROR: No suffcient points( < 2 ) ! Faild to registrate!');
end
if showflag
        cp_showMatch(I1,imresize(I2ori,zoomscale/scale),regis_points111(:,1:2),...
                     zoomscale*regis_points222(:,1:2),[],'Putative matches'); 
end
% cp_showMatch(I1,I2,regis_points111,regis_points222,[],'BBF matching result');
%% Scale invariant tilt angle consistency matching
if iteration == 0
    delta0 = mod(round(180 / pi*(regis_points222(:,3) - regis_points111(:,3))),360);
    % delta0 = delta0 - mod(delta0,2); % Take Δorientation = 2°as a various level 
    dd = 6;
    d_delta = 0 : dd : 360;
    n_delta = histc(delta0, d_delta);
    [~,nindex] = sort(n_delta ,'descend' );
    n0 = nindex(1);
    nmat = [n0^2, n0, 1;(n0-1)^2, n0-1, 1;(n0+1)^2, n0+1, 1] \ ...
           [n_delta(n0) n_delta(n0-1+360/dd*(n0==1)) n_delta(n0+1-(n0==360/dd))]';
    Modetheta_discrete = -nmat(2)/ 2 /nmat(1); % -b/2a

    Modetheta = Modetheta_discrete * dd; % 抛物线插值
%         x_dd = (Modetheta_discrete*dd - 1.2*dd):0.01*dd:(Modetheta_discrete*dd +1.2*dd);
%         xx = x_dd / dd;
%         yy = xx.^2 * nmat(1)+xx * nmat(2) + nmat(3);
%         figure, bar(d_delta,n_delta,'Facecolor','cyan');%[1 0.8 0.6]
%         xlabel('\Delta\phi(\circ)');ylabel('候选旋转角个数');
%         hold on, plot(x_dd-dd,yy,'LineWidth',2), 
%                  plot(dd*[n0 n0-1+360/dd*(n0==1) n0+1-(n0==360/dd)]-dd,[n_delta(n0) n_delta(n0-1+360/dd*(n0==1)) n_delta(n0+1-(n0==360/dd))],'r*');
%         hold off;
else
    Modetheta = 0;
end
% image to the same direction then those correct matches line will be parallel
trans222 = [regis_points222(:,1)-size(I2,2)/2  regis_points222(:,2)-size(I2,1)/2] * ...
           [cos(Modetheta*pi/180) -sin(Modetheta*pi/180); sin(Modetheta*pi/180) cos(Modetheta*pi/180)];
trans222 = [trans222(:,1)+size(I2,2)/2 trans222(:,2)+size(I2,1)/2];
phi_uv = cp_atan(zoomscale*trans222(:,2)-regis_points111(:,2),...
         zoomscale*trans222(:,1)+size(I1,2)-regis_points111(:,1));
% show result of bilateral matching
if showflag
    cp_showMatch(I1,imresize(imrotate(I2,Modetheta,'crop'),zoomscale),regis_points111(:,1:2),...
                 zoomscale*trans222(:,1:2),[],['After ' num2str(iteration) ' resized and rotated']); 
end
dd = 5; d_phi = -90 : dd : 90;
n_phi = histc(phi_uv,d_phi);
[~,nindex] = sort(n_phi ,'descend' ); n0 = nindex(1);
interval_index = find(n_phi < (n_phi(n0) * 0.2));
interval_index = interval_index - n0;
left_phi = find(interval_index < 0); right_phi = find(interval_index > 0);
maxtheta1 = -dd * interval_index(left_phi(end)); maxtheta2 = dd * interval_index(right_phi(1));
nmat = [n0^2, n0, 1;(n0-1+180/dd*(n0==1))^2, n0-1+180/dd*(n0==1), 1;(n0+1-180/dd*(n0==180/dd))^2, n0+1-180/dd*(n0==180/dd), 1]\...
       [n_phi(n0) n_phi(n0-1+180/dd*(n0==1)) n_phi(n0+1-180/dd*(n0==180/dd))]';
ModePhi_discrete = -nmat(2)/ 2 /nmat(1); % -b/2a
ModePhi = ModePhi_discrete * dd - 90;
delta1 = ModePhi - maxtheta1; delta2 = ModePhi + maxtheta2;
[valid0,~] = find( (phi_uv>= delta1) & (phi_uv <=delta2) );

Dist = sqrt((zoomscale*trans222(valid0,2)-regis_points111(valid0,2)).^2+(zoomscale*trans222(valid0,1)+size(I1,2)-regis_points111(valid0,1)).^2);
meandist = mean(Dist);
[valid1,~] = find( (Dist>= 0.5*meandist) & (Dist <=1.5*zoomscale*meandist) );

regis_points11 = regis_points111(valid0(valid1),:);
regis_points22 = regis_points222(valid0(valid1),:);

% % show result of Tile angle consistency matching
if showflag
    cp_showMatch(I1,imresize(imrotate(I2,Modetheta,'crop'),zoomscale),regis_points111(valid0(valid1),1:2),...
                 zoomscale*trans222(valid0(valid1),1:2),[],['After ' num2str(iteration) ' Tile angle consistency']);
end
if iteration == 0
    correctindex = cp_mismatchRemoval(regis_points11, regis_points22, I1, I2, 1.0 * maxErr);
else
    correctindex = cp_mismatchRemoval(regis_points11, regis_points22, I1, I2, 1.0 * maxErr);
end

% if iteration == 0
%     correctindex = cp_Ransac(regis_points11,regis_points22,maxErr); % new version: 2022-09-23
% else
%     correctindex = cp_Ransac(regis_points11,regis_points22,5);
% end

regis_points1 = regis_points11(correctindex,1:2);
regis_points2 = regis_points22(correctindex,1:2);

