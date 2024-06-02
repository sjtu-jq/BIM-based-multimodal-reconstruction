function Icombine = cp_combineImages(ImageDir)
%CP_COMBINEIMAGES input images of some parts of the equipment scene, and
%output one complete image of whole equipment
if nargin==0
    ImageDir = uigetdir('E:/3D_R/TestProgram/','Select the folder storing images to be stitched:');
    % [Filename1,pathname] = uigetfile('*.*','pick 1(st) image');
end

ImageNameList = dir([ImageDir '/*.jpg']);
NumOfImage = length(ImageNameList);

% 以文件夹里面的第一张图像为基础，往后拼接补充第一张图像缺失的部分
Icombine = imread([ImageNameList(1).folder '/' ImageNameList(1).name]);
if size(Icombine,3)==3
    for i = 2:NumOfImage
        Ipart = imread([ImageNameList(i).folder '/' ImageNameList(i).name]);
        [x,y] = find(Ipart(:,:,1)~=0 | Ipart(:,:,2)~=0 | Ipart(:,:,3)~=0);
        for j=1:length(x)
            if sum(Icombine(x(j),y(j),:))==0
                Icombine(x(j),y(j),:) =Ipart(x(j),y(j),:);
            elseif ( sum(Icombine(x(j),y(j),:))~=0 ) && ( sum(Ipart(x(j),y(j),:)) > ( sum(Icombine(x(j),y(j),:)) ) )
                Icombine(x(j),y(j),:) = Ipart(x(j),y(j),:);
            end
        end
    %     Icombine(:) = max(Icombine(:), Ipart(:));
    end
else
    for i = 2:numofpart
        Ipart = imread([ImageNameList(i).folder '/' ImageNameList(i).name]);
        [x,y] = find(Ipart(:,:)>0);
        for j=1:length(x)
            if Icombine(x(j),y(j))==0
                Icombine(x(j),y(j)) =Ipart(x(j),y(j));
            elseif Icombine(x(j),y(j))>0 % &&  ( Ipart(x(j),y(j))> (Icombine(x(j),y(j))+15))
                Icombine(x(j),y(j)) = max(Icombine(x(j),y(j)),Ipart(x(j),y(j)));
            end
        end
    end
end
end
