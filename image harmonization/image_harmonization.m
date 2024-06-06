%% Basic information
% recommended MATLAB version >= R2020b

clc; clear; close all force;

%% add the image registration function into workpath
addpath('../image registration/');

%% initial the multimodal images list

visible_list  = MyDir('.\test images\visible', 'jpg');
infrared_list = MyDir('.\test images\infrared', 'bmp');

%% harmonize the images' color to be the consistent with the reference image

ref_img = imread(fullfile(visible_list(1).folder, visible_list(1).name));

for imgid = 2 : length(visible_list)
    to_harm_img = imread(fullfile(visible_list(imgid).folder, visible_list(imgid).name));
    
    [P1, P2, tform] = cp_registration(ref_img, to_harm_img, 1);
    harmornized_img = data_harmonize(ref_img, to_harm_img, P1, P2);
    
    figure('Name', 'Image harmonization result', 'Color', 'w'),
    subplot(131), imshow(ref_img), title('reference image');
    subplot(132), imshow(to_harm_img), title('original image');
    subplot(133), imshow(harmornized_img), title('harmonized image');
end
%% harmonize image sequence based on the SFM results
% match_list = readmatrix('../image decision/python scripts/match_pairs_id.txt');
% img_number = length(visible_list);
% 
% % for each image, compute the number of its matching image
% match_static = zeros(1, img_number);
% 
% % initial adjacent matrix according to match_list
% match_adj = diag(10000 * ones(1, img_number));
% 
% for imgid = 1 : size(match_list, 1)
%     match_static(match_list(imgid, 1:2)) = match_static(match_list(imgid, 1:2)) + 1;
%     match_adj(match_list(imgid, 1), match_list(imgid, 2)) = match_list(imgid, 3);
%     match_adj(match_list(imgid, 2), match_list(imgid, 1)) = match_list(imgid, 3);
% end

% vis_harm_flag = false(1, img_number);
% [~, seed_ind] = max(match_static);
% vis_harm_flag(seed_ind) = true;
% 
% while ~isempty( find(~vis_harm_flag, 1) )
%     wait_for_harm = find( ~vis_harm_flag );
%     % for each image, find its best matching reference image
%     for imgid = wait_for_harm
%         [~, ind] = sort(match_adj(imgid,:), 'Descend');
%         found_ind = find(vis_harm_flag(ind), 1);
%         best_ind = ind(found_ind);
%         if match_adj(imgid, best_ind) == 0
%             continue;
%         end
%         
%         fprintf('match: %d -> %d, number: %d\n', imgid, best_ind, match_adj(imgid, best_ind));
%         ref_img = imread(fullfile(visible_list(best_ind).folder, visible_list(best_ind).name));
%         har_img = imread(fullfile(visible_list(imgid).folder, visible_list(imgid).name));
%         
%         % harmonize the color distribution of current image
%         [P1, P2, tform] = cp_registration(ref_img, har_img, 1);
%         harmornized_img = data_harmonize(ref_img, har_img, P1, P2);
% 
%         % figure('Name', 'Matching points', 'Color', 'w'),
%         % showMatchedFeatures(ref_img, har_img, P1, P2, 'montage');
% 
%         figure('Name', 'Image harmonization result', 'Color', 'w'),
%         subplot(131), imshow(ref_img), title('reference image');
%         subplot(132), imshow(har_img), title('original image');
%         subplot(132), imshow(harmornized_img), title('harmonized image');
%         
%         vis_harm_flag(imgid) = true;
%         %         imwrite( fullfile('.\Source multimodal virtual images\Robot\visible_harmonized', visible_list(imgid).name) );
%     end
% end
