%% Basic information
% recommended MATLAB version = R2020b

clc; clear; close all force;

%% add the image registration function into workpath
addpath('../image registration/');

%% initial the multimodal images list

% path to visible, infrared, ultraviolet, acoustic images

visible_list  = MyDir('\Source multimodal virtual images\Robot\visible', 'png');
infrared_list = MyDir('\Source multimodal virtual images\Robot\infrared', 'png');
uv_list       = MyDir('\Source multimodal virtual images\Robot\ultraviolet', 'png');
acoustic_list = MyDir('\Source multimodal virtual images\Robot\acoustic', 'png');

%% intial the image pose relationship

% match relationship between source visible images is obtained by COLMAP
% sfm pipeline, while corresponding python scripts is given in the folder './python
% scripts/' to generate the needed 'match_pairs_id.txt'

match_list = readmatrix('./python scripts/match_pairs_id.txt');
img_number = length(visible_list);

% for each image, compute the number of its matching image
match_static = zeros(1, img_number);

% initial adjacent matrix according to match_list
match_adj = diag(ones(1, img_number));

for imgid = 1 : length(robot_data.match_list)
    match_static(match_list(imgid, 1:2)) = match_static(match_list(imgid, 1:2)) + 1;
    match_adj(match_list(imgid, 1), match_list(imgid, 2)) = match_list(imgid, 3);
    match_adj(match_list(imgid, 2), match_list(imgid, 1)) = match_list(imgid, 3);
end

%% according to the matching relationship, perform image decision and harmonization

valid_area_alpha = 0.15; % 1-0.85

% decision process for ultraviolet images
% for infrared and acoustic images, just replace the variable 'uv_list' from line 47 to line 152 by
% 'infrared_list' and 'acoustic_list' respectively.

for imgid = 1 : img_number
    vis_img = imread(fullfile(visible_list(imgid).folder, visible_list(imgid).name), 'Background', [1 1 1]);
    wait_for_reg = find(match_adj(imgid, :)>0);
    evaluate_criterion = zeros(length(wait_for_reg), 3);
    transfrom_mat(length(wait_for_reg)).trans = projective2d;
    
    for id = 1 : length(wait_for_reg)
        regid = wait_for_reg(id);
        reg_img = imread(fullfile(uv_list(regid).folder, uv_list(regid).name), 'Background', [1 1 1]);

        %===== 1. registration acccuracy
        try
            [P1, P2, tform, ~, NOEP] = cp_registration(vis_img, reg_img, 0);
        catch
            transfrom_mat(regid).trans = [];
            evaluate_criterion(id, :) = 0;
            continue;
        end

        %===== Projective matrix and image
        transfrom_mat(regid).trans = tform;
        reg_img_trans = imwarp(reg_img, tform, 'OutputView', imref2d(size(vis_img)));
        
        %===== 2. valid area ratio
        reg_mask = im2uint8( ones(size(reg_img, 1), size(reg_img, 2)) );
        reg_mask_trans = imwarp(reg_mask, tform, 'OutputView', imref2d(size(vis_img)));
        row_min = round(valid_area_alpha * size(vis_img, 1));
        row_max = round((1-valid_area_alpha) * size(vis_img, 1));
        col_min = round(valid_area_alpha * size(vis_img, 2));
        col_max = round((1-valid_area_alpha) * size(vis_img,2));
        [u, ~, ~] = find( reg_mask_trans(row_min:row_max, col_min:col_max) == 255 );
        VAP = length(u) / numel(reg_mask_trans);
        
        %===== 3. entropy
        img_entropy = entropy(rgb2gray(reg_img_trans));
        % store the result
        evaluate_criterion(id, 1) = NOEP;
        evaluate_criterion(id, 2) = VAP;
        evaluate_criterion(id, 3) = img_entropy;
        fprintf('Multispectral match: %d -> %d\n%15s\t%15s\t%15s\t%15s\n%15d\t%15.4f\t%15.4f\t%15.4f\n',imgid, regid, ...
            'Matches', 'Acccuracy', 'Valid area', 'Entropy', length(P1), evaluate_criterion(id, :));
        
        %             ori_img = imread(fullfile(inf_list(regid).folder, inf_list(regid).name), 'Background', [1 1 1]);
        %             trans_img = imwarp(ori_img, transfrom_mat(regid).trans, 'OutputView', imref2d(size(vis_img)));
        %             figure('Name', 'Transformation Infrared Image'), imshow(trans_img);
    end
    
    % gassian normalization
    gas_std = length(wait_for_reg) - 1;
    gas_kernel = exp( -(0:1:gas_std) / 2 / gas_std^2 );
    gas_kernel = gas_kernel / sum(gas_kernel);
    [~, rmse_sort] = sort(evaluate_criterion(:, 1), 'Descend');
    [~, valid_area_sort] = sort(evaluate_criterion(:, 2), 'Descend');
    [~, entropy_sort] = sort(evaluate_criterion(:, 3), 'Descend');
    norm_result = zeros(1, length(wait_for_reg));
    norm_result(rmse_sort) = norm_result(rmse_sort) + gas_kernel;
    norm_result(valid_area_sort) = norm_result(valid_area_sort) + gas_kernel;
    norm_result(entropy_sort) = norm_result(entropy_sort) + gas_kernel;
    
    % decise the priority
    [~, decision_sort] = sort(norm_result, 'Descend');
    
    % stitch image data
    stitch_img_data = im2uint8(zeros(size(vis_img)));
    stitch_mask = true(size(vis_img, 1), size(vis_img, 2));
    for dataid = wait_for_reg(decision_sort)
        if isempty(transfrom_mat(dataid).trans)
            continue;
        end
        ori_img = imread( fullfile(uv_list(dataid).folder, uv_list(dataid).name ), 'Background', [1 1 1]);
        trans_img = imwarp(ori_img, transfrom_mat(dataid).trans, 'OutputView', imref2d(size(vis_img)));
        
        ori_mask = im2uint8( ones(size(ori_img, 1), size(ori_img, 2)) );
        ori_mask_trans = imwarp(ori_mask, transfrom_mat(dataid).trans, 'OutputView', imref2d(size(vis_img)));
        
        [v, u] = find(stitch_mask);
        stitch_img_data(v + (u-1)*size(vis_img, 1))                         = trans_img(v + (u-1)*size(vis_img, 1));
        stitch_img_data(v + (u-1)*size(vis_img, 1)+numel(vis_img(:,:,1)))   = trans_img(v + (u-1)*size(vis_img, 1)+numel(vis_img(:,:,1)));
        stitch_img_data(v + (u-1)*size(vis_img, 1)+2*numel(vis_img(:,:,1))) = trans_img(v + (u-1)*size(vis_img, 1)+2*numel(vis_img(:,:,1)));
        
        stitch_mask( ori_mask_trans==255 ) = false;
    end
    [v, u] = find(stitch_mask);
    stitch_img_data(v + (u-1)*size(vis_img, 1))                         = 251;
    stitch_img_data(v + (u-1)*size(vis_img, 1)+numel(vis_img(:,:,1)))   = 155;
    stitch_img_data(v + (u-1)*size(vis_img, 1)+2*numel(vis_img(:,:,1))) = 91;
    
    %     figure('Name', 'Image stitching result', 'Color', 'w'),
    %     subplot(121), imshowpair(vis_img, stitch_img_data, 'montage');
    %     subplot(122), imshowpair(vis_img, stitch_img_data, 'falsecolor');
    
    % write the decision image to the destination path
    dst_img_path = fullfile('\Source multimodal virtual images\Robot\ultraviolet_decision');
    if ~exist(dst_img_path, 'dir')
        mkdir(dst_img_path);
    end
    transfrom_mat = [];
    %     imwrite(stitch_img_data, fullfile(dst_img_path, visible_list(imgid).name));
        
end
