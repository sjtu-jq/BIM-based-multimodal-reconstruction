%% Basic information
% recommended MATLAB version >= R2020b

clc; clear; close all force;

%% add the image registration function into workpath
addpath('../image registration/');

%% initial the multimodal images list

% path to visible, infrared, ultraviolet, acoustic images

visible_list  = MyDir('..\..\Source multimodal virtual images\Robot\visible', 'png');
infrared_list = MyDir('..\..\Source multimodal virtual images\Robot\infrared', 'png');
uv_list       = MyDir('..\..\Source multimodal virtual images\Robot\ultraviolet', 'png');
acoustic_list = MyDir('..\..\Source multimodal virtual images\Robot\acoustic', 'png');

%% intial the image pose relationship

% match relationship between source visible images is obtained by COLMAP
% sfm pipeline, while corresponding python scripts is given in the folder './python
% scripts/' to generate the needed 'match_pairs_id.txt'


% windows CMD to database.db path, and run command: python
% .\python scripts\export_match_relationship.py --database_path database.db
% --match_list_path .\python scripts\match_pairs --min_num_matches 20

% command = ['cd e: & cd ' pwd ' & 'python export_match_relationship.py
% .\python scripts\export_match_relationship.py --database_path database.db
% --match_list_path .\python scripts\match_pairs --min_num_matches 20']
% [cmd_status, cmd_out] = system(command);

match_list = readmatrix('./python scripts/match_pairs_id.txt');
img_number = length(visible_list);

% for each image, compute the number of its matching image
match_static = zeros(1, img_number);

% initial adjacent matrix according to match_list
match_adj = diag(10000 * ones(1, img_number));

for imgid = 1 : size(match_list, 1)
    match_static(match_list(imgid, 1:2)) = match_static(match_list(imgid, 1:2)) + 1;
    match_adj(match_list(imgid, 1), match_list(imgid, 2)) = match_list(imgid, 3);
    match_adj(match_list(imgid, 2), match_list(imgid, 1)) = match_list(imgid, 3);
end

%% according to the matching relationship, perform image decision and harmonization

valid_area_alpha = 0.15; % 1-0.85

parallel_flag = false;
if ~parallel_flag
    % decision process for ultraviolet images
    % for infrared and acoustic images, just set the variable 'decision_image_list' by
    % 'uv_list', 'infrared_list', and 'acoustic_list' respectively.
    decision_list = infrared_list;
    
    for imgid = 1 : img_number
        vis_img = imread(fullfile(visible_list(imgid).folder, visible_list(imgid).name), 'Background', [1 1 1]);
        wait_for_reg = find(match_adj(imgid, :) > 0);
        evaluate_criterion = zeros(length(wait_for_reg), 3);
        transfrom_mat(length(wait_for_reg)).trans = projective2d;
        
        for id = 1 : length(wait_for_reg)
            regid = wait_for_reg(id);
            reg_img = imread(fullfile(decision_list(regid).folder, decision_list(regid).name), 'Background', [1 1 1]);
            
            %===== 1. registration acccuracy
            try
                [P1, P2, tform] = cp_registration(vis_img, reg_img, 1);
            catch
                transfrom_mat(regid).trans = [];
                evaluate_criterion(id, :) = 0;
                continue;
            end
            
            %===== Projective matrix and image
            transfrom_mat(regid).trans = tform;
            reg_img_trans = imwarp(reg_img, tform, 'OutputView', imref2d(size(vis_img)));
            
            %===== valid image mask
            reg_mask = im2uint8( ones(size(reg_img, 1), size(reg_img, 2)) );
            reg_mask_trans = imwarp(reg_mask, tform, 'OutputView', imref2d(size(vis_img)));
            row_min = round(valid_area_alpha * size(vis_img, 1));
            row_max = round((1-valid_area_alpha) * size(vis_img, 1));
            col_min = round(valid_area_alpha * size(vis_img, 2));
            col_max = round((1-valid_area_alpha) * size(vis_img,2));
            
            valid_mask = reg_mask_trans(row_min:row_max, col_min:col_max) == 255;
            reg_img_trans(row_min:row_max, col_min:col_max, 1) = reg_img_trans(row_min:row_max, col_min:col_max, 1) .* uint8(valid_mask);
            reg_img_trans(row_min:row_max, col_min:col_max, 2) = reg_img_trans(row_min:row_max, col_min:col_max, 2) .* uint8(valid_mask);
            reg_img_trans(row_min:row_max, col_min:col_max, 3) = reg_img_trans(row_min:row_max, col_min:col_max, 3) .* uint8(valid_mask);
            
            %===== 1. NOEP
            edge_img1 = edge( rgb2gray( vis_img(row_min:row_max, col_min:col_max, :) ), 'Canny');
            edge_img2 = edge( rgb2gray( reg_img_trans(row_min:row_max, col_min:col_max, :) ), 'Canny');
            NOEP = sum(sum(edge_img1 & edge_img2));
            
            %===== 2. VAP
            VAP = sum(valid_mask(:));
            
            %===== 3. Entropy
            img_entropy = entropy(rgb2gray( reg_img_trans(row_min:row_max, col_min:col_max, :) ));
            
            % store the result
            evaluate_criterion(id, 1) = NOEP;
            evaluate_criterion(id, 2) = VAP;
            evaluate_criterion(id, 3) = img_entropy;
            fprintf('Image decision: %d -> %d\n%15s\t%15s\t%15s\t%15s\n%15d\t%15d\t%15d\t%15.4f\n', imgid, regid, ...
                'FeatureMatches', 'NOEP', 'VAP', 'Entropy', length(P1), evaluate_criterion(id, :));
        end
        
        % gassian normalization
        gas_std = length(wait_for_reg) - 1;
        gas_kernel = exp( -(0:1:gas_std) / 2 / gas_std^2 );
        gas_kernel = gas_kernel / sum(gas_kernel);
        [~, NOEP_sort] = sort(evaluate_criterion(:, 1), 'Descend');
        [~, VAP_sort] = sortrows([evaluate_criterion(:, 2) evaluate_criterion(:, 1)], 'Descend');
        [~, Entropy_sort] = sortrows([evaluate_criterion(:, 3) evaluate_criterion(:, 1)], 'Descend');
        norm_result = zeros(1, length(wait_for_reg));
        
        % here, w_NOEP, w_VAP, w_Entropy are normlized weights for determining the
        % priority of the metrics, default is [1, 1, 1]; optional
        % combinations like [6 3 1], [4 4 2], etc.
        w_NOEP = 1;
        w_VAP = 1;
        w_Entropy = 1;
        norm_result(NOEP_sort)    = norm_result(NOEP_sort)    + w_NOEP    * gas_kernel;
        norm_result(VAP_sort)     = norm_result(VAP_sort)     + w_VAP     * gas_kernel;
        norm_result(Entropy_sort) = norm_result(Entropy_sort) + w_Entropy * gas_kernel;
        
        % decide the priority
        [~, decision_sort] = sort(norm_result, 'Descend');
        fprintf('Decision quality: %2d = ', imgid)
        
        % stitch image data
        stitch_img_data = im2uint8(zeros(size(vis_img)));
        stitch_mask = true(size(vis_img, 1), size(vis_img, 2));
        for dataid = wait_for_reg(decision_sort)
            if dataid == wait_for_reg(decision_sort(end))
                fprintf('%2d\n', dataid);
            else
                fprintf('%2d > ', dataid);
            end
            if isempty(transfrom_mat(dataid).trans)
                continue;
            end
            ori_img = imread( fullfile(decision_list(dataid).folder, decision_list(dataid).name ), 'Background', [1 1 1]);
            trans_img = imwarp(ori_img, transfrom_mat(dataid).trans, 'OutputView', imref2d(size(vis_img)));
            
            ori_mask = im2uint8( ones(size(ori_img, 1), size(ori_img, 2)) );
            ori_mask_trans = imwarp(ori_mask, transfrom_mat(dataid).trans, 'OutputView', imref2d(size(vis_img)));
            
            [v, u] = find(stitch_mask);
            stitch_img_data(v + (u-1)*size(vis_img, 1))                         = trans_img(v + (u-1)*size(vis_img, 1));
            stitch_img_data(v + (u-1)*size(vis_img, 1)+numel(vis_img(:,:,1)))   = trans_img(v + (u-1)*size(vis_img, 1)+numel(vis_img(:,:,1)));
            stitch_img_data(v + (u-1)*size(vis_img, 1)+2*numel(vis_img(:,:,1))) = trans_img(v + (u-1)*size(vis_img, 1)+2*numel(vis_img(:,:,1)));
            
            stitch_mask( ori_mask_trans==255 ) = false;
        end
        transfrom_mat = [];
        
        [v, u] = find(stitch_mask);
        stitch_img_data(v + (u-1)*size(vis_img, 1))                         = 251;
        stitch_img_data(v + (u-1)*size(vis_img, 1)+numel(vis_img(:,:,1)))   = 155;
        stitch_img_data(v + (u-1)*size(vis_img, 1)+2*numel(vis_img(:,:,1))) = 91;
        
        figure('Name', 'Image stitching result', 'Color', 'w'),
        subplot(121), imshowpair(vis_img, stitch_img_data, 'montage');
        subplot(122), imshowpair(vis_img, stitch_img_data, 'falsecolor');
        
        % write the decision image to the destination path
        dst_img_path = '\Source multimodal virtual images\Robot\ultraviolet_decision';
        %         if ~exist(dst_img_path, 'dir')
        %             mkdir(dst_img_path);
        %         end
        %imwrite(stitch_img_data, fullfile(dst_img_path, visible_list(imgid).name));
    end
else
    % parallel version
    % for saving runtime, 'parfor' is recommended
    
    visible  = 1;
    infrared = 2;
    uv       = 3;
    acoustic = 4;
    
    parfor modality = [infrared uv acoustic]
        if modality == infrared
            decision_list = infrared_list;
            decision_path = 'infrared_decision';
        elseif modality == uv
            decision_list = uv_list;
            decision_path = 'uv_decision';
        else
            decision_list = acoustic_list;
            decision_path = 'acoustic_decision';
        end
        
        for imgid = 1 : img_number
            vis_img = imread(fullfile(visible_list(imgid).folder, visible_list(imgid).name), 'Background', [1 1 1]);
            wait_for_reg = find(match_adj(imgid, :) > 0);
            evaluate_criterion = zeros(length(wait_for_reg), 3);
            transfrom_mat(length(wait_for_reg)).trans = projective2d;
            
            for id = 1 : length(wait_for_reg)
                regid = wait_for_reg(id);
                reg_img = imread(fullfile(decision_list(regid).folder, decision_list(regid).name), 'Background', [1 1 1]);
                
                %===== 1. registration acccuracy
                try
                    [P1, P2, tform] = cp_registration(vis_img, reg_img, 1);
                catch
                    transfrom_mat(regid).trans = [];
                    evaluate_criterion(id, :) = 0;
                    continue;
                end
                
                %===== Projective matrix and image
                transfrom_mat(regid).trans = tform;
                reg_img_trans = imwarp(reg_img, tform, 'OutputView', imref2d(size(vis_img)));
                
                %===== valid image mask
                reg_mask = im2uint8( ones(size(reg_img, 1), size(reg_img, 2)) );
                reg_mask_trans = imwarp(reg_mask, tform, 'OutputView', imref2d(size(vis_img)));
                row_min = round(valid_area_alpha * size(vis_img, 1));
                row_max = round((1-valid_area_alpha) * size(vis_img, 1));
                col_min = round(valid_area_alpha * size(vis_img, 2));
                col_max = round((1-valid_area_alpha) * size(vis_img,2));
                
                valid_mask = reg_mask_trans(row_min:row_max, col_min:col_max) == 255;
                reg_img_trans(row_min:row_max, col_min:col_max, 1) = reg_img_trans(row_min:row_max, col_min:col_max, 1) .* uint8(valid_mask);
                reg_img_trans(row_min:row_max, col_min:col_max, 2) = reg_img_trans(row_min:row_max, col_min:col_max, 2) .* uint8(valid_mask);
                reg_img_trans(row_min:row_max, col_min:col_max, 3) = reg_img_trans(row_min:row_max, col_min:col_max, 3) .* uint8(valid_mask);
                
                %===== 1. NOEP
                edge_img1 = edge( rgb2gray( vis_img(row_min:row_max, col_min:col_max, :) ), 'Canny');
                edge_img2 = edge( rgb2gray( reg_img_trans(row_min:row_max, col_min:col_max, :) ), 'Canny');
                NOEP = sum(sum(edge_img1 & edge_img2));
                
                %===== 2. VAP
                VAP = sum(valid_mask(:));
                
                %===== 3. Entropy
                img_entropy = entropy(rgb2gray( reg_img_trans(row_min:row_max, col_min:col_max, :) ));
                
                % store the result
                evaluate_criterion(id, 1) = NOEP;
                evaluate_criterion(id, 2) = VAP;
                evaluate_criterion(id, 3) = img_entropy;
                fprintf('Image decision: %d -> %d\n%15s\t%15s\t%15s\t%15s\n%15d\t%15d\t%15d\t%15.4f\n', imgid, regid, ...
                    'FeatureMatches', 'NOEP', 'VAP', 'Entropy', length(P1), evaluate_criterion(id, :));
            end
            
            % gassian normalization
            gas_std = length(wait_for_reg) - 1;
            gas_kernel = exp( -(0:1:gas_std) / 2 / gas_std^2 );
            gas_kernel = gas_kernel / sum(gas_kernel);
            [~, NOEP_sort] = sort(evaluate_criterion(:, 1), 'Descend');
            [~, VAP_sort] = sortrows([evaluate_criterion(:, 2) evaluate_criterion(:, 1)], 'Descend');
            [~, Entropy_sort] = sortrows([evaluate_criterion(:, 3) evaluate_criterion(:, 1)], 'Descend');
            norm_result = zeros(1, length(wait_for_reg));
            
            % here, w_NOEP, w_VAP, w_Entropy are normlized weights for determining the
            % priority of the metrics, default is [1, 1, 1]; optional
            % combinations like [6 3 1], [4 4 2], etc.
            w_NOEP = 1;
            w_VAP = 1;
            w_Entropy = 1;
            norm_result(NOEP_sort)    = norm_result(NOEP_sort)    + w_NOEP    * gas_kernel;
            norm_result(VAP_sort)     = norm_result(VAP_sort)     + w_VAP     * gas_kernel;
            norm_result(Entropy_sort) = norm_result(Entropy_sort) + w_Entropy * gas_kernel;
            
            % decide the priority
            [~, decision_sort] = sort(norm_result, 'Descend');
            fprintf('Decision quality: %2d = ', imgid)
            
            % stitch image data
            stitch_img_data = im2uint8(zeros(size(vis_img)));
            stitch_mask = true(size(vis_img, 1), size(vis_img, 2));
            for dataid = wait_for_reg(decision_sort)
                if dataid == wait_for_reg(decision_sort(end))
                    fprintf('%2d\n', dataid);
                else
                    fprintf('%2d > ', dataid);
                end
                if isempty(transfrom_mat(dataid).trans)
                    continue;
                end
                ori_img = imread( fullfile(decision_list(dataid).folder, decision_list(dataid).name ), 'Background', [1 1 1]);
                trans_img = imwarp(ori_img, transfrom_mat(dataid).trans, 'OutputView', imref2d(size(vis_img)));
                
                ori_mask = im2uint8( ones(size(ori_img, 1), size(ori_img, 2)) );
                ori_mask_trans = imwarp(ori_mask, transfrom_mat(dataid).trans, 'OutputView', imref2d(size(vis_img)));
                
                [v, u] = find(stitch_mask);
                stitch_img_data(v + (u-1)*size(vis_img, 1))                         = trans_img(v + (u-1)*size(vis_img, 1));
                stitch_img_data(v + (u-1)*size(vis_img, 1)+numel(vis_img(:,:,1)))   = trans_img(v + (u-1)*size(vis_img, 1)+numel(vis_img(:,:,1)));
                stitch_img_data(v + (u-1)*size(vis_img, 1)+2*numel(vis_img(:,:,1))) = trans_img(v + (u-1)*size(vis_img, 1)+2*numel(vis_img(:,:,1)));
                
                stitch_mask( ori_mask_trans==255 ) = false;
            end
            transfrom_mat = [];
            
            [v, u] = find(stitch_mask);
            stitch_img_data(v + (u-1)*size(vis_img, 1))                         = 251;
            stitch_img_data(v + (u-1)*size(vis_img, 1)+numel(vis_img(:,:,1)))   = 155;
            stitch_img_data(v + (u-1)*size(vis_img, 1)+2*numel(vis_img(:,:,1))) = 91;
            
            figure('Name', 'Image stitching result', 'Color', 'w'),
            subplot(121), imshowpair(vis_img, stitch_img_data, 'montage');
            subplot(122), imshowpair(vis_img, stitch_img_data, 'falsecolor');
            
            % write the decision image to the destination path
            dst_img_path = ['\Source multimodal virtual images\Robot\' decision_path];
            %         if ~exist(dst_img_path, 'dir')
            %             mkdir(dst_img_path);
            %         end
            %imwrite(stitch_img_data, fullfile(dst_img_path, visible_list(imgid).name));
        end
    end
end
    
    
    
    
    
