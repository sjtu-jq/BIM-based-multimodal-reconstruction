function colmap_pipeline(colmap_bat_path, input_project_path)
% COLMAP_PIPELINE(input_project_path)
% A MATLAB scripts for perform COLMAP SFM and MVS pipeline

% colmap_bat_path: \the path to your downloaded colmap windows
%           bat\COLMAP.bat, you need to specify your colmap.bat that can be downloaded at COLMAP
%           github page
% input_project_path: the parent folder that includes the source image
%           './images'

sfm_pipeline = true;
mvs_pipeline = false;


source_images_path = fullfile(input_project_path, 'images');

%% SFM pipeline
if sfm_pipeline
    % create folder to store the SFM results .\sparse
    sparse_path = fullfile(input_project_path, 'colmap_sparse');
    if ~exist(sparse_path, 'dir')
        mkdir(sparse_path);
    else
        fprintf('[info] Sparse folder existed: %s\n', sparse_path);
    end
    
    % create database.db
    tic;
    database_filename = fullfile(input_project_path, '\database.db');
    if ~exist(database_filename, 'file')
        create_db_command = stitchStrings(colmap_bat_path, 'database_creator', '--database_path', database_filename);
        [cmd_status, cmd_out] = system(create_db_command);
        if cmd_status
            error('[error] System executed the command: %s failed!\n system output: %s\n', create_db_command, cmd_out);
        else
            fprintf('cmd output:%s\n[info]>>>>>> Database created: %s <<<<<<<< \n', 'SUCCEEDED!', obj.Second2MSMFormat(toc));
        end
    else
        fprintf('[info] Database path existed: %s\nRuntime:%s\n', database_filename, obj.Second2MSMFormat(toc));
    end
    
    % extract feature points
    tic;
    camera_model = 'SIMPLE_RADIAL';
    extract_feature_command = stitchStrings(colmap_bat_path, 'feature_extractor', '--database_path', database_filename,...
        '--image_path', source_images_path, '--ImageReader.camera_model', camera_model,...
        '--ImageReader.single_camera_per_image', 1);
    [cmd_status, cmd_out] = system(extract_feature_command);
    if cmd_status
        error('[error] System executed the command: %s failed!\n system output: %s\n', extract_feature_command, cmd_out);
    else
        fprintf('cmd output:%s\n[info]>>>>>> Feature extracted: %s <<<<<<<< \n', 'SUCCEEDED!', obj.Second2MSMFormat(toc));
    end
    
    % match feature points
    tic;
    match_feature_command = stitchStrings(colmap_bat_path, 'exhaustive_matcher', '--database_path', database_filename);
    [cmd_status, cmd_out] = system(match_feature_command);
    if cmd_status
        error('[error] System executed the command: %s failed!\n system output: %s\n', match_feature_command, cmd_out);
    else
        fprintf('cmd output:%s\n[info]>>>>>> Feature matched: %s <<<<<<<< \n', 'SUCCEEDED!', obj.Second2MSMFormat(toc));
    end
    
    % recover camera poses
    tic;
    sfm_command = stitchStrings(colmap_bat_path, 'mapper', '--database_path', database_filename,...
        '--image_path', source_images_path, '--output_path', sparse_path);
    [cmd_status, ~] = system(sfm_command);
    if cmd_status
        sfm_command = stitchStrings(colmap_bat_path, 'mapper', '--database_path', database_filename,...
            '--image_path', source_images_path, '--output_path', sparse_path,...
            '--Mapper.multiple_models', 0);
        [cmd_status, cmd_out] = system(sfm_command);
        if cmd_status
            error('[SFM error] System executed the command: %s failed!\n system output: %s\n', sfm_command, cmd_out);
        end
    end
    fprintf('cmd output:%s\n[info]>>>>>> SFM completed: %s <<<<<<<< \n', 'SUCCEEDED!', obj.Second2MSMFormat(toc));
    move_file_command = stitchStrings('copy', [fullfile(sparse_path, '0') '\*'], sparse_path);
    [cmd_status, cmd_out] = system(move_file_command);
    if cmd_status
        error('[error] copy ./sparse/0/* files failed due to: %s', cmd_out);
    end
        
    % export the .nvm file, for MeshLab texture mapping on BIM
    tic;
    nvm_filename = fullfile(source_images_path, 'colmap_export.nvm');
    export_command = stitchStrings(colmap_bat_path, 'model_converter', '--input_path', sparse_path,...
        '--output_path', nvm_filename, '--output_type', 'NVM');
    [cmd_status, cmd_out] = system(export_command);
    if cmd_status
        error('[error] System executed the command: %s failed!\n system output: %s\n', export_command, cmd_out);
    else
        fprintf('cmd output:%s\n[info]>>>>>> NVM Model export completed: %s <<<<<<<< \n', 'SUCCEEDED!', obj.Second2MSMFormat(toc));
    end
end
%% MVS pipeline
if mvs_pipeline
    % create folder to store MVS dense results
    dense_path = fullfile(input_project_path, 'colmap_dense');
    if ~exist(dense_path, 'dir')
        mkdir(dense_path, 'dir');
    else
        fprintf('[info] Dense path existed: %s\n', dense_path);
    end
    
    % undistort images
    tic;
    undistort_command = stitchStrings(colmap_bat_path, 'image_undistorter', '--image_path', source_images_path, ...
        '--input_path', sparse_path, '--output_path', dense_path);
    [cmd_status, cmd_out] = system(undistort_command);
    if cmd_status
        error('[error] System executed the command: %s failed!\n system output: %s\n', undistort_command, cmd_out);
    else
        fprintf('cmd output:%s\n[info]>>>>>> Images undistorted: %s <<<<<<<< \n', 'SUCCEEDED!', obj.Second2MSMFormat(toc));
    end
    
    % dense match
    tic;
    stereo_command = stitchStrings(colmap_bat_path, 'patch_match_stereo', '--workspace_path', dense_path);
    [cmd_status, cmd_out] = system(stereo_command);
    if cmd_status
        error('[error] System executed the command: %s failed!\n system output: %s\n', stereo_command, cmd_out);
    else
        fprintf('cmd output:%s\n[info]>>>>>> Stereo match completed: %s <<<<<<<< \n', 'SUCCEEDED!', obj.Second2MSMFormat(toc));
    end
    
    % depth fusion
    tic;
    model_name = fullfile(dense_path, 'fused.ply');
    fusion_command = stitchStrings(colmap_bat_path, 'stereo_fusion', '--workspace_path', dense_path,...
        '--output_path', model_name);
    [cmd_status, cmd_out] = system(fusion_command);
    if cmd_status
        error('[error] System executed the command: %s failed!\n system output: %s\n', fusion_command, cmd_out);
    else
        fprintf('cmd output:%s\n[info]>>>>>> Stereo fusion completed: %s <<<<<<<< \n', 'SUCCEEDED!', obj.Second2MSMFormat(toc));
    end
end
end

function longString = stitchStrings(varargin)
stringArgs = string(varargin);
longString = strjoin(stringArgs, ' ');
end
