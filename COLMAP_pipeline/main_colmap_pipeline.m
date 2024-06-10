% COLMAP reconstruction command line version
% ref: https://colmap.github.io/cli.html

close all force; clear; clc;

%% initial COLMAP project path

% .\Project\
%     |
%     |__ .\images

colmap_bat_path = '\absolute-path-to-your\COLMAP-3.8-windows-cuda\COLMAP.bat';
project_path = '\absolute-path-to-your\Project\';

colmap_pipeline(dst_project_path, colmap_project_path);







