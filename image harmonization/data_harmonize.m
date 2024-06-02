function harmonized_data_2 = data_harmonize(ref_data_1, adjust_data_2, ref_pt_1, adjust_pt_2, alpha_radius, show_flag, objective_model)
% ref_data_1 is reference data; adjust_data_2 is to be harmonized data;
% ref_pt_1 and adjust_pt_2 is matching points of ref_data_1 and
% adjust_data_2 detected by CAO-C2F method.
    
    % generate polygon area to calculate the CDF (Cumulative
    % Distribution Function)
    
    if nargin == 4
        alpha_radius = size(ref_data_1, 2)/3;
        show_flag = 'show';
        objective_model = 'L2';
    end
    
    polygon_1 = alphaShape(double(ref_pt_1), alpha_radius);
    polygon_2 = alphaShape(double(adjust_pt_2), alpha_radius);
    
    [all_row_ind_1, all_col_ind_1] = meshgrid(1:1:size(ref_data_1, 1), 1:1:size(ref_data_1, 2));
    [all_row_ind_2, all_col_ind_2] = meshgrid(1:1:size(adjust_data_2, 1), 1:1:size(adjust_data_2, 2));
    
    valid_ind_1 = inShape(polygon_1, all_row_ind_1, all_col_ind_1);
    valid_ind_2 = inShape(polygon_2, all_row_ind_2, all_col_ind_2);
    
    overlap_area_1 = false(size(ref_data_1,1),size(ref_data_1,2));
    overlap_area_2 = false(size(adjust_data_2,1),size(adjust_data_2,2));

    overlap_area_1(valid_ind_1) = true;
    overlap_area_2(valid_ind_2) = true;
    
    if strcmpi(show_flag, 'show')
        figure('Name', 'Image Registration Results', 'Color' ,'w'),
        subplot(2, 2, [1 2]), showMatchedFeatures(ref_data_1, adjust_data_2, ref_pt_1, adjust_pt_2, 'montage');
        subplot(2, 2, 3), imshow(ref_data_1); hold on; plot(polygon_1);
        subplot(2, 2, 4), imshow(adjust_data_2); hold on; plot(polygon_2);
    end

    if size(ref_data_1, 3) > 1 && size(adjust_data_2, 3) > 1
        data_format = 'image';
    elseif size(ref_data_1, 3) == 1 && size(adjust_data_2, 3) == 1
        data_format = 'temperature';
    else
        error('Wrong input data size format!');
    end
    
    if strcmpi(data_format, 'image')
        data_1_r = ref_data_1(:,:,1);
        data_1_r_overlap = data_1_r(overlap_area_1);
        
        data_1_g = ref_data_1(:,:,2);
        data_1_g_overlap = data_1_g(overlap_area_1);
        
        data_1_b = ref_data_1(:,:,3);
        data_1_b_overlap = data_1_b(overlap_area_1);
        
        data_2_r = adjust_data_2(:,:,1);
        data_2_r_overlap = data_2_r(overlap_area_2);
        
        data_2_g = adjust_data_2(:,:,2);
        data_2_g_overlap = data_2_g(overlap_area_2);
        
        data_2_b = adjust_data_2(:,:,3);
        data_2_b_overlap = data_2_b(overlap_area_2);
        
        % adjust each color channel
        harmonize_index_r = harmonize_image_2(data_1_r_overlap, data_2_r_overlap, show_flag, objective_model);
        harmonize_index_g = harmonize_image_2(data_1_g_overlap, data_2_g_overlap, show_flag, objective_model);
        harmonize_index_b = harmonize_image_2(data_1_b_overlap, data_2_b_overlap, show_flag, objective_model);

        harmonized_data_2 = adjust_data_2;
        harmonized_data_2(:,:,1) = harmonize_index_r(data_2_r+1);
        harmonized_data_2(:,:,2) = harmonize_index_g(data_2_g+1);
        harmonized_data_2(:,:,3) = harmonize_index_b(data_2_b+1);
        
    elseif strcmpi(data_format, 'temperature')
        data_1 = ref_data_1(overlap_area_1);
        data_2 = adjust_data_2(overlap_area_2);
        
        [scale_factor, trans_factor] = harmonize_temp(data_1, data_2, 'show');
        harmonized_data_2 = scale_factor*adjust_data_2 + trans_factor;
    end

end

% multi-stage linear optimization
function new_color_index = harmonize_image_2(input_ref_channel, input_har_channel, show_flag, objective_model)
    if nargin == 2
        show_flag = 'dontshow';
        objective_model = 'L2';
    end
    
    color_resulotion = 1;
    hist_ref = histcounts(input_ref_channel, (0-color_resulotion/2) : color_resulotion : (255+color_resulotion/2) );
    hist_har = histcounts(input_har_channel, (0-color_resulotion/2) : color_resulotion : (255+color_resulotion/2) );
    
    cdf_ref = cumsum(hist_ref) / numel(input_ref_channel);
    cdf_har = cumsum(hist_har) / numel(input_har_channel);
  
    % quantile
    quantile_level = 10;
    quantile_vec = 1/quantile_level : 1/quantile_level : 1;

    sample_ref = interp1(no_repeat([0 cdf_ref]), -1:1:255, quantile_vec, 'linear', 'extrap');
    sample_har = interp1(no_repeat([0 cdf_har]), -1:1:255, quantile_vec, 'linear', 'extrap');
    
    % multi-stage L2 optimization
    translation_factor = zeros(1, quantile_level-1);
    scale_factor = zeros(1, quantile_level-1);
    fitting_har = sample_har;
    for iQuantile = 1 : (quantile_level-1)
        B = sample_ref(iQuantile : iQuantile+1)';
        A = [sample_har(iQuantile : iQuantile+1)' ones(2, 1)];
        if strcmpi(objective_model, 'L2')
            X = (A'*A) \ (A'*B);
        end
        if strcmpi(objective_model, 'Linf')
            lower_limit = [-255, -255, -255]';
            upper_limit = [255, 255, 255]';
            options = optimoptions('linprog', 'Display', 'off');
            X = linprog([0; 0; 1], [A, -ones(size(A,1),1); -A, -ones(size(A,1),1)], [B; -B], [], [], lower_limit, upper_limit, options);
        end
        translation_factor(iQuantile) = X(1);
        scale_factor(iQuantile) = X(2);
        fprintf('[%2d]: scale_factor = %.2f \t translation_factor = %.2f\n', iQuantile, translation_factor(iQuantile), scale_factor(iQuantile));
        fitting_har(iQuantile) = fitting_har(iQuantile) * translation_factor(iQuantile) + scale_factor(iQuantile);
    end
    fitting_har(quantile_level) = fitting_har(quantile_level) * translation_factor(quantile_level-1) + scale_factor(quantile_level-1);
    
    fitting_x_axes = 0:1:255;
    new_color_index = fitting_x_axes;
    for iColor = 0:1:255
        % judge the origin color intensity stage in the origin CDF curve
        whichStage = find(iColor < sample_har, 1);
        if isempty(whichStage)
            new_color_index(iColor+1) = iColor * translation_factor(quantile_level-1) + scale_factor(quantile_level-1);
        elseif whichStage <= 2
            new_color_index(iColor+1) = iColor * translation_factor(1) + scale_factor(1);
        else
            new_color_index(iColor+1) = iColor * translation_factor(whichStage-1) + scale_factor(whichStage-1);
        end
    end
    
    % show the origin CDF curves and the harmonizaed CDF curves
    if strcmpi(show_flag, 'show') 
        line_width = 0.6;
        point_size = 8;
        figure('Name', 'Fitting CDF', 'Color', 'w'),
        fig = gcf;
        fig.Units = 'centimeters';
        fig.Position = [fig.Position(1) fig.Position(2) 6.0 3.0];

        subplot(131),
        % without quantiles point on CDF curves
        plot(0:255, cdf_ref, 'Color', 'g', 'LineWidth', line_width); hold on;
        plot(0:255, cdf_har, 'Color', 'b', 'LineWidth', line_width);
        hold off;
        ax1 = gca;
        ax1.XTickLabel = {}; ax1.YTickLabel = {};
        ax1.XColor = 'none'; ax1.YColor = 'none';
        
        % with quantiles point on CDF curves
        subplot(132),
        plot(0:255, cdf_ref, 'Color', 'g', 'LineWidth', line_width); hold on;
            scatter(sample_ref, quantile_vec, point_size, 'g', 'filled'); xlim([-1,255]), ylim([0,1]);
        plot(0:255, cdf_har, 'Color', 'b', 'LineWidth', line_width);
            scatter(sample_har, quantile_vec, point_size, 'b', 'filled'); xlim([-1,255]), ylim([0,1]); 
        hold off;
        ax2 = gca;
        ax2.XTickLabel = {}; ax2.YTickLabel = {};
        ax2.XColor = 'none'; ax2.YColor = 'none';
        
        subplot(133),
        plot(0:255, cdf_ref, 'Color', 'g', 'LineWidth', line_width); hold on;
            scatter(sample_ref, quantile_vec, point_size, 'g', 'filled'); xlim([-1,255]), ylim([0,1]);
        plot( new_color_index, cdf_har, 'Color', 'b', 'LineWidth', line_width);
            scatter(fitting_har, quantile_vec, point_size, 'b', 'filled'); xlim([-1,255]), ylim([0,1]);
        hold off;
        ax3 = gca;
        ax3.XTickLabel = {}; ax3.YTickLabel = {};
        ax3.XColor = 'none'; ax3.YColor = 'none';
    end
    % exportgraphics(ax1, '..\ThirdSubmission-TIM\Figures\materials\cdf_curve.png', 'Resolution', 600);
    % exportgraphics(ax2, '..\ThirdSubmission-TIM\Figures\materials\cdf_quantiles.png', 'Resolution', 600);
    % exportgraphics(ax3, '..\ThirdSubmission-TIM\Figures\materials\cdf_quantiles_harmonize.png', 'Resolution', 600);
        
    new_color_index = uint8(new_color_index);
end

% [For other applications, not presented in my paper] harmonize single channel data for infrared temperature
function [scale_factor, translation_factor] = harmonize_temp(input_ref_channel, input_har_channel, show_flag)
    if nargin == 2
        show_flag = 'dontshow';
    end
    
    temp_resulotion = 0.3;
    bin_min_edge = min( min(input_ref_channel(:)), min(input_har_channel(:)) ) - temp_resulotion/2;
    bin_max_edge = max( max(input_ref_channel(:)), max(input_har_channel(:)) ) + temp_resulotion/2;
    
    hist_ref = histcounts(input_ref_channel, bin_min_edge:temp_resulotion:bin_max_edge);
    hist_har = histcounts(input_har_channel, bin_min_edge:temp_resulotion:bin_max_edge);
    
    cdf_ref = cumsum(hist_ref) / numel(input_ref_channel);
    cdf_har = cumsum(hist_har) / numel(input_har_channel);
  
    % quantile
    quantile_level = 10;
    quantile_vec = 0.1 : 0.9/(quantile_level-1) : 1;

    sample_ref = interp1(no_repeat([0 cdf_ref]), bin_min_edge:temp_resulotion:bin_max_edge, quantile_vec, 'linear', 'extrap');
    sample_har = interp1(no_repeat([0 cdf_har]), bin_min_edge:temp_resulotion:bin_max_edge, quantile_vec, 'linear', 'extrap');
    
    B = sample_ref';
    A = [sample_har' ones(length(sample_har), 1)];
    
    X = (A'*A) \ (A'*B);
    
    scale_factor = X(1);
    translation_factor = X(2);
    
    fprintf('*******[Harmonize temperature]*******\n scale_factor = %.2f\ttranslation_factor = %.2f\n', scale_factor, translation_factor);
    
    fitting_har = A * X;
       
    if strcmpi(show_flag, 'show') 
        line_width = 1.5;
        point_size = 30;
        figure('Name', 'Fitting CDF', 'Color', 'w'),
        subplot(121),
        plot(bin_min_edge:temp_resulotion:bin_max_edge, [0 cdf_ref], 'Color', 'g', 'LineWidth', line_width); hold on;
            scatter(sample_ref, quantile_vec, point_size, 'g', 'filled'); xlim([bin_min_edge,bin_max_edge]), ylim([-0.1,1.1]);
        plot(bin_min_edge:temp_resulotion:bin_max_edge, [0 cdf_har], 'Color', 'b', 'LineWidth', line_width); hold on;
            scatter(sample_har, quantile_vec, point_size, 'b', 'filled'); xlim([bin_min_edge,bin_max_edge]), ylim([-0.1,1.1]);
        title('Origin CDF curve');
        
        subplot(122),
        plot(bin_min_edge:temp_resulotion:bin_max_edge, [0 cdf_ref], 'Color', 'g', 'LineWidth', line_width); hold on;
            scatter(sample_ref, quantile_vec, point_size, 'g', 'filled'); xlim([bin_min_edge,bin_max_edge]), ylim([-0.1,1.1]);
        plot( (bin_min_edge:temp_resulotion:bin_max_edge)*X(1)+X(2), [0 cdf_har], 'Color', 'b', 'LineWidth', line_width); hold on;
            scatter(fitting_har, quantile_vec, point_size, 'b', 'filled'); xlim([bin_min_edge,bin_max_edge]), ylim([-0.1,1.1]);
        title('Fitting CDF curve');
    end
end

function output_vec = no_repeat(input_vec)
    output_vec = input_vec;
    for ind = 1 : length(output_vec)-1
        if round(output_vec(ind),4) >= round(output_vec(ind+1),4)
            output_vec(ind+1) = output_vec(ind) + 5e-6;
        end
    end
end