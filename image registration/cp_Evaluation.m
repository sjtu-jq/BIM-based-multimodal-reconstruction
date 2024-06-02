function [evaluation, correctindex] = cp_Evaluation(p1, p2, refaffmat,corner12)
%CP_ENTROPYEVALUATION caculate the entropy, precision and RMSE of the registration results
% the relative value of the entropy is useful in identifying detector 
% spatial distribution behaviors such as non-random clustering of keypoint detections

number = length(p1);
bp = zeros(number,1);
for i = 1:number
    bp(i) = sum( exp( -((p1(:,1) - p1(i,1)).^2 + (p1(:,2) - p1(i,2)).^2) / (number.^2/2) ) );
end
bp = bp ./ sum(bp(:));
evaluation = cell(2,1);
evaluation{1} = {'Entropy' 'correct' 'inaccurate' 'fulse' 'Precision' 'Recall' 'RMSE' 'F1-Score'};
evaluation{2}(1) = sum(-bp .* log2(bp));
if nargin == 1
    Criteria = evaluation{1}
    Metrics = evaluation{2}
    correctindex = 1:1:number;
    return;
end
correctDistance = 3.5; inaccurateDistance = 2 * correctDistance;
if nargin == 4
    p2ref = [p1 ones(number,1)] * refaffmat;
    p2ref = p2ref(:,1:2) ./ p2ref(:,3);
    dist = sqrt( (p2ref(:,1) - p2(:,1)).^2 + (p2ref(:,2) - p2(:,2)).^2 );
    [corr,~] = find( dist <= correctDistance );
    [inacc,~] = find( correctDistance < dist & dist <= inaccurateDistance );
    correctindex = [corr; inacc];
    correspondences = 0;
    [cor1,~] = find(corner12(:,1) == 0);
    cor1_recall = [corner12(1:cor1-1,:) ones(cor1-1,1)] * refaffmat;
    cor1_recall = cor1_recall(:,1:2) ./ cor1_recall(:,3);
    for i = 1:cor1-1
        delta_cor = corner12(cor1+1:end,:) - cor1_recall(i,:);
        delta_cor = sqrt( delta_cor(:,1).^2 + delta_cor(:,2).^2 );
        [~,indc] = sort(delta_cor);
        if delta_cor(indc(1)) <= inaccurateDistance
            correspondences = correspondences + 1;
            corner12(cor1+indc(1),:) = [-10 10];
        end
    end
    correctmatches = length(corr);
    inaccuratematches = length(inacc);
    fulsematches = number - correctmatches - inaccuratematches;
    precision = 1 - fulsematches / number;
    recall = correctmatches / correspondences;
    rmse = sqrt(  sum(dist.^2) / number);
    F1score = 2*precision*recall/(precision+recall);
    evaluation{2}(2:8)= [correctmatches, inaccuratematches, fulsematches, precision,...
                        recall, rmse, F1score]; 
    Criteria = evaluation{1}
    Metrics = evaluation{2}
elseif nargin ~=1 && nargin~=5
    error('Input error!');
end


