function [row_ind, col_ind] = munkres(costMat,euclidean_dist_thresh)
    % Wrapper for MATLAB's built-in assignment problem solver
    % If you have the Optimization Toolbox, you can use matchpairs:
    % [pairs, cost] = matchpairs(costMat, inf);
    % row_ind = pairs(:,1);
    % col_ind = pairs(:,2);
    % Otherwise, you can use this simple implementation or download from File Exchange.
    
    % Here is a simple fallback using matchpairs if available:
    if exist('matchpairs', 'file')
        [pairs, ~] = matchpairs(costMat, euclidean_dist_thresh);
        row_ind = zeros(size(costMat,1),1);
        col_ind = zeros(size(costMat,1),1);
        row_ind = pairs(:,1);
        col_ind = pairs(:,2);
    else
        % If matchpairs not available, assign zeros (no assignment)
        row_ind = zeros(size(costMat,1),1);
        col_ind = zeros(size(costMat,1),1);
        warning('matchpairs function not found. Assignments will be empty.');
    end
end
