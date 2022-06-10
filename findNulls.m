function nullIdcs = findNulls(P, varargin)
% Function returns the indices correponding to the nulls of the pattern P.
% By default, two closest-to-the-main-lobe nulls will be found. To find 
% more than two nulls, give the corresponding number as the second 
% parameter at the function input.
switch nargin
    case 1
        nrOfNulls = 2;
    otherwise
        nrOfNulls = varargin{1};
end

[~, LOC] = findpeaks(-P); % Flip the pattern
[~, idxMax] = max(P);
[~, I] = sort(abs(idxMax - LOC));
nullIdcs = sort(LOC(I(1:nrOfNulls))); % Take two the closest points
end%function