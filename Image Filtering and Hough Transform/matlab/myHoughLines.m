function [rhos, thetas] = myHoughLines(H, nLines)
 nLines=50;
 d_H = imdilate(H, strel('square', 5));
 mask = (d_H == H);
 H = H .* mask;
 [~, sInd] = sort(H(:), 'descend');
 mInd = sInd(1:nLines);
 s = size(H);
 [rhos, thetas] = ind2sub(s, mInd);
end