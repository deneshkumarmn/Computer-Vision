function [H, rhoScale, thetaScale] = myHoughTransform(Im, threshold, rhoRes, thetaRes)
    Im = Im';
    [l, m] = size(Im);
    m_rho = ceil(norm([l, m])/rhoRes);
    rho_num = m_rho + 1; 
    theta_num = 2.0*pi / thetaRes;
    
    i = 1:theta_num;
    thetaScale = (thetaRes * (i-1));

    rhoScale = (0:rhoRes:(rhoRes*m_rho));
    rho_bin = [rhoScale - 0.5*rhoRes, rhoRes*m_rho + 0.5*rhoRes];

    pInd = find(Im > threshold);
    s = size(Im);
    [pSubx, pSuby] = ind2sub(s, pInd);
    
    pSubx = pSubx - 1;
    pSuby = pSuby - 1;

    H = zeros([rho_num, theta_num]);

    thetax = pSubx * cos(thetaScale); 
    thetay = pSuby * sin(thetaScale);
    rho = thetax + thetay;

    for i=1:theta_num
        H(:,i) = histcounts(rho(:, i), rho_bin)';
    end
 
end
