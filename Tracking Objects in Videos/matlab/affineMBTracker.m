function [W_out] = affineMBTracker(img, tmp, rect, W_in, context)


[m0,n0] = size(img);

x = rect(1); y = rect(2); w = rect(3); h = rect(4);
t = y; l = x; b = y+h-1; r = x+w-1;

J0 = context.J;
invH0 = context.invH;

maxIter = 30;
iter = 0;
dp = 100*ones(6,1);
thresh = 3;
patience_thresh = 0.05*thresh;

W = W_in;
while (norm(dp) > thresh) && (iter < maxIter)
    dp_prev = dp;
   
    warpedI = warpH(img,W,[m0,n0],0);
    I = double(warpedI(t:b,l:r));
    T = double(tmp);
    E = (I-T);
    dp = invH0*J0'*E(:);
    Wdp = Warp_with(dp);
    W = W*inv(Wdp);    
    
    if norm(dp-dp_prev) > patience_thresh
        iter = 0;
    end
    
    iter = iter+1;
    fprintf('iter# %d\n',iter);
end

W_out = W;
end


