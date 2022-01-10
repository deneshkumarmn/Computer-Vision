function [u, v] = LucasKanade(It, It1, rect)
[m0,n0] = size(It);

xi = rect(1); y = rect(2); w = rect(3); h = rect(4);
t = y; l = xi; b = y+h-1; r = xi+w-1;

p_init = zeros(6,1);


dp = ones(6,1)*100; 
thresholed = 1.5;
max_iter = 30;
iterations = 0;
p = p_init;

while (norm(dp) > thresholed) && (iterations < max_iter)
    W = Warp_with(p);
    
    warpedIt1 = warpH(It1,W,[m0,n0],0);

   
    I = double(warpedIt1(t:b,l:r));
    T = double(It(t:b,l:r));
    E = (T-I);

   
    [dIt1x,dIt1y] = gradient(I);
    dIt1x = dIt1x(:); 
    dIt1y = dIt1y(:); 

   
    [xs,ys] = meshgrid(t:b,l:r);
    xs = xs(:); 
    ys = ys(:); 
    
    J = [xs.*dIt1x, xs.*dIt1y, ys.*dIt1x, ys.*dIt1y, dIt1x, dIt1y]; % m by 6
    
    H = J'*J; 
    dp = H\(J'*E(:)); 
    p = p - dp;
    
    iterations = iterations+1;
    fprintf('iter# %d\n',iterations);
end

u = p(5);
v = p(6);

end
