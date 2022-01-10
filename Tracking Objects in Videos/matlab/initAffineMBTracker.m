function [affineMBContext] = initAffineMBTracker(img, rect)

x = rect(1); y = rect(2); w = rect(3); h = rect(4);
t = y; l = x; b = y+h-1; r = x+w-1;

T = double(img(t:b,l:r));
[d_Tx,d_Ty] = gradient(T);
d_Tx = d_Tx(:);
d_Ty = d_Ty(:); 

[xs,ys] = meshgrid(t:b,l:r);
xs = xs(:);
ys = ys(:);

J = [xs.*d_Tx, xs.*d_Ty, ys.*d_Tx, ys.*d_Ty, d_Tx, d_Ty];
H = J'*J;
affineMBContext = struct('J',J,'invH',H^(-1));

end


