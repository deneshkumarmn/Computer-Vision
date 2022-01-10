function [img1] = myImageFilter(img0, h)
 x=flip(h,1)
 f_x=flip(x,2)
 [r,t] = size(img0);
 [a,b] = size(f_x);
 s_pad = (a-1)/2;
 padded_img = padarray(img0, [s_pad s_pad])
 padded_img = double(padded_img);
 img = zeros(r-a+1,t-b+1);

 for i = 1:r

  for j = 1:t
    
    img(i,j) = sum(sum(padded_img(i:i+a-1,j:j+b-1).*(f_x)));
    
  end

 end
 img1=img;

end