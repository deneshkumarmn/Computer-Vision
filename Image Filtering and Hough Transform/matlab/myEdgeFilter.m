function [img1] = myEdgeFilter(img0, sigma)
  gaussian_filter = fspecial('gaussian', 2*ceil(3*sigma)+1, sigma);
  conv_img = myImageFilter(img0, gaussian_filter);
  x_s = fspecial('sobel')';
  img_x = myImageFilter(conv_img, x_s);
  img_x=double(img_x);
  y_s = fspecial('sobel');
  img_y = myImageFilter(conv_img, y_s);
  img_y=double(img_y);
  img_magnitude = sqrt(img_x.*img_x + img_y.*img_y); 
  angle = atan2(img_y, img_x);
  img_magnitude = padarray(img_magnitude, [1, 1], 'replicate', 'both');
  al = [-1.0*pi,  7.0*pi;  1.0*pi, -7.0*pi; 3.0*pi, -5.0*pi; -3.0*pi, 5.0*pi; ] / 8.0;
  ar = [ 1.0*pi, -7.0*pi;  3.0*pi, -5.0*pi; 5.0*pi, -3.0*pi; -1.0*pi, 7.0*pi; ] / 8.0;
  img1 = img_magnitude;
  s = size(angle);
  for j = 1:4
     if j == 1
          hpi = intersect(find(angle <= ar(j,1)), find(angle >= al(j, 1)));
          hpi_1 = union(find(angle <= ar(j, 2)), find(angle >= al(j, 2)));
     else
            hpi = intersect(find(angle <= ar(j,1)), find(angle >= al(j, 1)));
            hpi_1 = intersect(find(angle <= ar(j, 2)), find(angle >= al(j, 2)));
     end
     newHpi = union(hpi, hpi_1);
     [is, js] = ind2sub(s, newHpi);
     l = size(is);
     l = l(1);
     is = is+ 1;
     js = js + 1;
     for i = 1:l
         if j == 1
              if img_magnitude(is(i),js(i)) <= img_magnitude(is(i),js(i)-1) || img_magnitude(is(i), js(i)) <= img_magnitude(is(i),js(i)+1)
                img1(is(i), js(i)) = 0;
              end
         elseif j == 2
              if img_magnitude(is(i),js(i)) <= img_magnitude(is(i)+1,js(i)+1) || img_magnitude(is(i), js(i)) <= img_magnitude(is(i)-1,js(i)-1)
                 img1(is(i), js(i)) = 0;
              end
         elseif j == 3
              if img_magnitude(is(i),js(i)) <= img_magnitude(is(i)+1,js(i)) || img_magnitude(is(i), js(i)) <= img_magnitude(is(i)-1,js(i))
                 img1(is(i), js(i)) = 0;
              end
         else
                if img_magnitude(is(i),js(i)) <= img_magnitude(is(i)-1,js(i)+1) || img_magnitude(is(i), js(i)) <= img_magnitude(is(i)+1,js(i)-1)
                    img1(is(i), js(i)) = 0;
                end
         end
     end
  end
  img1 = img1(2:end-1, 2:end-1);
end