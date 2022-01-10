function [cord] = three2d(u,v,w, vue2)
  
        wc_mat = [u;v;w; 1];                               
        new_temp = (vue2.Kmat * vue2.Pmat) * wc_mat;
        cord = [new_temp(1)/new_temp(3); new_temp(2)/new_temp(3); 1];
        
end
