clear
clc

tracker = [160, 120, 300,260]  ;
video = VideoWriter('/Users/deneshkumar/Documents/MATLAB/results');
open(video);

width = abs(tracker(1)-tracker(3));
height = abs(tracker(2)-tracker(4));

figure;

prev_frame = imread('/Users/deneshkumar/Documents/MATLAB/data/car/frame0020.jpg');

for i = 20:279
    prev_frame = imread(sprintf('/Users/deneshkumar/Documents/MATLAB/data/car/frame%04d.jpg', i));

    new_frame = imread(sprintf('/Users/deneshkumar/Documents/MATLAB/data/car/frame%04d.jpg', i+1));
    
    imshow(prev_frame);
    hold on;
    rectangle('Position',[tracker(1),tracker(2),width,height], 'LineWidth',3, 'EdgeColor', 'g');
    hold off;
    pause(0.1);
    
    Frame = getframe;
    writeVideo(video,Frame);
    

    [u,v] = LucasKanade(prev_frame,new_frame,tracker);
    
    tracker(1) = round(tracker(1)+u);
    tracker(2) = round(tracker(2)+v);
    tracker(3) = round(tracker(3)+u);
    tracker(4) = round(tracker(4)+v);

   
end

close(video)