clear
clc
x_c = 240;
y_c = 192;
width = 210;
height = 170;
l = x_c-floor(width/2);
t = y_c-floor(height/2);
track = [l t width height];

vid = VideoWriter('/Users/deneshkumar/Documents/MATLAB/results');
open(vid);
figure;
prev_frame = imread('/Users/deneshkumar/Documents/MATLAB/data/car/frame0020.jpg');
temp = prev_frame(t:(t+height-1),l:(l+width-1));  


Win = Warp_with(zeros(6,1));
contx = initAffineMBTracker(prev_frame, track);


newtracker = track;
for i = 21:280
    img_dir = sprintf('/Users/deneshkumar/Documents/MATLAB/data/car/frame%04d.jpg', i);
    if (~exist(img_dir,'file'))
        continue;
    end
    im = imread(img_dir);
    Wout = affineMBTracker(im, temp, track, Win, contx);

    
    new_c = Wout*[x_c;y_c;1];
    newtracker = [round(new_c(1)-floor(width/2)), round(new_c(2)-floor(height/2)), width, height];


    clf;
    hold on;
    imshow(im);   
    rectangle('Position', newtracker, 'EdgeColor', [1 1 0]);
    title(num2str(i));
    drawnow;

    F = getframe;
    writeVideo(vid,F);

end
close(vid);
