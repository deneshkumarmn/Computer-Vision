clc
clear

profile on  % For Calculating Algorithm Efficiency

% Loading the mp4 files from Vue 2 and Vue 4
f_v2_mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
vue2_reader = VideoReader(f_v2_mp4);
f_v4_mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';
vue4_reader = VideoReader(f_v4_mp4);

%Loading points
load('vue2CalibInfo.mat');
load('vue4CalibInfo.mat');
load('Subject4-Session3-Take4_mocapJoints.mat');


% Initializing 12 joints with zeros from frame 1 to frame 26214 ,
% so the dimension of the matrix is 1x26214 for all the joints
joint1 = zeros(1,26214);
joint2 = zeros(1,26214);
joint3 = zeros(1,26214);
joint4 = zeros(1,26214);
joint5 = zeros(1,26214);
joint6 = zeros(1,26214);
joint7 = zeros(1,26214);
joint8 = zeros(1,26214);
joint9 = zeros(1,26214);
joint10 = zeros(1,26214);
joint11 = zeros(1,26214);
joint12 = zeros(1,26214);
avg_error = zeros(1,26214); 

% Intialization
mocapFnum = 1; 
i = 1;

% Looping for all frames
for mocapFnum = 1:7777
            % Reading the 3D joint data for all 12 joints which is in world
            u = mocapJoints(mocapFnum,:,1);  % U world coordinates
            v = mocapJoints(mocapFnum,:,2); % V world coordinates
            w = mocapJoints(mocapFnum,:,3); % W world coordinates
            confidence = mocapJoints(mocapFnum,:,4); % for selection

            % Reading Camera Parameters for both Vue 2 and Vue 4
            p_2 = vue2.Pmat(:,4);
            camera_location_2 = -(vue2.Rmat.')*p_2;  % Checking Vue 2 position
            p_4 = vue4.Pmat(:,4);
            camera_location_4 = -(vue4.Rmat.')*p_4;  % Checking Vue 4 position

            pmatrix_v2 = vue2.Rmat*camera_location_2;  % Checking Pmatrix for Vue 2 
            pmatrix_v4 = vue4.Rmat*camera_location_4;   % Checking Pmatrix for Vue 4


            %This step is to project 3D world co-ordinates into 2D image pixel locations
            P_j_2 = zeros(3,12);
            P_j_4 = zeros(3,12);
            for i = 1:12
                P_j_2(:,i) = three2d(u(i),v(i),w(i), vue2);     % transformation of 3d world co-ordinates to 2d image pixel coordinates for vue2
                P_j_4(:,i) = three2d(u(i),v(i),w(i), vue4);    % transformation of 3d world co-ordinates to 2d image pixel coordinates for vue4
            end

            % Next Step is Triangulation to get back the 3d world
            % co-ordinates from the 2d image co-ordinates
       

            c2_c1 = camera_location_4-camera_location_2; % Calculate the difference between Vue 4 Camera Position and Vue 2 Camera Position



            p = zeros(3,12);       
            for i = 1:12
                sol2 = vue2.Kmat\P_j_2(:,i);     
                sol4 = vue4.Kmat\P_j_4(:,i);
                vue2_View_Ray = (vue2.Rmat.')* sol2;   
                vue4_View_Ray = (vue4.Rmat.') * sol4;

                final_sol = cross(vue2_View_Ray,vue4_View_Ray)/abs(cross(vue2_View_Ray,vue4_View_Ray));


                Amat(:,1) = vue2_View_Ray;     
                Amat(:,2) = -vue4_View_Ray;    
                Amat(:,3) = final_sol(:,3);    

                solution = Amat\(c2_c1);      
                p1 = camera_location_2 + (solution(1)*vue2_View_Ray); 
                p2 = camera_location_4 + (solution(2)*vue4_View_Ray);

                p(:,i) = (p1 + p2)/2;    % Finally obtained back the world co-ordinates
            end


            % Measure error between triangulation 3d world co-ordinates and original 3D points
            error_average = 0;
            error = zeros(1,12);  

            for i = 1:12
                error(i) = sqrt((u(i) - p(1,i)).^2 + (v(i) - p(2,i)).^2 + (w(i) - p(3,i)).^2);   
                error_average = error(i) + error_average;
            end

            avg_error(mocapFnum) = error_average/12;
            joint1(mocapFnum) = error(1); 
            joint2(mocapFnum) = error(2);
            joint3(mocapFnum) = error(3);
            joint4(mocapFnum) = error(4);
            joint5(mocapFnum) = error(5);
            joint6(mocapFnum) = error(6);
            joint7(mocapFnum) = error(7);
            joint8(mocapFnum) = error(8);
            joint9(mocapFnum) = error(9);
            joint10(mocapFnum) = error(10);
            joint11(mocapFnum) = error(11);
            joint12(mocapFnum) = error(12);
            
end

% Minimum error for 12 joints

minerrorj1 = min(joint1(:));
minerrorj2 = min(joint2(:));
minerrorj3 = min(joint3(:));
minerrorj4 = min(joint4(:));
minerrorj5 = min(joint5(:));
minerrorj6 = min(joint6(:));
minerrorj7 = min(joint7(:));
minerrorj8 = min(joint8(:));
minerrorj9 = min(joint9(:));
minerrorj10 = min(joint10(:));
minerrorj11 = min(joint11(:));
minerrorj12 = min(joint12(:));

% maximum error for 12 joints
maxerrorj1 = max(joint1(:));
maxerrorj2 = max(joint2(:));
maxerrorj3 = max(joint3(:));
maxerrorj4 = max(joint4(:));
maxerrorj5 = max(joint5(:));
maxerrorj6 = max(joint6(:));
maxerrorj7 = max(joint7(:));
maxerrorj8 = max(joint8(:));
maxerrorj9 = max(joint9(:));
maxerrorj10 = max(joint10(:));
maxerrorj11 = max(joint11(:));
maxerrorj12 = max(joint12(:));

% mean error for 12 joints
meanerrorj1 = mean(joint1(:));
meanerrorj2 = mean(joint2(:));
meanerrorj3 = mean(joint3(:));
meanerrorj4 = mean(joint4(:));
meanerrorj5 = mean(joint5(:));
meanerrorj6 = mean(joint6(:));
meanerrorj7 = mean(joint7(:));
meanerrorj8 = mean(joint8(:));
meanerrorj9 = mean(joint9(:));
meanerrorj10 = mean(joint10(:));
meanerrorj11 = mean(joint11(:));
meanerrorj12 = mean(joint12(:));

% Median error for 12 joints
mederrorj1 = median(joint1(:));
mederrorj2 = median(joint2(:));
mederrorj3 = median(joint3(:));
mederrorj4 = median(joint4(:));
mederrorj5 = median(joint5(:));
mederrorj6 = median(joint6(:));
mederrorj7 = median(joint7(:));
mederrorj8 = median(joint8(:));
mederrorj9 = median(joint9(:));
mederrorj10 = median(joint10(:));
mederrorj11 = median(joint11(:));
mederrorj12 = median(joint12(:));

% Std Deviation of error for 12 joints 
stderrorj1 = std(joint1(:));
stderrorj2 = std(joint2(:));
stderrorj3 = std(joint3(:));
stderrorj4 = std(joint4(:));
stderrorj5 = std(joint5(:));
stderrorj6 = std(joint6(:));
stderrorj7 = std(joint7(:));
stderrorj8 = std(joint8(:));
stderrorj9 = std(joint9(:));
stderrorj10 = std(joint10(:));
stderrorj11 = std(joint11(:));
stderrorj12 = std(joint12(:));



mocapFnum1 = 7777;
x1 = mocapJoints(mocapFnum1,:,1);  
y1 = mocapJoints(mocapFnum1,:,2); 
z1 = mocapJoints(mocapFnum1,:,3); 
confidence = mocapJoints(mocapFnum1,:,4); 
P_j_2 = zeros(3,12);
P_j_4 = zeros(3,12);
for i = 1:12
    P_j_2(:,i) = three2d(x1(i),y1(i),z1(i), vue2); 
    P_j_4(:,i) = three2d(x1(i),y1(i),z1(i), vue4);
end

% Plotting for Camera vue2
figure(2)

vue2_reader.CurrentTime = (mocapFnum1-1)*(50/100)/vue2_reader.FrameRate;
vid2Frame = readFrame(vue2_reader);
imshow(vid2Frame);

title('Vue2');
hold on
for i = 1:12
    plot(P_j_2(1,i),P_j_2(2,i), 'r+', 'MarkerSize',5, 'LineWidth', 2);  
end

%plot
plot([P_j_2(1,1) P_j_2(1,4)], [P_j_2(2,1) P_j_2(2,4)], 'c');
plot([P_j_2(1,1) P_j_2(1,2)], [P_j_2(2,1) P_j_2(2,2)], 'c');
plot([P_j_2(1,2) P_j_2(1,3)], [P_j_2(2,2) P_j_2(2,3)], 'c');
plot([P_j_2(1,4) P_j_2(1,5)], [P_j_2(2,4) P_j_2(2,5)], 'c');
plot([P_j_2(1,5) P_j_2(1,6)], [P_j_2(2,5) P_j_2(2,6)], 'c');
plot([P_j_2(1,7) P_j_2(1,10)], [P_j_2(2,7) P_j_2(2,10)], 'c');
plot([P_j_2(1,7) P_j_2(1,8)], [P_j_2(2,7) P_j_2(2,8)], 'c');
plot([P_j_2(1,8) P_j_2(1,9)], [P_j_2(2,8) P_j_2(2,9)], 'c');
plot([P_j_2(1,10) P_j_2(1,11)], [P_j_2(2,10) P_j_2(2,11)], 'c');
plot([P_j_2(1,11) P_j_2(1,12)], [P_j_2(2,11) P_j_2(2,12)], 'c');
plot([P_j_2(1,1) P_j_2(1,7)], [P_j_2(2,1) P_j_2(2,7)], 'c');
plot([P_j_2(1,4) P_j_2(1,10)], [P_j_2(2,4) P_j_2(2,10)], 'c');

%Plotting Avg Error

figure(3);

i = 1:26214;
plot(i, avg_error, 'ob', 'MarkerSize',0.5, 'LineWidth', 0.5);


title('errorAvg(mocapFnum)');
xlabel('mocapFnum');
ylabel('Avg Error');


% Plotting for Camera vue 4

figure(4)
vue4_reader.CurrentTime = (mocapFnum1-1)*(50/100)/vue4_reader.FrameRate;
vid4Frame = readFrame(vue4_reader);
imshow(vid4Frame);
title('Vue4 at frame');
hold on
for i = 1:12
    plot(P_j_4(1,i), P_j_4(2,i), 'r+', 'MarkerSize',5, 'LineWidth', 2);
end 
plot([P_j_4(1,1) P_j_4(1,4)], [P_j_4(2,1) P_j_4(2,4)], 'c');
plot([P_j_4(1,1) P_j_4(1,2)], [P_j_4(2,1) P_j_4(2,2)], 'c');
plot([P_j_4(1,2) P_j_4(1,3)], [P_j_4(2,2) P_j_4(2,3)], 'c');
plot([P_j_4(1,4) P_j_4(1,5)], [P_j_4(2,4) P_j_4(2,5)], 'c');
plot([P_j_4(1,5) P_j_4(1,6)], [P_j_4(2,5) P_j_4(2,6)], 'c');
plot([P_j_4(1,7) P_j_4(1,10)], [P_j_4(2,7) P_j_4(2,10)], 'c');
plot([P_j_4(1,7) P_j_4(1,8)], [P_j_4(2,7) P_j_4(2,8)], 'c');
plot([P_j_4(1,8) P_j_4(1,9)], [P_j_4(2,8) P_j_4(2,9)], 'c');
plot([P_j_4(1,10) P_j_4(1,11)], [P_j_4(2,10) P_j_4(2,11)], 'c');
plot([P_j_4(1,11) P_j_4(1,12)], [P_j_4(2,11) P_j_4(2,12)], 'c');
plot([P_j_4(1,1) P_j_4(1,7)], [P_j_4(2,1) P_j_4(2,7)], 'c');
plot([P_j_4(1,4) P_j_4(1,10)], [P_j_4(2,4) P_j_4(2,10)], 'c');


profile viewer