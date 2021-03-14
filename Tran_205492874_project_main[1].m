%==========================================================================
%Final Project (Tran_205492874_project_main.m)
%David Tran
%UCLA ID: 205-492-874
%{
Description: The purpose of this script is to implement a self-driving car
that locates an empty parking spot in a lot populated with 20 spots in 
total on both sides. Then, a car will be created using a separate function
and parked cars will be populated randomly in the lot. The goal is to have
this vehicle navigate the lot and park into a lot without colliding into
any other parked cars. To enable visualization, the path of the car will be
drawn onto the figure, as it traverses from its initial to final positions.
Finally, animations will also be generated that show the car moving along
this path at a constant speed. This video will last for 10 seconds and
contain 30 frames per second, per given specifications.
%}

%=============================Clear Cache==================================
clc; close all; clear all;

%============================Video Writer==================================
video_file = VideoWriter('top_row.mp4','MPEG-4');
video_file.FrameRate = 30;  % Set video frame rate
open(video_file);           % Open the video file to write into

%===========================Running the script=============================
%%Variables and Arrays
l_car = 5;                      %length of car in [m]
w_car = 2.30;                   %width of car in [m]
l_spot = 1.25 * l_car;          %length of spot in [m]
w_spot = 2 * w_car;             %width of spot in [m]
l_lot = 4 * l_spot;             %length (vertical) of parking lot in [m]
w_lot = 14 * w_spot;            %width (horizontal) of parking lot in [m]
n_row = 10;                     %number of spots per row [dimensionless]
color = [39 116 174] / 255;     %color of car (UCLA Blue)
parkedColor = [255 209 0] / 255;%color of parked cars (UCLA Yellow)
finalColor = [210 100 10] / 255;%color of self-driving car in its final parked state
angle = 0;                      %initial angle of car in [radians w.r.t x-axis]
finalAngle = pi/2;              %final angle of car in [radians w.r.t x-axis]
x_car = 0;                      %starts car at random x value
y_car = l_spot+(w_car)/2 + (l_lot - l_spot - w_car/2 - ((l_spot)+(w_car)/2))*rand;
                                %starts car at random y value
x_spots = zeros(1,2*n_row);     %array to store x values of center of each spot
y_spots = zeros(1,2*n_row);     %array to store y values of center of each spot
filled = zeros(1,2*n_row);      %array to store if a spot is filled
n_cars = randi([1 19]);         %number of parked cars
parkedCount = 0;                %count of parked cars
TR = 4.75;                      %turning radius of car
distance = zeros(1,2*n_row);    %array to store distance to each spot
t0 = 0;                         %initial time in [s]
tf = 10;                        %final time in [s]
fps = 30;                       %frames per second
frames = (tf - t0) * fps;       %total frames
path_x = zeros(1,frames);        %x coordinates of path with 300 steps (30 fps * 10 s = 300 frames)
path_y = zeros(1,frames);        %y coordinates of path with 300 steps (30 fps * 10 s = 300 frames)

%%Create the parking lot
[x_spots, y_spots] = create_lot(l_lot, w_lot, l_spot, w_spot, n_row);

%%Create the initial car (use a while loop to keep iterating until parked)
%%and fill the parking lot with parked cars
create_car(x_car, y_car, l_car, w_car, color, angle);   %draw the car

%Determine which spots have cars are parked at random
while parkedCount < n_cars
    for k = 1:2*n_row
        if (rand <= n_cars/(2*n_row)) && (filled(k) == 0) && (parkedCount < n_cars)
            %car parked
            filled(k) = 1; %spot is filled
            parkedCount = parkedCount + 1;
        elseif (rand > n_cars/(2*n_row)) && (filled(k) == 0)
            %no car
            %seems redundant but ensures that no parked spots are overriden
            filled(k) = 0; %spot is empty
        end
    end
end

%Fill the parking lot with cars
populate_spots(x_spots, y_spots, n_cars, parkedColor, l_car, w_car, n_row, filled);

%Must have at least one empty spot, verify that here
if ismember(0,filled) ~= 1
    error('Rerun the script: all spots are filled.');
elseif n_cars >= 2*n_row
    error('All spots are filled.');
end

for k = 1:2*n_row
    if filled(k) == 0
        %spot is empty, check distance and store it in array, then find min
        distance(k) = sqrt((x_spots(k) - x_car)^2 + (y_spots(k) - y_car)^2);
    else
        distance(k) = 10000;    %Arbitrary large value
    end
end

%Find indice of closest parking spot
minVal = min(min(distance));
minInd = find(distance==minVal);
x_spot = x_spots(minInd);
y_spot = y_spots(minInd);

%Check if indice is in top or bottom row
if minInd > 10
    topStatus = 0;
else
    topStatus = 1;
end

%Variables/Calculations of total steps and distance of path
frames = (tf - t0) * fps;                   %total frames
length_horizontal = x_spot - x_car - TR;    %horizontal length
length_vertical = abs(y_spot - y_car) - TR; %vertical length
length_curve = TR * pi/2;                   %curve length

%Total frames/points per path
frames_horizontal = floor((length_horizontal/(length_horizontal + length_vertical + length_curve)) * frames);
frames_vertical = floor((length_vertical/(length_horizontal + length_vertical + length_curve)) * frames);
frames_curve = ceil((length_curve/(length_horizontal + length_vertical + length_curve)) * frames);

d_theta = (pi/2)/frames_curve;    %change in angle per frame
    
[path_x, path_y] = find_path(x_car, y_car, x_spot, y_spot, TR, t0, tf, fps, topStatus); %draw the path of the car    
create_car(path_x(frames), path_y(frames), l_car, w_car, finalColor, finalAngle);   %final position of the car
plot(path_x,path_y,'k'); %plot the path of the car to its intended destination
axis equal;
pause(0.5); %hold frame for 0.5 seconds

for k = 1:frames
    if k <= frames_horizontal
        %horizontal path
        newAngle = angle;
    elseif k > (frames_horizontal + frames_curve) 
        %vertical path
        newAngle = finalAngle;
    elseif topStatus == 1 && k > frames_horizontal && k < (frames_horizontal + frames_curve)
        %in top row
        %define the change in angle in curved portion
        newAngle = (k - frames_horizontal) * d_theta;
    elseif topStatus == 0 && k > frames_horizontal && k < (frames_horizontal + frames_curve)
        %in bottom row
        newAngle = -(k - frames_horizontal) * d_theta;
    end
    
    %obtain the kth position and plot it
    [x_spots, y_spots] = create_lot(l_lot, w_lot, l_spot, w_spot, n_row);
    populate_spots(x_spots, y_spots, n_cars, parkedColor, l_car, w_car, n_row, filled);
    create_car(path_x(k), path_y(k), l_car, w_car, color, newAngle);
    
    axis equal;
    
    pause(0.001);   %hold frame for 0.001 seconds
    
    %obtain frame
    current_frame = getframe(gcf);
    
    %store frame in the video file
    writeVideo(video_file,current_frame);
    
    %clear previous position
    clf;
end

close(video_file);  %close video file

%=============================Functions====================================
function [x_spots, y_spots] = create_lot(l_lot, w_lot, l_spot, w_spot, n_row)
%This function takes in 5 parameters that define the dimensions of the
%overall parking lot, the individual spots, and the number of spots per
%row. The outputs are arrays that are the center of each parking spot.

%%Error Checking
if w_lot < (3*w_spot + n_row*w_spot)
    error('Invalid lot size, must have at least 3 spots of white space.');
elseif ~isnumeric(l_lot) || ~isnumeric(w_lot) || ~isnumeric(l_spot) || ~isnumeric(w_spot) || ~isnumeric(n_row)
    error('Parameters must be numeric values.');
elseif (l_lot <= 0) || (w_lot <= 0) || (l_spot <= 0) || (w_spot <= 0) || (n_row <= 0)
    error('Parameters must be positive numbers.');
end

%%Create the figure and grid
figure(1);
title('Self-Parking Car Simulation','Interpreter','latex');
axis([0 w_lot 0 l_lot+5]);

%%Draw the actual parking lot
x = [0 w_lot w_lot 0 0];
y = [0 0 l_lot l_lot 0];
hold on;
plot(x,y,'k');

%%Determine the centers of each parking spot and draw the spots
for k = 1:2*n_row
    if k <= n_row
        x_spots(k) = (4 * w_spot) + (w_spot * k) - (w_spot/2);
        y_spots(k) = l_lot - (l_spot / 2);
        x = [x_spots(k)+(w_spot/2) x_spots(k)-(w_spot/2) x_spots(k)-(w_spot/2) x_spots(k)+(w_spot/2) x_spots(k)+(w_spot/2)];
        y = [l_lot l_lot l_lot-l_spot l_lot-l_spot l_lot];
        plot(x,y,'r');
    else
        x_spots(k) = (4 * w_spot) + (w_spot * (k - 10)) - (w_spot/2);
        y_spots(k) = l_spot / 2;
        x = [x_spots(k)+(w_spot/2) x_spots(k)-(w_spot/2) x_spots(k)-(w_spot/2) x_spots(k)+(w_spot/2) x_spots(k)+(w_spot/2)];
        y = [l_spot l_spot 0 0 l_spot];
        plot(x,y,'r');
    end   
end

end

function create_car(x_car, y_car, l_car, w_car, color, angle)
%With no outputs, this function's purpose is to simply create a rectangular
%box that depicts the self-driving car. It has 6 parameters that define the
%center of the rectangle, its dimensions, color, and angle of rotation with
%respect to the x axis.

%%Error Checking
if ~isnumeric(l_car) || ~isnumeric(w_car) || ~isnumeric(angle)
    error('Parameters must be numeric.');
elseif (l_car <= 0) || (w_car <= 0)
    error('l_car or w_car must be positive.');
end

%%Draw the car
x = [(x_car+(l_car/2)*cos(angle)+(w_car/2)*sin(angle)) (x_car-(l_car/2)*cos(angle)+(w_car/2)*sin(angle)) ... 
    (x_car-(l_car/2)*cos(angle)-(w_car/2)*sin(angle)) (x_car+(l_car/2)*cos(angle)-(w_car/2)*sin(angle)) (x_car+(l_car/2)*cos(angle)+(w_car/2)*sin(angle))];
y = [(y_car+(l_car/2)*sin(angle)-(w_car/2)*cos(angle)) (y_car-(l_car/2)*sin(angle)-(w_car/2)*cos(angle)) ...
    (y_car-(l_car/2)*sin(angle)+(w_car/2)*cos(angle)) (y_car+(l_car/2)*sin(angle)+(w_car/2)*cos(angle)) (y_car+(l_car/2)*sin(angle)-(w_car/2)*cos(angle))];
fill(x,y,color);    %UCLA FIGHT FIGHT FIGHT

end

function populate_spots(x_spots, y_spots, n_cars, parkedColor, l_car, w_car, n_row, filled)
%Once again, this function has no outputs, taking in 3 parameters that
%serve to add a random number of parked cars in spots created by
%create_lot. These parameters are the center of each spot and tell how many
%spots are to be populated.

count = 0;  %count of cars

%%Error Checking
if n_cars > 19 || n_cars < 0
    error('Number of cars exceeds maximum or is negative.');
elseif ~isnumeric(n_cars) || ~isnumeric(l_car) || ~isnumeric(w_car) || ~isnumeric(n_row)
    error('Parameters must be numeric values.');
end

%%Fill the lot with the cars
for k = 1:2*n_row
    if filled(k) == 1 && count < n_cars
        create_car(x_spots(k), y_spots(k), l_car, w_car, parkedColor, pi/2);
        count = count + 1;
    end
end

end

function [path_x, path_y] = find_path(x_car, y_car, x_spot, y_spot, TR, t0, tf, fps, topStatus)
%Implementing the actual self-parking feature of the car, this function
%determines the path that the self-driving vehicle traverses from its
%initial to final parking spot. The first 2 parameters define the center of
%the car; the next 2 are the centers of the final spot; and TR is the
%turning radius. The outputs are arrays for all of the x and y coordinates
%of this path.

%%Although these functions are defined at the beginning, passing them to
%%this function would make it too clunky in my opinion, so I opted to
%%redefine them here.

%%Error Checking
if topStatus ~= 1 && topStatus ~= 0
    error('topStatus is not a valid argument.');
elseif ~isnumeric(x_spot) || ~isnumeric(y_spot) || ~isnumeric(x_car) || ~isnumeric(y_car) || ~isnumeric(TR)...
        || ~isnumeric(t0) || ~isnumeric(tf) || ~isnumeric(fps) || ~isnumeric(topStatus)
    error('Parameters must be numeric values.');
end

%%Variables/Calculations of total steps and distance of path
frames = (tf - t0) * fps;                   %total frames
length_horizontal = x_spot - x_car - TR;    %horizontal length
length_vertical = abs(y_spot - y_car) - TR; %vertical length
length_curve = TR * pi/2;                   %curve length

%Total frames/points per path
frames_horizontal = floor((length_horizontal/(length_horizontal + length_vertical + length_curve)) * frames);
frames_vertical = floor((length_vertical/(length_horizontal + length_vertical + length_curve)) * frames);
frames_curve = ceil((length_curve/(length_horizontal + length_vertical + length_curve)) * frames);

%Distance per frame
distance_horizontal = length_horizontal/frames_horizontal;
distance_vertical = length_vertical/frames_vertical;
d_theta = (pi/2)/frames_curve;    %change in angle per frame

%Initial positions
path_x(1) = x_car;              %initial x position of car
path_y(1) = y_car;              %initial y position of car

%%Car's positions for horizontal path
for k = 2:frames
    if k <= frames_horizontal
        %in horizontal portion
        path_x(k) = path_x(k-1) + distance_horizontal;
        path_y(k) = path_y(k-1);
    elseif (k > frames_horizontal) && (k <= (frames_horizontal + frames_curve))
        %in curved portion
        %car going to upper row       
        path_x(k) = x_spot - TR + TR*sin(d_theta * (k - frames_horizontal));
        
        if topStatus == 1
            %spot is in top row
            path_y(k) = y_car + TR - TR*cos(d_theta * (k - frames_horizontal));
        else
            %spot is in bottom row
            path_y(k) = y_car - TR + TR*cos(d_theta * (k - frames_horizontal));
        end
        
    elseif (k > (frames_horizontal + frames_curve)) && (k <= frames)
        %in vertical portion
        path_x(k) = path_x(k-1);
        
        if topStatus == 1 && abs(path_y(k-1) - y_spot) > 10^(-2.5)
            %the quantity 10^(-2.5) was determined through trial and error
            %spot is in top row
            path_y(k) = path_y(k-1) + distance_vertical;
        elseif topStatus == 0 && abs(path_y(k-1) - y_spot) > 10^(-2.5)
            %spot is in bottom row
            path_y(k) = path_y(k-1) - distance_vertical;
        end
        
        if (abs(path_y(k-1) - y_spot)) <= 10^(-2.5) && k <= frames
            %car is at its spot and it it still running
            %sometimes the car moves too far ahead or below the rest of the
            %cars, this line prevents that
            path_y(k) = y_spot;
        end
    end
end

end