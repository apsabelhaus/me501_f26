% hw1_me501_Q5a.m
% (C) Andrew Sabelhaus, 2024

% some setup
clear all;
close all; % just in case other figure windows are previously open
clc;

%% Q5a: Simulating a Point Mass Moving in One Dimension

% FILL_THIS_IN
m = 2;
k = 10;
bar_x1 = 0.5;

% FILL_THIS_IN
x1_0 = 0.2;
dotx1_0 = 0.0;

% FILL_THIS_IN
dt = 0.01; % milliseconds
tF = 10; % seconds. Final time. Simulate from "now" at t=0 until tF.

% it's usually a good idea to "pre-allocate" a place to store our results.
% MATLAB is a lot faster to change the values inside a vector or matrix rather than
% concatenating new values onto the end of an existing vector or matrix.
n_timesteps = tF/dt;
% our problem has two states, x1 and dotx1, so we will get 2 x n_timesteps
% data points. 

% FILL_THIS_IN
x_traj = zeros(2, n_timesteps+1);
% FILL_THIS_IN
x_traj(1,1) = x1_0;
x_traj(2,1) = dotx1_0;

% finally, simulate with Forward Euler
for t=1:n_timesteps
    % it's easier to see what's going on if I calculate f(x) first, at this
    % timestep, before performing the update.
    % For our spring-mass: f(boldx_t) = [ dotx_t; -(k/m)*(x1_t-bar_x1)];
    f_xt = [x_traj(2,t); -(k/m)*(x_traj(1,t) - bar_x1)];
    % FILL_THIS_IN
    x_traj(:, t+1) = x_traj(:,t) + dt*f_xt;
end

% plotting the positions vs time
timepts = 0:dt:tF;
plot(timepts, x_traj(1,:));
title('ME501 HW1 Q5a -- Point Mass 1D position vs time');
xlabel('time (sec)');
ylabel('x1 (meters)');
drawnow; % so it doesn't interfere with the video

% flag to easily toggle saving a video or not
save_video = 0;

if save_video
    % save a video. Setup:
    video_filename_Q5a = "hw1_me501_Q5a";
    video_profile_name = "Motion JPEG AVI"; % this is the default for matlab
    vwriter = VideoWriter(video_filename_Q5a, video_profile_name);
    % Set the frame rate: "this many frames per second"... which must come from
    % our sampling rate dt, so that the video shows at the right speed.
    % Remember that frequency = 1/period
    vwriter.FrameRate = 1/dt;
    open(vwriter);
    % start the plot:
    figure; hold on;
    title('ME501 HW1 Q5a -- Point Mass 1D Position Trajectory');
    xlabel('x1 (meters)');
    % choose some bounds for the window. you may/will need to adjust this!
    xlim([-2, 2]);
    % plot the bar_x1 point and initial condition x1_0, both on the "y-axis,"
    % and with some arbitrary size circle for the point mass. See help for the
    % scatter function for more info.
    scatter(bar_x1, 0, 30, "red", "filled");
    x1_plotted_handle = scatter(x_traj(1,1), 0, 30, "blue", "filled");
    legend(["bar x1", "x1 t"]);
    % save the first frame to the video. The getframe command turns a figure
    % window into a movie frame, and gcf is the current figure window that's
    % open ("get current figure")
    drawnow;
    writeVideo(vwriter, getframe(gcf));
    
    % save a video of the particle moving through its x1 positions
    for t=2:n_timesteps
        % delete the previous circle representing the particle
        delete(x1_plotted_handle);
        % and make a new one
        x1_plotted_handle = scatter(x_traj(1,t), 0, 30, "blue", "filled");
        % the legend gets deleted when you delete a handle, so add it back
        legend(["bar x1", "x1 t"]);
        % save it
        writeVideo(vwriter, getframe(gcf));
    end
    
    % we're done, close the video file so you can open it in another program to
    % watch
    close(vwriter);
    disp("Video save completed to " + video_filename_Q5a);
end














