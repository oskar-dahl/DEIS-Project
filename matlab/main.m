%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEIS-Project
% Copyright � 2018 Fredrik Johansson & Oskar Dahl. All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main code controlling the robot. Basically a long state machine.

clear all; clc; close all; rosshutdown;

addpath('/Users/fredrik/Desktop/DEIS-Project/matlab/functions/');
savepath;

% Launch GUI
gui = ctrlGUI;

global pubFlag;
global A_STATE;
global M_STATE;
global LEADER;
global spd_l;
global spd_r;
global gps;

pubFlag = 0;
A_STATE = 0;
M_STATE = 0;
LEADER = 0;
spd_l = 70;
spd_r = 70;
gps = -1*ones(10,5);

gui.SpeedleftEditField.Value = spd_l;
gui.SpeedrightEditField.Value = spd_r;


jerryPrev = [-1 -1 -1];
tomPrev = [-1 -1 -1];
ang = 0;

leader = 'Jerry';

% Setting ROS environments variables
setenv('ROS_MASTER_URI','http://192.168.1.8:11311');
setenv('ROS_IP','192.168.1.25');
rosinit();

[pubCtrlJ, ctrlMsgJ] = rospublisher('g5_ctrl_channel_jerry','std_msgs/String');
[pubCtrlT, ctrlMsgT] = rospublisher('g5_ctrl_channel_tom','std_msgs/String');

subGPS = rossubscriber('/josefoutput', 'std_msgs/String', @gpsCB);

%{
url = 'http://192.168.1.2/axis-cgi/jpg/image.cgi';
img_file='image_cam.jpg';
user = 'root';
pass = 'r3dM1lk';
urlwrite(url,img_file,'Authentication','Basic','Username',user,'Password',pass);
imrgb = imread('image_cam.jpg');
%}

run = true;
while run
    pubFlag
    if LEADER == 1
        if strcmp(leader,'Jerry')
            leader = 'Tom';
        elseif strcmp(leader,'Tom')
            leader = 'Jerry';
        end
        LEADER = 0;
    end
    
    if gps(5,1) ~= -1
        jerryPos = [gps(5,2) gps(5,1) 0];
    else
        jerryPos = -1;
    end
    if gps(10,1) ~= -1
        tomPos = [gps(10,2) gps(10,1) 0];
    else
        tomPos = -1;
    end
    
    % Calculate distance and angle between robots.
    if and((jerryPos ~= -1), (tomPos ~= -1))
        ang = atan2(tomPos(2)-jerryPos(2), tomPos(1)-jerryPos(1));
        if ang < 0 % Normalize angle
           ang = ang + 2*pi;
        end
        
        if M_STATE == 0 % Following state
            d = pdist([jerryPos; tomPos],'euclidean');
            PIDVal = floor(pidDist(0.5, (d-60)));
        elseif M_STATE == 1 % Side-by-side state
            pubFlag = 2;
            d = jerryPos(1)-tomPos(1); % Distance in x-axis, traveling in positive direction.
            if d >= 0
                PIDVal = min(30, floor(pidDist(2, d)));
            else
                PIDVal = max(-30, floor(pidDist(2, d)));
            end
        elseif M_STATE == 2 | M_STATE == 3 % Overtake state
            PIDVal = 30;
            d = jerryPos(1)-tomPos(1); % Distance in x-axis, traveling in positive direction.
        end
    else
        PIDVal = 0;
    end

    if pubFlag == 2 % If we use linefollower, messages are published all the time.
        if strcmp(leader, 'Jerry') % Jerry as leader
            PIDValJ = 0;
            PIDValT = PIDVal;
        elseif strcmp(leader, 'Tom') % Tom as leader
            PIDValJ = -PIDVal;
            PIDValT = 0;
        end
        
        % Mission control Jerry - Default leader
        MJ = [(spd_l+PIDValJ), (spd_r+PIDValJ), A_STATE];
        ctrlMsgJ.Data = genmsg(MJ);
        send(pubCtrlJ,ctrlMsgJ);

        % Mission control Tom
        MT = [(spd_l+PIDValT), (spd_r+PIDValT), A_STATE];
        ctrlMsgT.Data = genmsg(MT);
        send(pubCtrlT,ctrlMsgT);
        
    elseif pubFlag == 0 % In special cases, we only want to publish single messages.
        if M_STATE == 2 % Overtake left/right
            if strcmp(leader, 'Jerry') % Jerry as leader
                % TURN LEFT
                % Mission control Jerry
                MJ = [0, 0, 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 2];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE PAST
                % Mission control Jerry
                MJ = [0, 0, 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3.5);
                
                % TURN RIGHT
                % Mission control Jerry
                MJ = [0, 0, 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 3];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE FORWARD AGAIN
                % Mission control Jerry
                MJ = [spd_l, spd_r, 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
            elseif strcmp(leader, 'Tom') % Tom as leader
                % TURN LEFT
                % Mission control Jerry
                MJ = [spd_l, spd_r, 2];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [0, 0, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE PAST
                % Mission control Jerry
                MJ = [spd_l, spd_r, 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [0, 0, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3.5);
                
                % TURN RIGHT
                % Mission control Jerry
                MJ = [spd_l, spd_r, 3];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [0, 0, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE FORWARD AGAIN
                % Mission control Jerry
                MJ = [spd_l spd_r 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
           
            end
            setLeader(1);
            M_STATE = 0;
        elseif M_STATE == 3 % Overtake right side
            if strcmp(leader, 'Jerry') % Jerry as leader
                % TURN RIGHT
                % Mission control Jerry
                MJ = [0 0 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 3];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE PAST
                % Mission control Jerry
                MJ = [0 0 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3.5);
                
                % TURN LEFT
                % Mission control Jerry
                MJ = [0 0 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 2];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE FORWARD AGAIN
                % Mission control Jerry
                MJ = [spd_l, spd_r, 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
            elseif strcmp(leader, 'Tom') % Tom as leader
                % TURN RIGHT
                % Mission control Jerry
                MJ = [spd_l, spd_r, 3];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [0, 0, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE PAST
                % Mission control Jerry
                MJ = [spd_l, spd_r, 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [0, 0, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3.5);
                
                % TURN LEFT
                % Mission control Jerry
                MJ = [spd_l spd_r 2];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [0, 0, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE FORWARD AGAIN
                % Mission control Jerry
                MJ = [spd_l spd_r 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
            end
            setLeader(1);
            M_STATE = 0;
        elseif M_STATE == 4 %Merge left
            if strcmp(leader, 'Jerry') % Jerry as leader
                % DRIVE PAST
                % Mission control Jerry
                MJ = [(spd_l-30) (spd_r-30) 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [(spd_l+20), (spd_r+20), 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);

                pause(4);
                
                % TURN LEFT
                % Mission control Jerry
                MJ = [(spd_l-30) (spd_r-30) 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 2];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE FORWARD AGAIN
                % Mission control Jerry
                MJ = [spd_l spd_r 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
              
            elseif strcmp(leader, 'Tom') % Tom as leader
                % DRIVE PAST
                % Mission control Jerry
                MJ = [(spd_l+20) (spd_r+20) 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [(spd_l-30), (spd_r-30), 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);

                pause(4);
                
                % TURN LEFT
                % Mission control Jerry
                MJ = [spd_l spd_r 2];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [(spd_l-30), (spd_r-30), 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE FORWARD AGAIN
                % Mission control Jerry
                MJ = [spd_l spd_r 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
            end
            setLeader(1);
            M_STATE = 0;
        elseif M_STATE == 5 %Merge right
            if strcmp(leader, 'Jerry') % Jerry as leader
                % DRIVE PAST
                % Mission control Jerry
                MJ = [(spd_l-30) (spd_r-30) 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [(spd_l+20), (spd_r+20), 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);

                pause(4);
                
                % TURN RIGHT
                % Mission control Jerry
                MJ = [(spd_l-30) (spd_r-30) 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 3];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE FORWARD AGAIN
                % Mission control Jerry
                MJ = [spd_l spd_r 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
              
            elseif strcmp(leader, 'Tom') % Tom as leader
                % DRIVE PAST
                % Mission control Jerry
                MJ = [(spd_l+20) (spd_r+20) 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [(spd_l-20), (spd_r-20), 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);

                pause(4);
                
                % TURN RIGHT
                % Mission control Jerry
                MJ = [spd_l spd_r 3];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [(spd_l-30), (spd_r-30), 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE FORWARD AGAIN
                % Mission control Jerry
                MJ = [spd_l spd_r 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
            end
            setLeader(1);
            M_STATE = 0;
        elseif M_STATE == 6 %Fanning left
            if strcmp(leader, 'Jerry') % Jerry as leader
                % TURN LEFT
                disp('TURN LEFT');
                % Mission control Jerry
                MJ = [(spd_l-30), (spd_r-30), 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 2];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE SIDE-BY-SIDE
                disp('GO TO SIDE-BY-SIDE');
                % Mission control Jerry
                MJ = [spd_l, spd_r, 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
            elseif strcmp(leader, 'Tom') % Tom as leader
                % TURN LEFT
                % Mission control Jerry
                MJ = [spd_l, spd_r, 2];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [(spd_l-30), (spd_r-30), 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE SIDE-BY-SIDE
                % Mission control Jerry
                MJ = [spd_l spd_r 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
            end
            M_STATE = 1;
        elseif M_STATE == 6 %Fanning right
            if strcmp(leader, 'Jerry') % Jerry as leader
                % TURN RIGHT
                % Mission control Jerry
                MJ = [(spd_l-30), (spd_r-30), 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 3];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE SIDE-BY-SIDE
                % Mission control Jerry
                MJ = [spd_l, spd_r, 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
            elseif strcmp(leader, 'Tom') % Tom as leader
                % TURN RIGHT
                % Mission control Jerry
                MJ = [spd_l spd_r 3];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [(spd_l-30), (spd_r-30), 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
                
                pause(3);
                
                % DRIVE SIDE-BY-SIDE
                % Mission control Jerry
                MJ = [spd_l spd_r 1];
                ctrlMsgJ.Data = genmsg(MJ);
                send(pubCtrlJ,ctrlMsgJ);

                % Mission control Tom
                MT = [spd_l, spd_r, 1];
                ctrlMsgT.Data = genmsg(MT);
                send(pubCtrlT,ctrlMsgT);
            end
            M_STATE = 1;
        end
        
        % Mission control Jerry
        MJ = [spd_l, spd_r, A_STATE];
        ctrlMsgJ.Data = genmsg(MJ);
        send(pubCtrlJ,ctrlMsgJ);
        
        % Mission control Tom
        MT = [spd_l, spd_r, A_STATE];
        ctrlMsgT.Data = genmsg(MT);
        send(pubCtrlT,ctrlMsgT);
        
        pubFlag = 1;
    end
    
    % Set previous position for next iteration
    if jerryPos ~= -1
        if abs(jerryPos(2)-jerryPrev(2)) > 10 | abs(jerryPos(1)-jerryPrev(1)) > 10
            jerryPrev = jerryPos;
        end
        % Update GUI
        gui.JerryXEditField.Value = jerryPos(1);
        gui.JerryYEditField.Value = jerryPos(2);
        gui.JerryGPSLamp.Color = [0 1 0];
    else
        gui.JerryXEditField.Value = -1;
        gui.JerryYEditField.Value = -1;
        gui.JerrySpeedGauge.Value = spd_l;
        gui.JerryGPSLamp.Color = [1 0 0];
    end
    if tomPos ~= -1
        if abs(tomPos(2)-tomPrev(2)) > 10 | abs(tomPos(1)-tomPrev(1)) > 10
            tomPrev = tomPos;
        end
        % Update GUI
        gui.TomXEditField.Value = tomPos(1);
        gui.TomYEditField.Value = tomPos(2);
        gui.PIDGauge.Value = PIDVal;
        gui.TomSpeedGauge.Value = (spd_l+PIDVal);
        gui.TomGPSLamp.Color = [0 1 0];
    else
        gui.TomXEditField.Value = -1;
        gui.TomYEditField.Value = -1;
        gui.TomSpeedGauge.Value = (spd_l+PIDVal);
        gui.TomGPSLamp.Color = [1 0 0];
    end
    
    gui.AnglebetweenJerryandTomEditField.Value = ang;
    gui.LeaderEditField.Value = leader;
    
    pause(0.01); % Needed to trigger callbacks
end

%%%%% Plotting %%%%%
%{
    scatter(gps(:,2), gps(:,1));
    axis([0 800 0 600]);
    set(gca,'XAxisLocation','top');
    set(gca,'Ydir','reverse');
    drawnow % Force matlab to draw the plot now
    %}

%{
    imrgb = imread('http://192.168.1.2/axis-cgi/jpg/image.cgi');
    gps = main_detect_spirals_server_3_4(imrgb);
    scatter(gps(:,1), gps(:,2));
    axis([0 800 0 600]);
    set(gca,'XAxisLocation','top');
    set(gca,'Ydir','reverse');
    drawnow % Force matlab to draw the plot now
%}
