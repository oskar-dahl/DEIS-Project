function [] = setSpeedLeft(speed_left)
%SETSPEEDLEFT Set speed of left motor.

    global spd_l;
   
    if speed_left ~= 999
        spd_l = speed_left;
    end
end

