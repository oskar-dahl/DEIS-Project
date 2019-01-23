function [] = setSpeedRight(speed_right)
%SETSPEEDLEFT Set speed of right motor.

    global spd_r;
   
    if speed_right ~= 999
        spd_r = speed_right;
    end
end

