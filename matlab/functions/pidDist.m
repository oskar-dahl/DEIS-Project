function [PIDVal] = pidDist(Kp,err)
%PIDDIST Summary of this function goes here
%   Detailed explanation goes here
    P = err;
    PIDVal = Kp*P;
end

