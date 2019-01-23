function [] = setArduinoState(arduinoState)
%SETARDUINOSTATE Set arduino state of the robot.
    global A_STATE;
    
    if arduinoState ~= -1
        A_STATE = arduinoState;
    end
end

