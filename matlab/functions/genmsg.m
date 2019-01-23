function [msg] = genmsg(C)
%Used to generate a string message from numerical values

tempMsg = sprintf('%.0f, ', C);
msg = tempMsg(1:end-2);% strip final comma
end

