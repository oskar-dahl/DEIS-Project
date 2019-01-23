function gpsCB(src, msg)
    %GPSCB callback function from Josef gps data
    global gps
    gps = str2num(msg.Data);
end

