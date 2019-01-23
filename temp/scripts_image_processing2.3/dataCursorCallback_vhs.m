function output_txt = dataCursorCallback(obj,event_obj)
% Display the position of the data cursor, and the RGB data to 6 decimal places.
global hVec;
pos = get(event_obj,'Position');
output_txt = {['X: ',num2str(pos(1),4), ' ',...
    'Y: ',num2str(pos(2),4)]};

%h = get(event_obj,'target'); In matlab 2015 version this causes error. Case MATTERS! 
h = get(event_obj,'Target');
cdata = get (h, 'CData');
cmap = colormap;
hsvv= rgb2hsv(cdata(pos(2),pos(1),:));
%output_txt{end+1} = ['VHS: ' num2str(hsv(3),'%.4f') ', '  num2str(mod(hsv(1),2*pi)*180/pi,' %.1f') ', '  num2str(hsv(2),'%.4f')];
output_txt{end+1} = ['V: ' num2str(hsvv(3),'%.3f') ' H: '   num2str(hsvv(1)*2*pi,' %.3f')];
% set(hVec,'UData',hsvv(3)*cos(hsvv(1)*2*pi), 
set(hVec,'UData',50*hsvv(3)*cos(hsvv(1)*2*pi), 'VData',-50*hsvv(3)*sin(hsvv(1)*2*pi),'XData',pos(1),'YData',pos(2),'Color',hsv2rgb([mod(hsvv(1)*2*pi-pi,2*pi)/(2*pi),1, 1]),'visible','on');
drawnow;



