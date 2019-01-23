%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEIS-Project
% Copyright © 2018 Fredrik Johansson & Oskar Dahl. All rights reserved. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc;

img = imread('clean.jpg');

figure(1);
imshow(img);

[x, y] = getpts();

% Get number of points.
c = [0:1:(max(size(x)))-1]';

pts = [x y]';

fileID = fopen('lane_merge.txt','w');
fprintf(fileID,'%d %d\n',pts);
fclose(fileID);