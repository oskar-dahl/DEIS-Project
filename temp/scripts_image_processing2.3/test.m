clc; clear all; close all;

imrgb=imread('images/distorted800x600.jpg');

pos = main_detect_spirals_server_3_4(imrgb)

figure;
scatter(pos(:,1), pos(:,2));
set(gca,'XAxisLocation','top');
set(gca,'Ydir','reverse');