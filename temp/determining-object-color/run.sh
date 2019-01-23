#!/bin/sh
rm *.pyc
rm /pyimagesearch/*.pyc

python3 detect_color.py -i /images/undistored640x480.jpg
