/*
*   main.cpp
*
*   Project for the course 'DT8007 - Design of Embedded and Intelligent Systems' at Halmstad University.
*
*   Developers: Fredrik Johansson, Oskar Dahl
*   All rights reserved. ©
*/

#include "main.hpp"

Mat captureFeed(VideoCapture &capture){

    Mat cameraFeed;

    //-- Webcam
    //store image to matrix
    capture.read(cameraFeed);
    return cameraFeed;
}

/*
*   initCamera()
*   Initializes the camera for OSX use.
*
*   Output: -1 if error occured, 0 if success.
*/
int initCamera(VideoCapture &capture, int FRAME_WIDTH, int FRAME_HEIGHT){

    //-- Webcam
    capture.open(0);
    //set height and width of capture frame
    capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    // check if we succeeded
    if(!capture.isOpened()){
        return -1;
    }

    return 0;
}

int main (){

    char key;
    VideoCapture cap;
    Mat cameraFeed;
    Mat fgMaskMOG2;
    Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor

    //-- Initializes camera and checks for error.
    if( initCamera(cap, 640, 480) == -1 ){
        cout << "Camera initialization failed in main.cpp!" << endl;
        return -1;
    }

    //create GUI windows
    namedWindow("Camera");
    namedWindow("FG Mask MOG 2");

    pMOG2 = createBackgroundSubtractorMOG2(); //MOG2 approach

    while(key != 'q'){
        cameraFeed = captureFeed(cap);
        pMOG2->apply(cameraFeed, fgMaskMOG2);
        imshow("Camera", cameraFeed);
        imshow("FG Mask MOG 2", fgMaskMOG2);
        key = waitKey(30);
    }

    cap.release();
    return 0;
}