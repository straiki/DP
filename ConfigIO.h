#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

class ConfigIO {

private:
    String _filename;
    FileStorage _fs;

    Mat _cameraMatrix;
    Mat _disortionMatrix;

public:
    ConfigIO(String file = "../../images/camera_calib_lumia.yaml");
    ~ConfigIO();

    FileStorage getFS(){return _fs;}

    Mat Calibrate(Mat input);
};
