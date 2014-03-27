#include "ConfigIO.h"

ConfigIO::ConfigIO(String file) : _filename(file)
{// construct
//    if(write){
//        _fs = FileStorage(_filename, FileStorage::WRITE);
//    }else{
//        _fs = FileStorage(_filename, FileStorage::READ);
//    }

    FileStorage fs;
    string filename = _filename;
    fs.open(filename,cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        cerr << "Failed to open " << filename << endl;
        throw new Exception();
    }

    std::cout << "Reading from file..." << std::endl;
    fs["Camera_Matrix"]>>_cameraMatrix;
    fs["Distortion_Coefficients"]>>_disortionMatrix;

}

static int _num = 0;

Mat ConfigIO::Calibrate(Mat input)
{
    Mat tmp = input.clone();
    cout << "Calibrating" << _num++ << endl;
    undistort(input, tmp, _cameraMatrix, _disortionMatrix);
    return tmp;
}



ConfigIO::~ConfigIO()
{
    _fs.release();
}

//template<class T>
//void ConfigIO::WriteObject(String name, T object)
//{
//    _fs << name << object;
//}

//template<typename TYPE>
//TYPE ConfigIO::ReadObject(String name)
//{
//    TYPE obj_out;
//    _fs[name] >> obj_out;
//}
