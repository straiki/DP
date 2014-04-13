/*****************************************************************************
*   ExploringSfMWithOpenCV
******************************************************************************
*   by Roy Shilkrot, 5th Dec 2012
*   http://www.morethantechnical.com/
******************************************************************************
*   Ch4 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#include <iostream>
#include <string.h>

#include "Distance.h"
#include "MultiCameraPnP.h"
#include "Visualization.h"

using namespace std;

#include <opencv2/gpu/gpu.hpp>

class VisualizerListener : public SfMUpdateListener {
public:
	void update(std::vector<cv::Point3d> pcld,
				std::vector<cv::Vec3b> pcldrgb, 
				std::vector<cv::Point3d> pcld_alternate,
				std::vector<cv::Vec3b> pcldrgb_alternate, 
				std::vector<cv::Matx34d> cameras) {
		ShowClouds(pcld, pcldrgb, pcld_alternate, pcldrgb_alternate);
		
		vector<cv::Matx34d> v = cameras;
		for(unsigned int i=0;i<v.size();i++) {
			stringstream ss; ss << "camera" << i;
			cv::Matx33f R; 
			R(0,0)=v[i](0,0); R(0,1)=v[i](0,1); R(0,2)=v[i](0,2);
			R(1,0)=v[i](1,0); R(1,1)=v[i](1,1); R(1,2)=v[i](1,2);
			R(2,0)=v[i](2,0); R(2,1)=v[i](2,1); R(2,2)=v[i](2,2);
			visualizerShowCamera(R,cv::Vec3f(v[i](0,3),v[i](1,3),v[i](2,3)),255,0,0,0.2,ss.str());
		}
	}
};

std::vector<cv::Mat> Images;
std::vector<std::string> Images_names;

int main(int argc, char** argv) {
    cv::Ptr<MultiCameraPnP> distance;

    cv::Ptr<VisualizerListener> visualizerListener = new VisualizerListener; //with ref-count

    if(argc < 2){
        cerr << argv[0] << " -config:file.yaml or " << argv[0] << " <path_to_images> [downscale factor = 1.0]" << endl;
        return 0;
    }

    string param(argv[1]);
    std::string::size_type size;

    if (argc == 2 && (size = param.find("-config:")) != string::npos ) {
        cout << "Loading settings from file!" << endl;
        string config(param.substr(size+8));

        myloader *loader = new myloader(config);
        loader->parseConfig();
        loader->loadFiles();
//        return 0;
        distance = new MultiCameraPnP(loader, loader->imgs, loader->imgs_names, loader->directory_);
        distance->attach(visualizerListener);
        RunVisualizationThread();
    }else{
        double downscale_factor = 1.0;
        if(argc == 3)
            cout << argv[3] << endl;
    //        downscale_factor = strtod(argv[3], NULL);

        open_imgs_dir(argv[1],Images,Images_names,downscale_factor);
        if(Images.size() == 0) {
            cerr << "can't get image files" << endl;
            return 1;
        }

        // open files, conver to grayscale, load cam matrix
        distance = new MultiCameraPnP(Images,Images_names,string(argv[1]));

        distance->use_rich_features = true;

        distance->use_gpu = false;


        distance->attach(visualizerListener);
        RunVisualizationThread();

        distance->RecoverDepthFromImages();

        //get the scale of the result cloud using PCA
        double scale_cameras_down = 1.0;
        {
            vector<cv::Point3d> cld = distance->getPointCloud();
            if (cld.size()==0) cld = distance->getPointCloudBeforeBA();
            cv::Mat_<double> cldm(cld.size(),3);
            for(unsigned int i=0;i<cld.size();i++) {
                cldm.row(i)(0) = cld[i].x;
                cldm.row(i)(1) = cld[i].y;
                cldm.row(i)(2) = cld[i].z;
            }
            cv::Mat_<double> mean;
            cv::PCA pca(cldm,mean,CV_PCA_DATA_AS_ROW);
            scale_cameras_down = pca.eigenvalues.at<double>(0) / 5.0;
            //if (scale_cameras_down > 1.0) {
            //	scale_cameras_down = 1.0/scale_cameras_down;
            //}
        }

    }
	
	
	visualizerListener->update(distance->getPointCloud(),
							   distance->getPointCloudRGB(),
							   distance->getPointCloudBeforeBA(),
							   distance->getPointCloudRGBBeforeBA(),
							   distance->getCameras());
							   

	//ShowCloud(distance->getPointCloud(), 
	//		   distance->getPointCloudRGB(),
	//		   "baseline_only");
	//WaitForVisualizationThread();
	//return 1;
	
//	ShowClouds(distance->getPointCloud(), 
//			   distance->getPointCloudRGB(),
//			   distance->getPointCloudBeforeBA(),
//			   distance->getPointCloudRGBBeforeBA()
//			   );
	WaitForVisualizationThread();
}
