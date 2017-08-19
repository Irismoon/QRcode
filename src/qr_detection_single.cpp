#include "ros/ros.h"
#include <iostream>
#include <px4_autonomy/Position.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Takeoff.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <ARToolKitPlus/TrackerSingleMarker.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


using ARToolKitPlus::TrackerSingleMarker;
using namespace std;
using namespace cv;

const bool useBCH = false;
static const int width=640, height=360, bpp=1;
static const int numPixels = width *height * bpp;
unsigned char cameraBuffer[numPixels];
const int foundthld=3;
const int nummarker = 2003;
Mat img,img_gray;

void imageCallback(const sensor_msgs::Image &msg)
{
	//printf("callback established\n");
	cout << msg.data.size() << endl;
	/*for (int i=0;i<numPixels;i++){
		cameraBuffer[i] = 0.2989*msg.data[3*i]+0.5870*msg.data[3*i+1]+0.1140*msg.data[3*i+2];
		//printf("%s\n", );B,G,R

	}*/
	//printf("successful callback\n");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:%s",e.what());
    }
	img = cv_ptr->image;
	cvtColor(img, img_gray, CV_BGR2GRAY);
	int seq = 0;
	for(int j=0;j<img.rows;j++)
	{
		uchar* raw= (img_gray).ptr<uchar>(j); 
		for(int i=0;i<img.cols;i++)
		{
			cameraBuffer[seq] = raw[i];
			seq++;
		}
	}
	imshow("received",img_gray);
	waitKey(1);
};



int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "qr_detection_single");
    ros::NodeHandle node;
    ros::Subscriber image_sub;
    //ros::Publisher _pub = n.advertise<std_msgs::Bool>("/image_loss",1) ;
    ros::Rate loop_rate(10);
    image_sub = node.subscribe("/camera/image_raw",5,imageCallback);
    float conf;
    size_t numBytesRead;
    int i=0;
    int id=0;
    static bool found = false;
    bool identified[nummarker] = {false};
    char pngpath[50];
    //c markerId;
    /*printf("error:creat image instance\n");
    const char *fName = useBCH ? "/home/ubuntu/src/ARToolKitPlus/sample/data/image_320_240_8_marker_id_bch_nr0100.raw"
            : "/home/ubuntu/src/ARToolKitPlus/sample/data/image_320_240_8_marker_id_simple_nr031.raw";
    if (FILE* fp = fopen(fName, "rb")) {
        numBytesRead = fread(cameraBuffer, 1, numPixels, fp);
        fclose(fp);
    } else {
        printf("Failed to open %s\n", fName);
        return -1;
    }
    for(int i=0;i<numPixels;i++){

    	cout << int(cameraBuffer[i]) << endl;
    }*/

    // create a tracker that does:
    //  - 6x6 sized marker images (required for binary markers)
    //  - samples at a maximum of 6x6
    //  - works with luminance (gray) images
    //  - can load a maximum of 0 non-binary pattern
    //  - can detect a maximum of 8 patterns in one imagege
    TrackerSingleMarker tracker(width,height, 8, 6, 6, 6, 0);
    printf("initialized tracker\n");
    tracker.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    //tracker.setLoadUndistLUT(true);
    
    // load a camera file.
    if (!tracker.init("/home/ubuntu/catkin_ws/src/qrcode/data/cloudplat_camera.cal", 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR: init() failed\n");
        return -1;
    }

    tracker.getCamera()->printSettings();

    // define size of the marker in OpenGL units
    tracker.setPatternWidth(2.0);

    // the marker in the BCH test image has a thin border...
    tracker.setBorderWidth(useBCH ? 0.125 : 0.25);

    // set a threshold. alternatively we could also activate automatic thresholding
    tracker.setThreshold(150);

    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker.setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker.setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);

    // do the OpenGL camera setup
    // glMatrixMode(GL_PROJECTION)
    // glLoadMatrixf(tracker.getProjectionMatrix());

    // here we go, just two calls to find the camera pose
    while (ros::ok())
    {
    	//printf("afterrosok\n");
    ros::spinOnce();	
    std::vector<int> markerId = tracker.calc(cameraBuffer);
    //markerId.push_back(1);
    int x = tracker.selectBestMarkerByCf();
    conf = tracker.getConfidence();
    if(markerId.empty()){
    	ROS_INFO("none idetified" );
    }
    else{
    	if(markerId[0]==id&&!identified[id]){
    		i++;
            cout << i<< endl;
    		if(i>=foundthld){//continuously identified foundthld times, we save the photo and publish the result
    			i=0;
    			identified[id] = true;
                sprintf(pngpath,"/home/ubuntu/catkin_ws/src/qrcode/Result/<%d>.png",id);
    			imwrite(pngpath,img);//save photo and publish result
                printf("successfully write image\n");
    		}
    	}	
    	if(markerId[0]!=id){
    		id = markerId[0];
            cout << id << endl;
    	}

    	cout << "Found marker:" << markerId[0] << " at confidence: " << int(conf*100.0f) << "%%" << endl;
    	//printf("Found marker %d  (confidence %d%%)\n\  ", markerId[0], (int(conf * 100.0f)));
    }
    // use the result of calc() to setup the OpenGL transformation
    // glMatrixMode(GL_MODELVIEW)
    // glLoadMatrixf(tracker.getModelViewMatrix());

    //printf("\nFound marker %d  (confidence %d%%)\n\nPose-Matrix:\n  ", markerId[0], (int(conf * 100.0f)));
    /*for (int i = 0; i < 16; i++)
        printf("%.2f  %s", tracker.getModelViewMatrix()[i], (i % 4 == 3) ? "\n  " : "");
    */
    loop_rate.sleep();
    }

    return 0;
}