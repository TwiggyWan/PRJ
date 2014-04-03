#include <node.h>
#include <node_buffer.h>
#include <v8.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp> 
#include <iostream>
#include <vector>

using namespace v8;


/****************************************************************************************
 *											*
 *					image processing				*
 *											*
 ****************************************************************************************/

Handle<Value> test(const Arguments& args)
{
  HandleScope scope;
  std::vector<char> values;
  std::vector<char> valuesN;
  Local<Object> imageBefore = args[0]->ToObject();
  Local<Object> imageNow = args[1]->ToObject();
  Local<Array> props = imageBefore->GetPropertyNames();

    for(unsigned int i = 0; i < props->Length(); i++) {
        String::AsciiValue val(imageBefore->Get(i)->ToString());
        values.push_back(atoi(std::string(*val).c_str()));
        String::AsciiValue valN(imageNow->Get(i)->ToString());
        valuesN.push_back(atoi(std::string(*valN).c_str()));  
    }

    cv::initModule_nonfree();
    //cv::Mat img = cv::imdecode(values,1);
    //cv::Mat imgN = cv::imdecode(valuesN,1);
    //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    //cv::imshow( "Display window", img );                   // Show our image inside it.
    //cv::waitKey(0);  

    cv::Mat img_reference = cv::imdecode(values,CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat img_current = cv::imdecode(valuesN,CV_LOAD_IMAGE_GRAYSCALE);
    

    //reference frame computation
    cv::imshow("reference", img_reference);
    //detect keypoints
    std::vector<cv::KeyPoint> kp_ref;
    cv::SurfFeatureDetector detector(500); //the higher the threshold the less points we get
    detector.detect(img_reference, kp_ref);
    //drawKeypoints(reference,kp_ref,out,Scalar(255,255,255),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


    //get descriptors
    cv::SurfDescriptorExtractor extractor;
    cv::Mat descriptors_reference;
    extractor.compute(img_reference, kp_ref, descriptors_reference);


    //current frame computation
    cv::Mat descriptors_current;
    cv::Mat img_matches;
    std::vector<cv::KeyPoint> kp_current;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> goodMatches;
    double max_dist = 0;
    double min_dist = 100;

    //detect keypoints
    detector.detect(img_current, kp_current);

    //get descriptors
    extractor.compute(img_current, kp_current, descriptors_current);

    //matching
    cv::FlannBasedMatcher matcher;
    matcher.match(descriptors_reference, descriptors_current, matches);

    //find good matches to draw

    //calculation of max/min distances between keypoints
    for (int i = 0; i < descriptors_reference.rows; i++) {

        double dist = matches[i].distance;

        if (dist < min_dist) min_dist = dist;

        if (dist > max_dist) max_dist = dist;


    }

    std::cout << "Max distance :" << max_dist << std::endl;
    std::cout << "Min distance :" << min_dist << std::endl;

    //draw only matches for dist<3*min_dist
    for (int i = 0; i < descriptors_reference.rows; i++) 
    {
        if (matches[i].distance < 3 * min_dist) 
        {
            goodMatches.push_back(matches[i]);
        }
    }

    //display the matches
   cv::drawMatches(img_reference, kp_ref, img_current, kp_current,
            goodMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
            std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    
    //localization
    std::vector<cv::Point2f> ref;
    std::vector<cv::Point2f> current;

    for (int i = 0; i < goodMatches.size(); i++) 
    {
        //get keypoints from the good matches
        ref.push_back(kp_ref[goodMatches[i].queryIdx].pt);
        current.push_back(kp_current[goodMatches[i].trainIdx].pt);


    }

    //get homography matrix
    cv::Mat homography(3, 3, CV_64F); //findhomography renvoie une mat de doubles -> 64bits

    homography = cv::findHomography(ref, current, cv::RANSAC);

    //compute v and w(omega)

    //get inverse matrix
    cv::Mat homographyTransposed;
    cv::transpose(homography, homographyTransposed);

    //get 3x3 identity matrix
    cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);


    //compute task function members

    //taskV
    cv::Mat taskV = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat mStar = cv::Mat::zeros(3, 1, CV_64F);

    mStar.at<double>(0, 0) = kp_ref[goodMatches[0].queryIdx].pt.x;
    mStar.at<double>(0, 1) = kp_ref[goodMatches[0].queryIdx].pt.y;
    mStar.at<double>(0, 2) = 1.0;

    taskV = homography - identity;

    taskV = taskV*mStar;

    //taskOmega
    cv::Mat taskOmega = cv::Mat::zeros(3, 3, CV_64F);

    taskOmega = homography - homographyTransposed;

    std::cout << homography << "HOMOGRAPHY" << std::endl;
    std::cout << homographyTransposed << "TRANSPOSED" << std::endl;
    std::cout << taskOmega << "TASKOMEGA" << std::endl;

    //compute speed and angular speed
    float lambdaV = 0.1;
    float lambdaOmega = 0.1;

    cv::Mat speed = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat angularSpeed = cv::Mat::zeros(3, 1, CV_64F);

    speed = -(lambdaV * identity) * taskV;
    angularSpeed = -(lambdaOmega * identity) * taskOmega;

    //TODO : convertir speed et angularSpeed en params interprétables par le js
    
    
    //show result
    std::cout << speed << "SPEED" << std::endl;
    std::cout << angularSpeed << "ANGULAR SPEED" << std::endl;
    cv::imshow("Result", img_matches);
    
    std::string ret = speed.at(0) + "," + speed.at(1) + "," + speed.at(2) + "," + 
			angularSpeed.at(0) + "," + angularSpeed.at(1) + "," angularSpeed.at(2);
      Local<Value> r = String::New(ret);
      return scope.Close(r);

}

/****************************************************************************************
 *											*
 *					return to nodejs				*
 *											*
 ****************************************************************************************/

void init(Handle<Object> exports) 
{
	exports->Set(String::NewSymbol("test"),
      	FunctionTemplate::New(test)->GetFunction());
}
 
NODE_MODULE(nodecpptest, init)
