/* 
 * File:   main.cpp
 * Author: Anis
 * 
 * Deuxième version
 * SURF detector, calcul de l'homographie
 * Created on 12 février 2014, 13:15
 */

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/flann/flann.hpp"

using namespace cv;
using namespace std;


int main(int argc, char** argv) 
{
    
    cvNamedWindow("Result");
    
    initModule_features2d();

    //open camera, check if succeeded
    VideoCapture capWebCam(0);
    if(!capWebCam.isOpened())
    {
        cout<<"error accessing to camera data!"<<endl;
        return -1;
    }
         
    //reference frame computation
    
    //get reference frame
    Mat img_reference;    
    if(!capWebCam.read(img_reference))
    {
        cout<<"error getting the frame"<<endl;
        return -2;
    }
    
    imshow("reference",img_reference);
    
    //transform it to grayscale
    cvtColor(img_reference,img_reference,CV_RGB2GRAY);
    
    //detect keypoints
    vector<KeyPoint> kp_ref;
    SurfFeatureDetector detector(500); //the higher the threshold the less points we get
    
    detector.detect(img_reference,kp_ref);
    //drawKeypoints(reference,kp_ref,out,Scalar(255,255,255),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    
    //get descriptors
    SurfDescriptorExtractor extractor;
    Mat descriptors_reference;
    extractor.compute(img_reference,kp_ref,descriptors_reference);
    
    
    //current frame computation
    char key = 'a';
    while(key !='q')
    {
       //variables used inside the loop : can i put them inside?
        Mat img_current;
        Mat descriptors_current;
        Mat img_matches;
        vector<KeyPoint> kp_current;
        vector<DMatch> matches;
        vector<DMatch> goodMatches;
        double max_dist = 0;
        double min_dist = 100;
        //get next frame
        if(!capWebCam.read(img_current))
        {
            cout<<"error getting the frame"<<endl;
            break;
        }
        

        //convert it to gray
        cvtColor(img_current,img_current,CV_RGB2GRAY);

        
        //detect keypoints
        detector.detect(img_current,kp_current);        

        //get descriptors
        extractor.compute(img_current,kp_current,descriptors_current);
        
        //matching
        FlannBasedMatcher matcher;
        matcher.match(descriptors_reference,descriptors_current,matches);
        
        //find good matches to draw
        
        //calculation of max/min distances between keypoints
        for(int i=0; i<descriptors_reference.rows;i++)
        {
                double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
        
        }
        
        cout<<"Max distance :"<<max_dist<<endl;
        cout<<"Min distance :"<<min_dist<<endl;
        
        //draw only matches for dist<3*min_dist
        
        for(int i=0;i < descriptors_reference.rows;i++)
        {
            if(matches[i].distance < 3*min_dist)
            {
                goodMatches.push_back(matches[i]);
            }
        }
        
        //display the matches
        drawMatches(img_reference,kp_ref,img_current,kp_current,
                goodMatches,img_matches,Scalar::all(-1),Scalar::all(-1),
                vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        
        
        //localization
        vector<Point2f> ref;
        vector<Point2f> current;
        
        for(int i=0;i<goodMatches.size();i++)
        {
            //get keypoints from the good matches
            ref.push_back(kp_ref[goodMatches[i].queryIdx].pt);
            current.push_back(kp_current[goodMatches[i].trainIdx].pt);
            
            
        }
        
        //get homography matrix
        Mat homography(3,3,CV_64F); //findhomography renvoie une mat de doubles -> 64bits
        
        homography = findHomography(ref,current,RANSAC);
        
        //compute v and w(omega)
        
        //get inverse matrix
        Mat homographyTransposed;transpose(homography,homographyTransposed);
        
        //get 3x3 identity matrix
        Mat identity = Mat::eye(3,3,CV_64F);
        
        
        
        //compute task function members
        
        //taskV
        Mat taskV = Mat::zeros(3,3,CV_64F);
        Mat mStar = Mat::zeros(3,1,CV_64F);
        
        mStar.at<double>(0,0)=kp_ref[goodMatches[0].queryIdx].pt.x;
        mStar.at<double>(0,1)=kp_ref[goodMatches[0].queryIdx].pt.y;
        mStar.at<double>(0,2)=1.0;
        
        taskV = homography - identity;
        
        taskV = taskV*mStar;
        
        //taskOmega
        Mat taskOmega = Mat::zeros(3,3,CV_64F);

        taskOmega = homography - homographyTransposed;
        
        cout<<homography<<"HOMOGRAPHY"<<endl;
        cout<<homographyTransposed<<"TRANSPOSED"<<endl;
        cout<<taskOmega<<"TASKOMEGA"<<endl;
        
        //compute speed and angular speed
        float lambdaV=0.1;
        float lambdaOmega=0.1;
        
        Mat speed = Mat::zeros(3,1,CV_64F);
        Mat angularSpeed = Mat::zeros(3,1,CV_64F);
        
        speed = -(lambdaV*identity)*taskV;
        angularSpeed = - (lambdaOmega*identity)*taskOmega;
        
        
        //show result
       
        cout<<speed<<"SPEED"<<endl;
        cout<<angularSpeed<<"ANGULAR SPEED"<<endl;
        imshow("Result",img_matches);
        key=waitKey(1);
        
    }
    

    
    
    return 0;
}

