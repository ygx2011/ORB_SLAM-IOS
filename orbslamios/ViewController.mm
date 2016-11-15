//
//  ViewController.m
//  orbslamios
//
//  Created by Ying Gaoxuan on 16/11/1.
//  Copyright © 2016年 Ying Gaoxuan. All rights reserved.
//

#import "ViewController.h"

#include <iostream>
#include <boost/thread.hpp>
#include "LocalMapping.hpp"
#include "LoopClosing.hpp"
#include "KeyFrameDatabase.hpp"
#include "ORBVocabulary.hpp"
#include "Converter.hpp"

using namespace cv;
using namespace std;

ORB_SLAM::Map* _World;
ORB_SLAM::Tracking* _Tracker;
ORB_SLAM::ORBVocabulary* _Vocabulary;
ORB_SLAM::KeyFrameDatabase* _Database;
ORB_SLAM::LocalMapping* _LocalMapper;
ORB_SLAM::LoopClosing* _LoopCloser;

bool loadVocab = true;

@interface ViewController ()
{
    CvVideoCamera* videoCamera;
}

@end

@implementation ViewController

@synthesize videoCamera = _videoCamera;

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
    
    self.imageView.contentMode = UIViewContentModeScaleAspectFit;
    self.imageView.image = [UIImage imageNamed:@"icon.jpg"];
    
    self.videoCamera = [[CvVideoCamera alloc] initWithParentView:self.imageView];
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self.videoCamera.defaultFPS = 30;
    self.videoCamera.grayscaleMode = NO;
}

- (IBAction)startButtonPressed:(id)sender
{
    cout<<"Loading Vocab"<<endl;
    
    const char *ORBvoc = [[[NSBundle mainBundle]pathForResource:@"ORBvoc" ofType:@"bin"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
    _Vocabulary = new ORB_SLAM::ORBVocabulary();
    if (loadVocab) {
        bool bVocLoad = _Vocabulary->loadFromBinaryFile(ORBvoc);
        
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
        }
    }
    
    _Database = new ORB_SLAM::KeyFrameDatabase(*_Vocabulary);
    
    
    cout<<"Vocab loaded"<<endl;
    //Initialize the Tracking Thread and launch
    
    _World = new ORB_SLAM::Map();
    
    const char *settings = [[[NSBundle mainBundle]pathForResource:@"Settings" ofType:@"yaml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
    _Tracker = new ORB_SLAM::Tracking(_Vocabulary, _World, settings);
    boost::thread trackingThread(&ORB_SLAM::Tracking::Run, _Tracker);
    
    _Tracker->SetKeyFrameDatabase(_Database);
    
    //Initialize the Local Mapping Thread and launch
    _LocalMapper = new ORB_SLAM::LocalMapping(_World);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run, _LocalMapper);
    
    //Initialize the Loop Closing Thread and launch
    _LoopCloser = new ORB_SLAM::LoopClosing(_World, _Database, _Vocabulary);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, _LoopCloser);
    
    //Set pointers between threads
    _Tracker->SetLocalMapper(_LocalMapper);
    _Tracker->SetLoopClosing(_LoopCloser);
    
    _LocalMapper->SetTracker(_Tracker);
    _LocalMapper->SetLoopCloser(_LoopCloser);
    
    _LoopCloser->SetTracker(_Tracker);
    _LoopCloser->SetLocalMapper(_LocalMapper);
    
    [self.videoCamera start];
}

- (void)processImage:(cv::Mat &)image
{
    Mat colorInput;
    cvtColor(image, colorInput, CV_BGRA2GRAY);
    
    _Tracker->GrabImage(colorInput);
    
    switch(_Tracker->mState)
    {
        case -1 :
        {
            cout<<"SYSTEM_NOT_READY"<<endl;
            break;
        }
        case 0 :
        {
            cout<<"NO_IMAGES_YET"<<endl;
            break;
        }
        case 1 :
        {
            cout<<"NOT_INITIALIZED"<<endl;
            break;
        }
        case 2 :
        {
            cout<<"INITIALIZING"<<endl;
            break;
        }
        case 3 :
        {
            cout<<"WORKING"<<endl;
            Mat R = _Tracker->GetPose_R();
            Mat T = _Tracker->GetPose_T();
            
            std::vector<cv::Point2f> ygx_projectedPoints = _Tracker->Get_projectedPoints();
            for (size_t j = 0; j < ygx_projectedPoints.size(); ++j)
            {
                cv::Point2f r1 = ygx_projectedPoints[j];
                cv::circle(image, cv::Point(r1.x, r1.y), 2, cv::Scalar(0, 255, 0, 255), 1, 8);
            }
            
            break;
        }
        case 4 :
        {
            cout<<"LOST"<<endl;
            break;
        }
        default: {
            break;
        }
    }
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


@end
