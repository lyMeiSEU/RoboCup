#include "UniRobot.hpp"
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/Display.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2VisionManager.hpp>

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace webots;
using namespace managers;
using namespace std;
using namespace cv;

static double clamp(double value, double min, double max) 
{
  if (min > max) 
  {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

static double minMotorPositions[NMOTORS];
static double maxMotorPositions[NMOTORS];

void UniRobot::imageProcess()
{
	unsigned char *rgb = getRGBImage();  //get raw data, format: RGB
	Mat rgbMat = getRGBMat(); //get rgb data to cv::Mat
            
	cvtColor(rgbMat, rgbMat, COLOR_BGR2GRAY);//杞伆搴﹀浘
	threshold(rgbMat, rgbMat, 200, 255, THRESH_BINARY);//浜屽€煎寲

	showImage(rgb);
	if (mode == MODE_BALL)
	{
           resInfo.ball_found=false;
           int temp,block_a,block_b=0,max_a=0,max_b=0,Block_b;
           //cout<<"start"<<endl;
	for (int i = 0; i < rgbMat.rows; i += 3)
	{
            if (resInfo.ball_found) break;
  	    for (int j = 0; j < rgbMat.cols; j += 1)
               {	if (resInfo.ball_found) break;
                       temp=0;
                       block_a=0;
                       max_a=0;
                       if(rgbMat.at<unsigned char>(i, j) == 255)
                       {
            		for(int tempy = i; tempy <rgbMat.rows;tempy ++)
             		{
                        		if (rgbMat.at<unsigned char>(tempy,j) == 255&&block_a<35&&(block_a<max_a*2||max_a<3))
                       		{
                                   		//if(block_a>20&&block_atemp/3) {break;}
                                  		temp++;
                                  		temp+=block_a;
                                  		if(max_a<block_a)
                                  		{
                                  		max_a=block_a;
                                  		}
                                  		block_a=0;
                        		}
                        		else
                        		{
                        		block_a++;
                        		}
             		}
                       	if(temp<10||max_a>temp*2/5)
                       	{
                                     	continue;
                       	}
                                	//cout<<i<<" "<<j<<" "<<temp<<endl;
                       	
            			
                        		//cout<<i<<"   "<<j<<endl;
                        	
                        }
                        int num=0,NUM=0;
                        Block_b=0;
                        max_b=0;
                        if(j-temp/2<0||j+temp/2>rgbMat.cols){break;}
                        for(int tempx =j-temp/2;tempx<j+temp/2;tempx++)
                        {
                            if(rgbMat.at<unsigned char>(i, tempx) == 0)
                            {
                                block_b++;
                                if(block_b>temp*2/5){break;}
                                if(NUM==num)
                                {
                                NUM++;
                                }
                            }
                            else
                            {
                                Block_b+=block_b;
                                if(max_b<block_b){max_b=block_b;}
                               // if((block_b<temp/2&&NUM>1)||NUM>2){resInfo.ball_found=true;}
                                block_b=0;
                                if(num<NUM)
                                {
                                num++;
                                }
                            }
                        }
                        if(Block_b<temp*3/5&&max_a>3&&max_b>3&&max_a<temp/3&&max_b<temp/3&&((temp<55&&temp>25&&NUM)||(temp>55 && NUM>2)))
                        {
                          resInfo.ball_found=true;
                        }
             	//cout<<"A2"<<endl;
              	if (resInfo.ball_found)
              	{
                        	int length=0;
                              	for(int I=i;I<i+temp;I++)
                              	{
                                     if(rgbMat.at<unsigned char>(I, j+temp/4) == 0||rgbMat.at<unsigned char>(I, j-temp/4) == 0)
                                     {
                                         length++;
                                     }
                              	}
                              	//if(length-temp*5/4>0)
                              	//{
                                 // resInfo.ball_found=false;
                                 // continue;
                              	//}/
                        	resInfo.ball_y=i+temp/2;
                         	resInfo.ball_x=j;
                         	cout << "圆心 "<<resInfo.ball_x<<" "<<resInfo.ball_y<<"直径"<<temp<<endl;
                         	
              	}
              		
	}
	}
	}
	else if (mode == MODE_LINE)
	{
		//TODO Write down your code
		//update the resInfo

		int left = 0, right = 0, mid;
		vector<int> midX;
		vector<int> midY;

		// midX.push_back(rgbMat.rows-1);
		// midY.push_back(rgbMat.cols/2);
		for (int i = 0; i < rgbMat.rows; i += 10)
		{
			for (int j = 0; j < rgbMat.cols; j++)
			{
				if (rgbMat.at<unsigned char>(i, j) == 255)
				{
					left = j;
					break;
				}
			}
			for (int j = rgbMat.cols - 1; j > left; j--)
			{
				if (rgbMat.at<unsigned char>(i, j) == 255)
				{
					right = j;
					break;
				}
			}
			if (right != 0)
			{
				mid = (left + right) / 2;

				midX.push_back(i);
				midY.push_back(mid);

			}
		}

		double sumX = 0, sumY = 0;
		int size = midX.size();
		for (int i = 0; i < size; i++)
		{
			sumX += midX[i];
			sumY += midY[i];
		}
		double x = (double)rgbMat.rows - sumX / (double)size;
		double y = (double)rgbMat.cols / 2.0 - sumY / (double)size;
		resInfo.stepX = x / (double)rgbMat.rows;
		resInfo.stepY = - y / (double)rgbMat.rows;
		if (resInfo.stepX < -1 || resInfo.stepX>1)
			resInfo.stepX = 0;
		if (resInfo.stepY < -1 || resInfo.stepY>1)
			resInfo.stepY = 0;
		// if(resInfo.stepY>=1.0 || resInfo.stepY<=-1.0)
		// resInfo.stepY=0;


		bool leftGreen = true, rightGreen = true;
		int count1 = 0,count2 = 0;
		for (int i = 0; i < rgbMat.rows; i++)
		{
			if (rgbMat.at<unsigned char>(i, 0) == 255)
			{
				count1++;
			}
			if (rgbMat.at<unsigned char>(i, rgbMat.cols-1) == 255)
			{
				count2++;
			}
		}
		if(count1<rgbMat.rows/4) leftGreen=false;
		if(count2<rgbMat.rows/4) rightGreen=false;
                    
		resInfo.stepA = 0;
		if (leftGreen==true && rightGreen == false)
			resInfo.stepA = 0.2;
		if (rightGreen==true && leftGreen == false)
		{
			resInfo.stepA = -0.2;
		}
		
	
	}
	rgbMat.release();
	delete[]rgb;
}

// function containing the main feedback loop
void UniRobot::run() 
{

  cout << "---------------SEU-UniRobot-2018---------------" << endl;
  vector<string> modes;
  modes.push_back("1. Ball");
  modes.push_back("2. Line");
  cout << "Run Mode: " <<modes[(int)mode-1]<< endl;
  int break_num_b=0;
  // First step to update sensors values
  myStep();
  int res=0;
  
  // set eye led to green
  mEyeLED->set(0x00FF00);

  // play the hello motion
  mMotionManager->playPage(1); // init position
  mMotionManager->playPage(9); // walkready position
  //wait(200);

  // play the motion preparing the robot to walk
  mGaitManager->start();
  mGaitManager->step(mTimeStep);

  // main loop
  int fup = 0;
  int fdown = 0;
  const double acc_tolerance = 80.0;
  const double acc_step = 20;
  
  while (true) 
  {
    double neckPosition, headPosition;
    const double *acc = mAccelerometer->getValues();
    // count how many steps the accelerometer
    // says that the robot is down
    if (acc[1] < 512.0 - acc_tolerance)
      fup++;
    else
      fup = 0;

    if (acc[1] > 512.0 + acc_tolerance)
      fdown++;
    else
      fdown = 0;

    // the robot face is down
    if (fup > acc_step) 
    {
      mMotionManager->playPage(1); // init position
      mMotionManager->playPage(10); // f_up
      mMotionManager->playPage(9); // walkready position
      fup = 0;
    }
    // the back face is down
    else if (fdown > acc_step) 
    {
      mMotionManager->playPage(1); // init position
      mMotionManager->playPage(11); // b_up
      mMotionManager->playPage(9); // walkready position
      fdown = 0;
    }
    else //*********************************************************//
    {
      imageProcess();

      
      //TODO control the robot according to the resInfo you updated in imageProcess
      //demo
      //kick ball

      if(mode == MODE_BALL) // mode ball 
      {
       // for()
         // headPosition = clamp(0.7, minMotorPositions[19], maxMotorPositions[19]);
          //mMotors[19]->setPosition(headPosition);
        if(resInfo.ball_found)
        {
          res=false;
          break_num_b=0;
          if(resInfo.ball_x<140||(resInfo.ball_x<=180&&resInfo.ball_x>=160)){mGaitManager->setAAmplitude(0.06);}
          else if(resInfo.ball_x>180||(resInfo.ball_x<160&&resInfo.ball_x>=140)){mGaitManager->setAAmplitude(-0.06);}
          if(resInfo.ball_y<=200){mGaitManager->setXAmplitude(0.75);}
          if (resInfo.ball_y > 125)
          {
            headPosition = clamp(-0.24, minMotorPositions[19], maxMotorPositions[19]);
            mMotors[19]->setPosition(headPosition); //x -1.0 ~ 1.0
            res=true;
            if(resInfo.ball_y > 150&&res)
            {
              mGaitManager->stop();
             // wait(500);
              if (resInfo.ball_x >=70&&resInfo.ball_x<160)
              {
                //mGaitManager->stop();
                mMotionManager->playPage(13); // left kick
                imageProcess();
              }
              else if(resInfo.ball_x >=160&&resInfo.ball_x<250)
              {

                mMotionManager->playPage(12); // right kick
                imageProcess();
              }
              headPosition = clamp(0.5, minMotorPositions[19], maxMotorPositions[19]);
                mMotors[19]->setPosition(headPosition);
                imageProcess();
              mMotionManager->playPage(9); // walkready position
              //neckPosition = clamp(-0.3, minMotorPositions[18], maxMotorPositions[18]);
              //mMotors[18]->setPosition(neckPosition);
              mGaitManager->start();
            }
          }
        }
        else
        {
            break_num_b++;
             mGaitManager->setAAmplitude(0.0);
           // mGaitManager->setAAmplitude(0.0);
            if(break_num_b>20&&resInfo.ball_y>200)
            {
              mGaitManager->setXAmplitude(-0.7);
              break_num_b=0;
            }
            if(break_num_b>40)
            {
              res=false;
              headPosition = clamp(0.3, minMotorPositions[19], maxMotorPositions[19]);
              mMotors[19]->setPosition(headPosition);
              if(resInfo.ball_x >=200&&break_num_b>30)
              {
                mGaitManager->setAAmplitude(-0.04);
                headPosition = clamp(-0.1, minMotorPositions[19], maxMotorPositions[19]);
                mMotors[19]->setPosition(headPosition);
                imageProcess();
                if(resInfo.ball_found) continue;
              }
              if(resInfo.ball_x <=120&&break_num_b>30)
              {
                mGaitManager->setAAmplitude(0.04);
                headPosition = clamp(-0.1, minMotorPositions[19], maxMotorPositions[19]);
                mMotors[19]->setPosition(headPosition);
                if(resInfo.ball_found) continue;
              }
            }
        }
    
        mGaitManager->step(mTimeStep);
      }
      else if(mode == MODE_LINE) //mode line
      {
        //walk control
        cout<<resInfo.stepY<<' '<<resInfo.stepX<<' '<<resInfo.stepA<<endl;
        mGaitManager->setXAmplitude(resInfo.stepX); //x -1.0 ~ 1.0
        mGaitManager->setYAmplitude(resInfo.stepY); //y -1.0 ~ 1.0
        mGaitManager->setAAmplitude(resInfo.stepA); //dir -1.0 ~ 1.0
        mGaitManager->setBalanceEnable(true);
        mGaitManager->step(mTimeStep);
		
        //head control
        neckPosition = clamp(0.0, minMotorPositions[18], maxMotorPositions[18]); //head yaw position
        headPosition = clamp(0.0, minMotorPositions[19], maxMotorPositions[19]); //head pitch position
        mMotors[18]->setPosition(neckPosition);
        mMotors[19]->setPosition(headPosition);
      }
    }
    myStep();
  }
}

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

UniRobot::UniRobot(int argc, char **argv):
    Robot()
{
  mTimeStep = getBasicTimeStep();

  mEyeLED = getLED("EyeLed");
  mHeadLED = getLED("HeadLed");
  mHeadLED->set(0x00FF00);
  mBackLedRed = getLED("BackLedRed");
  mBackLedGreen = getLED("BackLedGreen");
  mBackLedBlue = getLED("BackLedBlue");
  mCamera = getCamera("Camera");
  mCamera->enable(2*mTimeStep);
  image_w = mCamera->getWidth();
  image_h = mCamera->getHeight();
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  mGyro = getGyro("Gyro");
  mGyro->enable(mTimeStep);
  mDisplay = getDisplay("Display");

  for (int i=0; i<NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
    minMotorPositions[i] = mMotors[i]->getMinPosition();
    maxMotorPositions[i] = mMotors[i]->getMaxPosition();
  }

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
  
  string s(argv[1]);
  mode = (RunMode)(stoi(s));
}

UniRobot::~UniRobot() 
{
}

void UniRobot::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void UniRobot::wait(int ms) 
{
  double startTime = getTime();
  double s = (double) ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

//param @rgb raw data in format of RGB 
void UniRobot::showImage(const unsigned char *rgb)
{
  int w = mDisplay->getWidth();
  int h = mDisplay->getHeight();
  Mat src(image_h, image_w, CV_8UC3, (unsigned char*)rgb);
  Mat dst;
  resize(src, dst, Size(w,h));
  ImageRef *img = mDisplay->imageNew(w, h, dst.data, Display::RGB);
  src.release();
  dst.release();
  mDisplay->imagePaste(img, 0, 0);
}

unsigned char* UniRobot::getRGBImage()
{
  unsigned char *im = (unsigned char*)(mCamera->getImage()); //image format: BGRA
  unsigned char *rgb = new unsigned char[3*image_w*image_h];
  unsigned int oidx=0, aidx=0;
  
  for(int y=0;y<image_h;y++)
  {
    for(int x=0;x<image_w;x++)
    {
      rgb[aidx+0] = im[oidx+2]; //R
      rgb[aidx+1] = im[oidx+1]; //G
      rgb[aidx+2] = im[oidx+0]; //B
      oidx += 4;
      aidx += 3;
    }
  }
  return rgb;
}

Mat UniRobot::getRGBMat()
{
  unsigned char *rgb = getRGBImage();
  Mat res(image_h, image_w, CV_8UC3, rgb);
  delete []rgb;
  return res;
}