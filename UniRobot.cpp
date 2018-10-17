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
	cvtColor(rgbMat, rgbMat, COLOR_BGR2GRAY);//转灰度图
	threshold(rgbMat, rgbMat, 2000, 255, THRESH_BINARY);//二值化
	cout<<"A1"<<endl;
	
	if (mode == MODE_BALL)
  	{
  	cout<<"A2"<<endl;
  	Mat dst_img;
  	cout<<"A"<<endl;	
	// 形态学操作
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	// 构建形态学操作的结构元
	cout<<"A"<<endl;
	morphologyEx(rgbMat, dst_img, MORPH_CLOSE, kernel, Point(-1, -1));
	//闭操作
	
	kernel = getStructuringElement(MORPH_RECT, Size(5,5), Point(-1, -1));
	// 构建形态学操作的结构元
	morphologyEx(dst_img, dst_img, MORPH_OPEN, kernel, Point(-1, -1));
	//开操作
	
	// 寻找轮廓
	vector<vector<Point>> contours;
	vector<Vec4i> hireachy;
	findContours(dst_img, contours, hireachy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	Mat result_img = Mat::zeros(rgbMat.size(), CV_8UC3);
	// 创建与原图同大小的黑色背景
	Point circle_center;
	//定义圆心坐标
	
	for (auto t = 0; t < contours.size(); ++t)
	{
		// 面积过滤
		double area = contourArea(contours[t]);
		//计算点集所围区域的面积
		
		// 横纵比过滤
		Rect rect = boundingRect(contours[t]);
		// 求点集的最小直立外包矩形
		float ratio = float(rect.width) / float(rect.height);
		//求出宽高比
		//if (ratio < 1.1 && ratio > 0.9)       //因为圆的外接直立矩形肯定近似于一个正方形，因此宽高比接近1.0
		
			printf("圆的面积: %f\n", area);
			double arc_length = arcLength(contours[t], true);
			//计算点集所围区域的周长
			printf("圆的周长 : %f\n", arc_length);
			int x = rect.x + rect.width / 2;
			int y = rect.y + rect.height / 2;
			circle_center = Point(x, y);
			//得到圆心坐标
			cout << "圆心坐标：" << "宽" << circle_center.x << " " << "高" << circle_center.y << endl;
		
		
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
	//rgb=RGB(rgbMat);
	showImage(rgb);
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

  // First step to update sensors values
  myStep();

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

/*unsigned char* UniRobot::RGB(Mat rgbMat)
{
  unsigned char *rgb=new unsigned char[3*image_w*image_h];
  int rows=rgbMat.rows,cols=rgbMat.cols;
  for(int i=0;i<rows;++i)
  {
    for(int j=0;j<cols;++j)
    {
      rgb[i*rows+cols+0]=rgbMat.at<Vec3b>(i,j)[0];
      rgb[i*rows+cols+1]=rgbMat.at<Vec3b>(i,j)[1];
      rgb[i*rows+cols+2]=rgbMat.at<Vec3b>(i,j)[2];
    }
  }
  return rgb;
}
*/
Mat UniRobot::getRGBMat()
{
  unsigned char *rgb = getRGBImage();
  Mat res(image_h, image_w, CV_8UC3, rgb);
  delete []rgb;
  return res;
}