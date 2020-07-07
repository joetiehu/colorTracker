/* colorTrack.cpp
Author: Tie Hu
Date: 07/04/2020
Edge detection and find the lines 
*/
#define _USE_MATH_DEFINES

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/*Read Image*/
void readImage(Mat& I, char* filename)
{
  I = imread(filename, 1);
}

/*Convert Image to Gray Scale*/
Mat convertGrayImage(Mat& I)
{
  Mat grayImage;
  cvtColor(I, grayImage, CV_BGR2GRAY);
  return grayImage;
}

/*Size of Image*/
Size sizeOfImage(Mat& I)
{
  Size s;
  int nrows;
  int ncolumns;
  s = I.size();
  nrows = s.height;
  ncolumns = s.width;
  cout << " Image Height: " << nrows << endl;
  cout << " Image Width : " << ncolumns << endl;
  return s;
}

/*Display image in a window*/
void showImage(const string& windowname, Mat& I)
{
  namedWindow(windowname,WINDOW_AUTOSIZE);
  imshow(windowname, I);
}

/*Smooth Image*/
Mat smoothImage(Mat& I, int ksize)
{
  Mat smoothImage;
  GaussianBlur(I, smoothImage, Size(ksize, ksize),0, 0, BORDER_DEFAULT);
  return smoothImage;
}

/*Find Edge*/
Mat cannyEdgeImage(Mat& I, int lowThreshold, int ratio, int kernel_size)
{
  Canny(I, I, lowThreshold, lowThreshold*ratio, kernel_size);
  return I;
}

/*Find Lines*/
vector<Vec2f> findLines(Mat& I, Mat& colorImage)
{
    vector<Vec2f> lines; // will hold the results of the detection
    HoughLines(I, lines, 1, CV_PI/180, 200, 0, 0 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
	cout<< lines[i][0]<<":"<<lines[i][1]<<endl;
        //line(colorImage , pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    }

    float rho_0, theta_0, rho, theta;
  float rho_gap = 16.0, rho_error = 0.1, theta_error = 0.1, theta_ortho = M_PI/2;
  bool findLine[4] = {0,0,0,0};
  vector<Vec2f> klines;
  
  for (size_t i = 0; i < lines.size(); i++)
  {
      rho_0 = lines[i][0];
      theta_0 = lines[i][1];
      cout << "i =" << i << endl;
      for(size_t j = (i+1); j < lines.size(); j++)
	{
	  rho = lines[j][0];
	  theta = lines[j][1];
	  cout << "j =" << j << endl;
	  cout <<"abs(theta-theta_0)" << abs(theta-theta_0) <<endl;
	  cout << "theta_error" << theta_error << endl;
	  if (abs(theta-theta_0) <= theta_error)
	    {
	      if(abs(abs(rho_0-rho)-rho_gap) <= rho_error)
		{
		  findLine[0] =1;
		  klines[0][0] = rho_0;
		  klines[0][1] = theta_0;
		  klines[1][0] = lines[j][0];
		  klines[1][1] = lines[j][1];
		}
	     }

	  cout << "thta-theta_0" << abs(abs(theta-theta_0)-theta_ortho) << endl;
	  if (abs(abs(theta-theta_0)-theta_ortho) <= theta_error)
	    {
	      if(findLine[1] == 0)
		{
		  cout << "findLine[1]:"<< findLine[1] << endl;
		  findLine[1] = 1;
		  cout << "findLine[1]"<< findLine[1]<< endl; 
		  //klines[2][0] = lines[j][0];
		  //klines[2][1] = lines[j][1];
		  cout <<"klines[2][0]" <<  endl;
		}
	      else if(findLine[1] == 1)
		{
		  findLine[2] = 1;
		  klines[3][0] = lines[j][0];
		  klines[3][1] = lines[j][1];
		}
	      cout << "j =" << "Finish first loop" << endl;
	    }
      
        }
  }
  
  for( size_t i = 0; i < klines.size(); i++ )
    {
        float rho = klines[i][0], theta = klines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
	cout<< klines[i][0]<<":"<<klines[i][1]<<endl;
        line(colorImage , pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    }

  return klines;
          
}

void roiImage(Mat& I, Point start, Point end)
{
  
}

vector<Vec2f> knearestLines(Mat& I, vector<Vec2f> lines)
{
  float rho_0, theta_0, rho, theta;
  float rho_gap = 16.0, rho_error = 0.1, theta_error = 0.1, theta_ortho = M_PI/2;
  bool findLine[4] = {0,0,0,0};
  vector<Vec2f> klines;
  
  for (size_t i =0; i < lines.size(); i++)
  {
      rho_0 = lines[i][0];
      theta_0 = lines[i][1];
      for(size_t j = (i+1); j < lines.size(); j++)
	{
	  rho = lines[j][0];
	  theta = lines[j][1];
	  if (abs(theta-theta_0) <= theta_error)
	    {
	      if(abs(abs(rho_0-rho)-rho_gap) <= rho_error)
		{
		  findLine[0] =1;
		  klines[0][0] = rho_0;
		  klines[0][1] = theta_0;
		  klines[1][0] = lines[j][0];
		  klines[1][1] = lines[j][1];
		}
	     }
	  if (abs(abs(theta-theta_0)-theta_ortho) <= theta_error)
	    {
	      if(findLine[1] == 0)
		{
		  findLine[1] =1;
		  klines[2][0] = lines[j][0];
		  klines[2][1] = lines[j][1];
		}
	      else if(findLine[1] == 1)
		{
		  findLine[2] = 1;
		  klines[3][0] = lines[j][0];
		  klines[3][1] = lines[j][1];
		}
	    }
      
        }
  }
  
  for( size_t i = 0; i < klines.size(); i++ )
    {
        float rho = klines[i][0], theta = klines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
	cout<< klines[i][0]<<":"<<klines[i][1]<<endl;
        line(I , pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    }

  return klines;
}

void findLineIntersection(Mat& I, vector<Vec2f> Lines)
{
}

void findMiddleLine(Mat& I, vector<Vec2f> Lines)
{
}
  
/*Draw a box */
void drawBox(Mat& I, Point start, Point end)
{
  int thickness = 2;
  int lineType = LINE_8;
  int shiftcode = 0;
  rectangle(
	    I,
	    start,
	    end,
	    Scalar(0, 255, 0),
	    thickness,
	    lineType,
	    shiftcode
	    );
}

/*Draw a Line*/
void drawLine(Mat& I, Point start, Point end)
{
  int thickness = 2;
  int lineType = LINE_8;

  line(
       I,
       start,
       end,
       Scalar(0,255,0),
       thickness,
       lineType
       );
}

int main(int argc, char** argv)
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    Mat image, colorImage, colorImage_02,  grayImage, blurImage, edgeImage;
    string displaywindowname = "Display Window";
    string roiwindowname = "ROI window";
    string blurwindowname = "Gaussian filter window";
    string edgewindowname = "Canny Edge window";
    string linewindowname = "plot lines wndow";
    Size s;
    int nrows;
    int ncolumns;
    int center_nrows;
    int center_ncolumns;
    int width_box;
    int height_box;
    int x_boxStart, y_boxStart, x_boxEnd, y_boxEnd;
    int ksize = 9;
    int lowThreshold = 20;
    int ratio = 3;
    int kernel_size = 3;
    Point boxStart, boxEnd ;

    vector<Vec2f> klines;
    vector<Vec2f> flines;
    
    boxStart = Point(300, 400);
    boxEnd = Point(500, 600);
    //image = imread( argv[1], 1 );

    /*Read Image*/
    readImage(image, argv[1]);

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    /*color Image*/
    colorImage = image.clone();
    colorImage_02 = image.clone();
    /*Convert to gray image*/
    grayImage = convertGrayImage(colorImage);
    /*apply gaussian blur filter*/
    blurImage = smoothImage(grayImage, ksize);
    edgeImage = blurImage.clone();
    /*find canny edge*/
    cout << "Default lowThreshold = " << lowThreshold << endl;
    cout << "Default HighThreshold = " << lowThreshold*ratio << endl;
    cout << "Default kernel_size = " << kernel_size << endl;
    cout << " New low Threshold:" << endl;
    cin >> lowThreshold;
    cout << "New Ratio: " << endl;
    cin >> ratio;
    cout << "New Kernel_size: " << endl;
    cin >> kernel_size;
    
    edgeImage = cannyEdgeImage(edgeImage, lowThreshold, ratio, kernel_size);
    flines = findLines(edgeImage, colorImage);
    //klines = knearestLines(colorImage_02, flines);
    //drawLine(image, x, y);
    //Draw a Region Box
    //drawBox(image, x, y);
    //s = image.size();
    s = sizeOfImage(image);
    nrows = s.height;
    ncolumns = s.width;
    center_nrows = 0.5 * nrows;
    center_ncolumns = 0.5 * ncolumns;
    cout << " Please Enter width_box " << endl;
    cin >> width_box;
    cout << " Please Enter height_box " << endl;
    cin >> height_box;
    
    x_boxStart = int(center_ncolumns-0.5*width_box);
    y_boxStart = int(center_nrows-0.5*height_box);
    x_boxEnd = int(center_ncolumns+0.5*width_box);
    y_boxEnd = int(center_nrows+0.5*height_box);
    cout << "x_boxStart" << x_boxStart << endl;
    cout << "y_boxStart" << y_boxStart << endl;
    cout << "x_boxEnd" << x_boxEnd << endl;
    cout << "y_boxEnd" << y_boxEnd << endl;
    
    boxStart = Point(x_boxStart, y_boxStart);
    boxEnd = Point(x_boxEnd, y_boxEnd);
    
    cout << "center of image height:" << center_nrows << endl;
    cout << "center of iamge width: " << center_ncolumns << endl;

    drawBox(image, boxStart, boxEnd);
    
    showImage(displaywindowname,image);
    //namedWindow("Display Image", WINDOW_AUTOSIZE );
    //imshow("Display Image", image);
    showImage(blurwindowname, blurImage);
    showImage(edgewindowname, edgeImage);
    showImage(linewindowname, colorImage);
    waitKey(0);
    return 0;
}
