/* colorTrack.cpp
Author: Tie Hu
Date: 07/04/2020
Edge detection and find the lines
Find the middle line 
plot the cross line and plot the center point 
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

/*Adjust Brightness*/
Mat adjustBrightness(Mat& I, int rtype, double alpha, double beta)
{
  Mat adjustedImage;
  I.convertTo(adjustedImage, rtype , alpha, beta);
  return adjustedImage;
}

/*Histogram Plot*/
void plotHist(Mat &I, const string windowname)
{
    vector<Mat> bgr_planes;
    split( I, bgr_planes );
    int histSize = 256;
    float range[] = { 0, 256 }; //the upper boundary is exclusive
    const float* histRange = { range };
    bool uniform = true, accumulate = false;
    Mat b_hist, g_hist, r_hist;
    calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ),
              Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
              Scalar( 255, 0, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ),
              Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
              Scalar( 0, 255, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ),
              Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
              Scalar( 0, 0, 255), 2, 8, 0  );
    }
    imshow(windowname, histImage );
}

/*Find Lines*/
void findLines(Mat& I, Mat& colorImage)
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
  float rho_gap = 16.0, rho_error = 1.5, theta_error = 0.001, theta_ortho = M_PI/2;
  bool findLine[4] = {0,0,0,0};
  float klines[4][2];
  float midLine[2][2];
  
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
	  cout <<"abs(theta-theta_0):" << abs(theta-theta_0) <<endl;
	  cout << "theta_error:" << theta_error << endl;
	  if (abs(theta-theta_0) <= theta_error)
	    {
	      if((abs(abs(rho_0-rho)-rho_gap) <= rho_error)&&(findLine[0]==0))
		{
		  findLine[0] =1;
		  klines[0][0] = rho_0;
		  klines[0][1] = theta_0;
		  klines[1][0] = lines[j][0];
		  klines[1][1] = lines[j][1];
		  cout << "klines[0][0]:" << klines[0][0] << endl;
		  cout << "klines[0][1]:" << klines[0][1] << endl;
		  cout << "klines[1][0]:" << klines[1][0] << endl;
		  cout << "klines[1][1]:" << klines[1][1] << endl;
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
		  klines[2][0] = lines[j][0];
		  klines[2][1] = lines[j][1];
		  cout << "klines[2][0]" << klines[2][0] << endl;
		  cout << "klines[2][1]" << klines[2][1] << endl;
		}
	      else if((findLine[1] == 1)&&(findLine[2] == 0))
		{
		  findLine[2] = 1;
		  klines[3][0] = lines[j][0];
		  klines[3][1] = lines[j][1];
		  cout << "klines[3][0]" << klines[3][0] << endl;
		  cout << "klines[3][1]" << klines[3][1] << endl; 
		}
	      cout << "j =" << "Finish first loop" << endl;
	    }
      
        }
      
  }
  
  for( size_t i = 0; i < 4; i++ )
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

  //find the midlle line
  midLine[0][0] = (klines[0][0] + klines[1][0])/2;
  midLine[0][1] = klines[0][1];

  midLine[1][0] = (klines[2][0]+ klines[3][0])/2;
  midLine[1][1] = klines[2][1];

  //plot the middle line
  for( size_t i = 0; i < 2; i++ )
    {
        float rho = midLine[i][0], theta = midLine[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
	cout<< midLine[i][0]<<":"<<midLine[i][1]<<endl;
        line(colorImage , pt1, pt2, Scalar(0,255,0), 3, LINE_AA);
    }

  //find the intersection of two lines
  float x1, y1, x2, y2, x3, y3, x4, y4;
  Point Pinter;

  for( size_t i = 0; i < 2; i++)
    {
        float rho = midLine[i][0], theta = midLine[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
	if (i == 0)
	  {
	    x1 = pt1.x;
	    y1 = pt1.y;
	    x2 = pt2.x;
	    y2 = pt2.y;
	  }
	else if ( i ==1)
	  {
	    x3 = pt1.x;
	    y3 = pt1.y;
	    x4 = pt2.x;
	    y4 = pt2.y;
	  }
    }
  Pinter.x = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
  Pinter.y = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
  cout << "Point Intersection x:" << Pinter.x << endl;
  cout << "Point Intersection y:" << Pinter.y << endl;
  circle(colorImage,
	 Pinter,
	 100,
	 Scalar(255, 0, 0),
	 10,
	 LINE_8);

  //measure the angle of the line
  double angleLine;
  angleLine = midLine[0][1]/M_PI*180.0;
  cout << "The angle of the cross line is:" << angleLine << endl;
 
  string angleText = "The Offset Angle is: " + to_string(angleLine) ;
  string posText = " The Center Pos is: " + to_string(Pinter.x) +" , " + to_string(Pinter.y);
  
  Point angleTextPos(100,200);
  Point posTextPos(100,150);

  
  //output the measuring results on the image
  putText(colorImage,
	  angleText,
	  angleTextPos,
	  0,
	  1,
	  Scalar(0,255,0),
	  2,
	  LINE_8,
	  false);
  putText(colorImage,
	  posText,
	  posTextPos,
	  0,
	  1,
	  Scalar(0,255,0),
	  2,
	  LINE_8,
	  false); 
}

Mat findroiImage(Mat& I, Point start, Point end)
{
  Rect Rec(start.x, start.y, end.x, end.y);
  Mat Roi = I(Rec);
  return Roi;
}

vector<Vec2f> knearestLines(Mat& I, vector<Vec2f> lines)
{
  float rho_0, theta_0, rho, theta;
  float rho_gap = 16.0, rho_error = 0.1, theta_error = 0.01, theta_ortho = M_PI/2;
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

void templateSearch(Mat& I)
{
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
    Mat image, colorImage, colorImage_02,  grayImage, blurImage, edgeImage,
      roiImage, brightImage;
    string displaywindowname = "Display Window";
    string roiwindowname = "ROI window";
    string blurwindowname = "Gaussian filter window";
    string edgewindowname = "Canny Edge window";
    string linewindowname = "plot lines window";
    string brightwindowname = "brightness window";
    string originalhistwindowname = "Histogram window (original)";
    string adjusthistwindowname = "Histogram window(adjusted)";
    string inputPath = "./image/mv/";
    string outputPath = "./image/mv/output/";
    string inputFile = inputPath + argv[1];
    string outputFile = outputPath + "output_"+argv[1];
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
    int rtype = -1;
    double alpha = 1.0, beta = 0.0;
    Point boxStart, boxEnd, roiStart, roiEnd ;
  
    vector<Vec2f> klines;
    vector<Vec2f> flines;
    
    boxStart = Point(300, 400);
    boxEnd = Point(500, 600);
    roiStart = boxStart;
    roiEnd = boxEnd;
    //image = imread( argv[1], 1 );

    /*Read Image*/
    int size_n = inputFile.length();
    int size_m = outputFile.length();
    /*define the character array*/
    char inputFileChar[size_n+1];
    char outputFileChar[size_m+1];
    /*copy the content of string to the character array*/
    strcpy(inputFileChar, inputFile.c_str());
    strcpy(outputFileChar, outputFile.c_str());
    readImage(image, inputFileChar);
    
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
     
    cout << "Default lowThreshold = " << lowThreshold << endl;
    cout << "Default HighThreshold = " << lowThreshold*ratio << endl;
    cout << "Default kernel_size = " << kernel_size << endl;
    cout << "Default alpha value = "<< alpha << endl;
    cout << " Default beta value = " << beta << endl;
    cout << " New low Threshold:" << endl;
    cin >> lowThreshold;
    cout << "New Ratio: " << endl;
    cin >> ratio;
    cout << "New Kernel_size: " << endl;
    cin >> kernel_size;
    cout << "New alpha :" << endl;
    cin >> alpha;
    cout << "New beta :" << endl;
    cin >> beta;

    /*color Image*/
    colorImage = image.clone();
    /*change the brightness of image*/
    brightImage = adjustBrightness(colorImage, rtype, alpha, beta);
    brightImage.copyTo(colorImage);    
    colorImage_02 = image.clone();
    /*plot original histogram*/
    plotHist(colorImage_02, originalhistwindowname);
    /*plot adjusted histogram*/
    plotHist(colorImage, adjusthistwindowname);
    /*Roi Image*/
    roiImage = findroiImage(colorImage, roiStart, roiEnd);
    /*Convert to gray image*/
    grayImage = convertGrayImage(colorImage);
    /*apply gaussian blur filter*/
    blurImage = smoothImage(grayImage, ksize);
    edgeImage = blurImage.clone();

    /*find canny edge*/
    edgeImage = cannyEdgeImage(edgeImage, lowThreshold, ratio, kernel_size);
    findLines(edgeImage, colorImage);
    string titleText = "Find the center of Cross Line and Offset Angle for file " + inputFile;
    Point titleTextPos(100, 100);
     putText(colorImage,
	  titleText,
	  titleTextPos,
	  0,
	  1,
	  Scalar(0,255,0),
	  2,
	  LINE_8,
	  false); 
       
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
    showImage(brightwindowname, brightImage); 
    showImage(edgewindowname, edgeImage);
    showImage(roiwindowname, roiImage);
    showImage(linewindowname, colorImage);
    imwrite(outputFileChar, colorImage);
    waitKey(0);
    return 0;
}
