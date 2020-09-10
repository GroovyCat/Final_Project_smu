#include "yoloNet.hpp"
#include <windows.h>
#include <WinBase.h>

using namespace cv;
using namespace std;

#define HEIGHT_RATE 3
#define WIDTH_RATE 4
#define MAX_ENABLE_ANGLE 65
#define MIN_ENABLE_ANGLE 20

Rect global_roi;
const int General_Left_Lane_Angle = 40;
const int General_Right_Lane_Angle = 40;

// 관심영역 지정
Mat Find_ROI(Mat frame) {
	// Get ROI

	Rect roi;
	roi.x = frame.cols / WIDTH_RATE;
	roi.y = frame.rows - (frame.rows / HEIGHT_RATE);
	roi.width = roi.x * 2;
	roi.height = frame.rows / HEIGHT_RATE;
	roi.height -= frame.rows / HEIGHT_RATE / 3;	// 자동차 보닛 없애기
	Mat roi_img = frame(roi);

	global_roi = roi;

	return roi_img;
}


// 캐니엣지 검출
Mat Detect_Edge(Mat roi) {
	// Convert Color to Gray
	Mat gray_img;
	cvtColor(roi, gray_img, COLOR_BGR2HSV);

	// Remove Noise
	Mat RNoise_img;
	uchar data[] = {
		0,1,0,
		1,1,1,
		0,1,0
	};
	Mat mask(3, 3, CV_8UC1, data);
	morphologyEx(gray_img, RNoise_img, MORPH_RECT, mask);

	// 이진화
	Mat thresh;
	threshold(RNoise_img, thresh, 130, 255, THRESH_BINARY);

	morphologyEx(thresh, thresh, MORPH_RECT, mask);

	// Canny Edge
	Mat Canny_img;
	Canny(thresh, Canny_img, 125, 255, 3);

	return Canny_img;
}



// 좌우 차선 각도 구하기
double Find_Angle(Mat roi) {
	double angleSum = 0.0;	// 평균 내기 위한 좌우 차선 각도 값들의 합
	int count = 0;	// 차선의 각도가 비정상적이라면. 각도를 더하지 않음. 평균 낼때 size()에서 더하지않는 수만큼 뺌.

	// 차선 평균 기울기 
	vector<Vec4i> lines;
	int threshold = 50;
	HoughLinesP(roi, lines, 1, CV_PI / 180, threshold);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		double angle = atan2(l[3] - l[1], l[2] - l[0]) * 180.0 / CV_PI;
		if (MAX_ENABLE_ANGLE > abs(angle) && abs(angle) > MIN_ENABLE_ANGLE) {
			angleSum += angle;
		}
		else {
			count++;
		}
	}

	// 각도 평균내기
	return angleSum / (lines.size() - count);
}


Mat First_Hough_Lane_Func(Mat roi, double angle) {
	// 1차 확률적 허프변환
	vector<Vec4i> lines;
	int threshold = 50; // r,θ 평면에서 몇개의 곡선이 한점에서 만났을 때 직선으로 판단할지에 대한 최소값
	HoughLinesP(roi, lines, 1, CV_PI / 180, threshold);

	//Mat Hough_img = gray_img.clone();
	Mat Hough_img(roi.rows, roi.cols, CV_8UC1, Scalar(0, 0, 0));

	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		double angle_ = atan2(l[3] - l[1], l[2] - l[0]) * 180.0 / CV_PI;
		if (angle_ >= (angle - 1.0) && angle_ <= (angle + 1.0)) {
			line(Hough_img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 1, LINE_AA);
		}
	}
	return Hough_img;
}

void Lane_Func(Mat Hough_img) {
	// 2차 허프변환
	vector<Vec2f> lines;
	int threshold2 = 70; // r,θ 평면에서 몇개의 곡선이 한점에서 만났을 때 직선으로 판단할지에 대한 최소값
	HoughLines(Hough_img, lines, 1, CV_PI / 180, threshold2);

	Mat dstImage(Hough_img.size(), CV_8UC3);
	cvtColor(Hough_img, dstImage, COLOR_GRAY2BGR);

	Vec2f params;
	float rho, theta;
	float c, s;
	float x0, y0;
	// for(int k=0; k<lines.cols; k++)
	for (int k = 0; k < lines.size(); k++)
	{
		//  params = lines.at<Vec2f>(0, k);
		params = lines[k];

		rho = params[0];
		theta = params[1];
		// printf("lines[%2d]=(rho, theta) = (%f, %f)\n", k, rho, theta);

		// drawing a line
		c = cos(theta);
		s = sin(theta);
		x0 = rho * c;
		y0 = rho * s;

		Point pt1(0, 0), pt2(0, 0);
		pt1.x = cvRound(x0 + 1000 * (-s));
		pt1.y = cvRound(y0 + 1000 * (c));

		pt2.x = cvRound(x0 - 1000 * (-s));
		pt2.y = cvRound(y0 - 1000 * (c));

		line(dstImage, pt1, pt2, Scalar(0, 0, 255), 1, LINE_AA);
	}
}

vector<Point> Virtual_Lane(Point2d angle) {
	Rect roi = global_roi;

	Point center;
	center.x = roi.width / 2 + roi.x;
	center.y = roi.height + roi.y;
	//circle(frame, center, 5, Scalar(0, 0, 255), -1);

	// 그리기
	Point p1, p2, p3, p4;
	p4.x = center.x - roi.width / 2.5;
	p3.x = center.x + roi.width / 2.5;
	p1.y = p2.y = center.y - roi.height + 10;
	p4.y = p3.y = center.y;

	int height = p3.y - p2.y;	// 마름모 높이

	// 각도에 따른 차선 그리기
	if (isnan(angle.x)) {
		//p1.x = p4.x + roi.width / 2.8;
		p1.x = p4.x + abs(height / tan(abs(General_Left_Lane_Angle) * CV_PI / 180));
	}
	else {
		p1.x = p4.x + abs(height / tan(abs(angle.x) * CV_PI / 180));
		if (p1.x > center.x) {
			//p1.x = p4.x + roi.width / 2.8;
			p1.x = p4.x + abs(height / tan(abs(General_Left_Lane_Angle) * CV_PI / 180));
		}
	}

	if (isnan(angle.y)) {
		//p2.x = p3.x - roi.width / 2.8;
		p2.x = p3.x - abs(height / tan(abs(General_Right_Lane_Angle) * CV_PI / 180));
	}
	else {
		p2.x = p3.x - abs(height / tan(abs(angle.y) * CV_PI / 180));
		if (p2.x < center.x) {
			//p2.x = p3.x - roi.width / 2.8;
			p2.x = p3.x - abs(height / tan(abs(General_Right_Lane_Angle) * CV_PI / 180));
		}
	}

	vector<Point> Position(4);
	Position[0] = p1;
	Position[1] = p2;
	Position[2] = p3;
	Position[3] = p4;

	return Position;
}

Point Rhombus(Mat Lroi, Mat Rroi, Point2d angle) {
	Point a;

	return a;
}


bool Lane_In_Person_Recognition(vector<Point>lane, Point2d angle, Rect Object)
{
	Point Obj_LB(Object.x, Object.y + Object.height);					// Object RoundBox Left Bottom
	Point Obj_RB(Object.x + Object.width, Object.y + Object.height);	// Object RoundBox Right Bottom
	Point Left_Tri(lane[0].x, lane[3].y);	// Left Triangle 
	Point Right_Tri(lane[1].x, lane[2].y);	// Right Triangle

	// 왼쪽 삼각형
	int b1 = lane[0].y - abs(angle.x) * lane[0].x;
	if (Obj_RB.y < (abs(angle.x) * Obj_RB.x + b1)) {		// 직선 안쪽에 있을 때
		if (Obj_RB.x < Left_Tri.x && Obj_RB.y < Left_Tri.y) {	// 삼각형 직각부분 꼭지점
			if (Obj_RB.x > lane[3].x && Obj_RB.y > lane[0].y) {	// 삼각형 왼쪽아래 꼭지점, 삼각형 오른쪽위 꼭지점
				return true;
			}
		}
	}

	// 오른쪽 삼각형
	int b2 = lane[1].y - (180 - abs(angle.y)) * lane[1].x;
	if (Obj_LB.y < (abs(angle.y) * Obj_LB.y + b2)) {		// 직선 안쪽에 있을 때
		if (Obj_LB.x > Right_Tri.x && Obj_LB.y < Right_Tri.y) {	// 삼각형 직각부분 꼭지점
			if (Obj_LB.x < lane[2].x && Obj_LB.y > lane[1].y) {	// 삼각형 오른쪽아래 꼭지점, 삼각형 왼쪽위 꼭지점
				return true;
			}
		}
	}

	// 가운데 사각형
	if (Obj_LB.x > lane[0].x && Obj_LB.x < lane[1].x) {
		if (Obj_LB.y > lane[0].y && Obj_LB.y < lane[3].y) {
			//cout << "center detected" << endl;
			return true;
		}
	}

	// 가운데 삼각형
	if (Obj_RB.x > lane[0].x && Obj_RB.x < lane[1].x) {
		if (Obj_RB.y > lane[0].y && Obj_RB.y < lane[3].y) {
			//cout << "center detected" << endl;
			return true;
		}
	}

	return false;
}

void consoleMove(int shiftX, int shiftY)
{
	CONSOLE_SCREEN_BUFFER_INFO coninfo;
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	GetConsoleScreenBufferInfo(hConsole, &coninfo);
	coninfo.dwCursorPosition.Y += shiftY;    // move up one line
	coninfo.dwCursorPosition.X += shiftX;    // move to the right the length of the word
	SetConsoleCursorPosition(hConsole, coninfo.dwCursorPosition);
}

void drawBoundingBox(Mat &img, int id, float confidence, Rect box)
{
    // Draw rectangle
    int top = box.y - box.height/2;
    int left = box.x - box.width/2;
    rectangle(img, Rect(left, top, box.width, box.height), Scalar(0,255,0), 3);

    // Create the label text
    String labelTxt = format("%.2f",confidence);
	String idTxt = format("person [%d] ", id);
    labelTxt = idTxt + labelTxt;

    // Draw the label text on the image
    int baseline;
    Size labelSize = getTextSize(labelTxt, FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
    top = max(top, labelSize.height);
    rectangle(img, Point(left,top - labelSize.height), Point(left + labelSize.width, top + baseline), Scalar(0,0,0), FILLED);
    putText(img, labelTxt, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
}

std::string keys =
"{ help  h     | | Print help message. }"
"{ input i     | | Path to input video file.}";

int main(int argc, char const *argv[])
{
    Point init_at1(0, 0), init_at2(0, 0);
    
	CommandLineParser parser(argc, argv, keys);
	const std::string imgFile = parser.get<String>("i");
    Mat frame;
    // Read video parameter
    VideoCapture cap(0);
    if (!cap.isOpened()) { printf("NO VIDEO\n"); }
    
	//동영상 플레이시 크기를  320x240으로 지정  
	cap.set(CAP_PROP_FRAME_WIDTH, 960);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720);

    // Create the YOLO network
    yoloNet yolo = yoloNet("yolov3-tiny.weights","yolov3-tiny.cfg","coco.names", 416, 416, 0.3);

	int previous_nObject = 0;

    while (1)
    {
        // load frame
        cap >> frame;
        if(frame.empty()) break;

        // run the network
        yolo.runOnFrame(frame);
        // get the output
        std::vector<yoloObject_t> objects;

        objects = yolo.getOutputObjects();

		consoleMove(0, -previous_nObject);
        for(int i = 0; i < objects.size(); i++) {
			printf("%3d. [", i);
			int nPercent = (int)((objects[i].confidence + 0.05) * 10);
			for (int j = 1; j <= 10; j++) {
				if (j <= nPercent) printf("=");
				else printf(" ");
			}
			std::cout << "] " << objects[i].confidence << std::endl;
            drawBoundingBox(frame, i, objects[i].confidence, objects[i].boundingBox);
        }
		if (objects.size() < previous_nObject) {
			for (int i = objects.size(); i < previous_nObject; i++) {
				for (int j = 0; j < 35; j++) std::cout << ' ';
				std::cout << std::endl;
			}
			consoleMove(0, -(previous_nObject - objects.size()));
		}
		previous_nObject = objects.size();

        // (1) ROI
		Mat roi = Find_ROI(frame);

		// (2) Canny Edge
		Mat Canny_img = Detect_Edge(roi);

		// (3) ROI to ROI
		Rect Lroi, Rroi;
		Lroi.x = 0;
		Lroi.y = Rroi.y = Canny_img.rows / 2;
		Lroi.width = Rroi.width = Rroi.x = Canny_img.cols / 2;
		Lroi.height = Rroi.height = Canny_img.rows / 2;

		Mat Left_roi, Right_roi;
		Left_roi = Canny_img(Lroi);
		Right_roi = Canny_img(Rroi);

		// (4) 좌우 차선 각도 구하기
		double Left_angle = Find_Angle(Left_roi);	// 왼쪽 차선은 (-)
		double Right_angle = Find_Angle(Right_roi);	// 오른쪽 차선은 (+)

		// (6) 가상 차선 좌표 구하기
		vector<Point> vitural_lane_Position = Virtual_Lane(Point2d(Left_angle, Right_angle));

		// (7) 가상 차선 그리기
		line(frame, vitural_lane_Position[0], vitural_lane_Position[1], Scalar(0, 0, 255), 3);	// 윗변
		line(frame, vitural_lane_Position[1], vitural_lane_Position[2], Scalar(255, 0, 0), 3);	// 오른쪽 차선
		line(frame, vitural_lane_Position[2], vitural_lane_Position[3], Scalar(0, 0, 255), 3);	// 밑변
		line(frame, vitural_lane_Position[3], vitural_lane_Position[0], Scalar(0, 255, 0), 3);	// 왼쪽 차선

		// (8) 차선안에 인식된 사물의 좌표가 포함되어 있을 때 - 경고 울림

        for (int i = 0; i < objects.size(); i++)
        {
            Rect Objcet_po = objects[i].boundingBox;	// <-- 인식된 사물 Rect형 좌표
            bool Alarm = Lane_In_Person_Recognition(vitural_lane_Position, Point2d(Left_angle, Right_angle), Objcet_po);
            
			if (Alarm == true) 
				Beep(200, 100);
        }
        // show the frame
		//resizeWindow("tracking", frame.cols, frame.rows);
        imshow("tracking", frame);

        // press ESC to exit
        char c = (char)waitKey(1);
        if(c == 27) break;
    }

    // release
    cap.release();

    destroyAllWindows();
    return 0;
}