#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int hMin1 = 0;
int hMax1 = 32;

int hMin2 = 150;
int hMax2 = 180;

int sMin = 100;
int sMax = 255;
int vMin = 100;
int vMax = 255;

int HUE_MAX = 180;
int SV_MAX = 255;

int main(int argc, char* argv[]) {

    if (argc != 2) {
        cout << "Cara Penggunaan: ./deteksi_app <nama_file_video>" << endl;
        return -1;
    }

    VideoCapture cap(argv[1]);
    if (!cap.isOpened()) {
        cout << "Error: Tidak bisa membuka file video." << endl;
        return -1;
    }

    namedWindow("Original Video", WINDOW_AUTOSIZE);
    namedWindow("Masked Video", WINDOW_AUTOSIZE);
    namedWindow("HSV Adjustments", WINDOW_AUTOSIZE);

    createTrackbar("Hue Min 1", "HSV Adjustments", &hMin1, HUE_MAX);
    createTrackbar("Hue Max 1", "HSV Adjustments", &hMax1, HUE_MAX);

    createTrackbar("Hue Min 2", "HSV Adjustments", &hMin2, HUE_MAX);
    createTrackbar("Hue Max 2", "HSV Adjustments", &hMax2, HUE_MAX);

    createTrackbar("Sat Min", "HSV Adjustments", &sMin, SV_MAX);
    createTrackbar("Sat Max", "HSV Adjustments", &sMax, SV_MAX);

    createTrackbar("Val Min", "HSV Adjustments", &vMin, SV_MAX);
    createTrackbar("Val Max", "HSV Adjustments", &vMax, SV_MAX);

    Mat frame, hsvFrame, mask1, mask2, finalMask;

    while (true) {
        cap.read(frame);

        if (frame.empty()) {
            cout << "Video selesai." << endl;
            break;
        }

        cvtColor(frame, hsvFrame, COLOR_BGR2HSV);

        Scalar lower_range_1(hMin1, sMin, vMin);
        Scalar upper_range_1(hMax1, sMax, vMax);
        
        Scalar lower_range_2(hMin2, sMin, vMin);
        Scalar upper_range_2(hMax2, sMax, vMax);

        inRange(hsvFrame, lower_range_1, upper_range_1, mask1);
        inRange(hsvFrame, lower_range_2, upper_range_2, mask2);

        bitwise_or(mask1, mask2, finalMask);

        imshow("Original Video", frame);
        imshow("Masked Video", finalMask);

        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
