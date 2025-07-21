#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// === 欧拉角 (KUKA: ZYX顺序) 转换为旋转矩阵 ===
Mat eulerZYXToRotMat(double A, double B, double C)
{
    A = A * CV_PI / 180.0;  // 角度 -> 弧度
    B = B * CV_PI / 180.0;
    C = C * CV_PI / 180.0;

    Mat Rz = (Mat_<double>(3, 3) <<
        cos(C), -sin(C), 0,
        sin(C), cos(C), 0,
        0, 0, 1);

    Mat Ry = (Mat_<double>(3, 3) <<
        cos(B), 0, sin(B),
        0, 1, 0,
        -sin(B), 0, cos(B));

    Mat Rx = (Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(A), -sin(A),
        0, sin(A), cos(A));

    return Rz * Ry * Rx;  // ZYX顺序
}

int main()
{
    // === KUKA机器人位姿 (X,Y,Z,A,B,C) ===
    vector<vector<double>> robotPoses = {
        {425.47, -1411.17, 1006.78, 86.70, -29.47, 94.19},
        {476.17, -1411.17, 1006.78, 83.54, -28.99, 100.65},
        {476.17, -1411.17, 1006.78, 83.79, -24.48, 100.09},
        {496.50, -1411.17, 1006.78, 83.94, -23.99, 99.73},
        {496.50, -1325.19, 1006.78, 83.67, -16.75, 100.28},
        {496.50, -1265.95, 1006.78, 83.89, -7.43, 99.24},
        {437.15, -1269.11, 1006.79, 85.17, -8.31, 90.00},
        {372.75, -1260.42, 1006.73, 81.68, -8.83, 83.36},
        {320.71, -1267.22, 1006.70, 83.10, -13.57, 78.51},
        {266.89, -1267.93, 1006.82, 94.53, -13.98, 74.62},
        {320.82, -1574.73, 1007.32, 96.11, -35.85, 76.34},
        {395.60, -1663.52, 1007.56, 90.07, -43.80, 86.60},
        {285.04, -1663.36, 1007.64, 94.95, -43.58, 79.56},
        {515.15, -1488.88, 1007.83, 82.68, -33.92, 98.88},
        {507.95, -1247.15, 1007.78, 83.81, -11.58, 96.03},
        {639.51, -1247.32, 1007.89, 81.91, -10.35, 105.92},
        {624.13, -1176.29, 1008.22, 82.87, -2.95, 104.87},
        {506.78, -1176.21, 1008.25, 83.38, -3.95, 96.52},
    };

    // === 棋盘格参数 ===
    Size boardSize(6, 5);    // 内角点数
    float squareSize = 30.0f; // 单位:mm
    vector<vector<Point3f>> objectPoints; // 3D世界坐标
    vector<vector<Point2f>> imagePoints;  // 2D像素坐标

    vector<Point3f> objp;
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            objp.emplace_back(j * squareSize, i * squareSize, 0);

    for (size_t i = 0; i < robotPoses.size(); ++i)
    {
        // 读取图片
        char filename[100];
        sprintf(filename, "D:\\rgb_image_%05d.png", 10000 + int(i));
        Mat img = imread(filename);
        if (img.empty()) {
            cerr << "[??] 图像未找到: " << filename << endl;
            continue;
        }

        // 棋盘格检测
        vector<Point2f> corners;
        bool found = findChessboardCorners(img, boardSize, corners);
        if (!found) {
            cerr << "[??] 棋盘格未检测到: " << filename << endl;
            continue;
        }

        // 亚像素角点优化
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));

        // 保存点
        objectPoints.push_back(objp);
        imagePoints.push_back(corners);

        // 可视化角点
        drawChessboardCorners(img, boardSize, corners, found);
        imshow("Detected Corners", img);
        waitKey(200);
    }
    destroyAllWindows();

    if (objectPoints.size() < 5) {
        cerr << "[?] 有效图片不足，至少需要5张成功检测棋盘格的图片！" << endl;
        return -1;
    }

    // === 相机内参标定 ===
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(objectPoints, imagePoints, Size(1920, 1080), cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "\n=== ?? 相机标定结果 ===" << endl;
    cout << "RMS 重投影误差: " << rms << endl;
    cout << "Camera Matrix:\n" << cameraMatrix << endl;
    cout << "Distortion Coefficients:\n" << distCoeffs << endl;

    // === 保存标定结果 ===
    FileStorage fs("D:\\camera_calibration_kuka.yml", FileStorage::WRITE);
    fs << "CameraMatrix" << cameraMatrix;
    fs << "DistCoeffs" << distCoeffs;
    fs.release();
    cout << "? 已保存到 D:\\camera_calibration_kuka.yml" << endl;

    return 0;
}
