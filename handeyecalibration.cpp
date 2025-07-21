#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// KUKA机器人欧拉角ZYX顺序转换为旋转矩阵
Mat eulerZYXToRotMat(double A, double B, double C)
{
    A = A * CV_PI / 180.0; // 单位转弧度
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

    return Rz * Ry * Rx;  // ZYX顺序旋转矩阵乘法
}

int main()
{
    // 读取相机内参和畸变参数
    FileStorage fs("D:\\camera_calibration_kuka.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "[Error] 无法打开相机内参文件" << endl;
        return -1;
    }
    Mat cameraMatrix, distCoeffs;
    fs["CameraMatrix"] >> cameraMatrix;
    fs["DistCoeffs"] >> distCoeffs;
    fs.release();
    cout << "? 已加载相机内参\n" << cameraMatrix << endl;

    // KUKA机器人位姿(X,Y,Z,A,B,C) 角度单位为度，A,B,C为ZYX顺序欧拉角
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

    // 棋盘格参数（角点数和尺寸）
    Size boardSize(6, 5);
    float squareSize = 30.f; // mm
    vector<Point3f> objPoints;
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            objPoints.emplace_back(j * squareSize, i * squareSize, 0);

    // 存储机器人姿态（基坐标系下）和平移
    vector<Mat> R_gripper2base, t_gripper2base;
    // 存储相机观察棋盘格姿态（相机坐标系下）和平移
    vector<Mat> R_target2cam, t_target2cam;

    for (size_t i = 0; i < robotPoses.size(); ++i)
    {
        // 读取对应图片
        char filename[100];
        sprintf(filename, "D:\\rgb_image_%05d.png", 10000 + int(i));
        Mat img = imread(filename);
        if (img.empty()) {
            cerr << "[Warning] 图片缺失: " << filename << endl;
            continue;
        }

        // 找棋盘格角点
        vector<Point2f> imgPoints;
        bool found = findChessboardCorners(img, boardSize, imgPoints);
        if (!found) {
            cerr << "[Warning] 棋盘格未检测到: " << filename << endl;
            continue;
        }

        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        cornerSubPix(gray, imgPoints, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));

        // 机器人姿态转旋转矩阵和平移向量
        Mat R = eulerZYXToRotMat(robotPoses[i][3], robotPoses[i][4], robotPoses[i][5]);
        Mat t = (Mat_<double>(3, 1) << robotPoses[i][0], robotPoses[i][1], robotPoses[i][2]);
        R_gripper2base.push_back(R);
        t_gripper2base.push_back(t);

        // 求解PnP 得到棋盘格相机坐标系的旋转平移
        Mat rvec, tvec;
        solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);
        Mat R_cam;
        Rodrigues(rvec, R_cam);
        R_target2cam.push_back(R_cam);
        t_target2cam.push_back(tvec);
    }

    // 手眼标定 (Tsai算法)
    Mat R_cam2gripper, t_cam2gripper;
    calibrateHandEye(R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        R_cam2gripper, t_cam2gripper,
        CALIB_HAND_EYE_TSAI);

    cout << "\n=== 手眼标定结果 ===" << endl;
    cout << "Rotation Matrix (R_cam2gripper):\n" << R_cam2gripper << endl;
    cout << "Translation Vector (t_cam2gripper):\n" << t_cam2gripper << endl;

    // 保存结果
    FileStorage fs_out("D:\\handeye_calibration_kuka.yml", FileStorage::WRITE);
    fs_out << "R_cam2gripper" << R_cam2gripper;
    fs_out << "t_cam2gripper" << t_cam2gripper;
    fs_out.release();

    cout << "? 标定结果已保存到 D:\\handeye_calibration_kuka.yml" << endl;

    return 0;
}
