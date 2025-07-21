#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// KUKA机器人欧拉角 (ZYX顺序) 转换为旋转矩阵
Mat eulerZYXToRotMat(double A, double B, double C)
{
    A = A * CV_PI / 180.0;  // 角度 → 弧度
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

    return Rz * Ry * Rx;  // ZYX 顺序
}

int main()
{
    // === 读取相机内参 ===
    FileStorage fs("D:\\camera_calibration_kuka.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "[?] 无法打开 camera_calibration.yml" << endl;
        return -1;
    }
    Mat cameraMatrix, distCoeffs;
    fs["CameraMatrix"] >> cameraMatrix;
    fs["DistCoeffs"] >> distCoeffs;
    fs.release();
    cout << "? 已加载相机内参\n" << cameraMatrix << endl;

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
    float squareSize = 30.f; // 棋盘格方格尺寸 (单位: mm)
    vector<Point3f> objPoints;
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            objPoints.emplace_back(j * squareSize, i * squareSize, 0);

    // 输出结果
    vector<Mat> rvecs, tvecs;

    for (size_t i = 0; i < robotPoses.size(); ++i)
    {
        // 读取对应图片
        char filename[100];
        sprintf(filename, "D:\\rgb_image_%05d.png", 10000 + int(i));
        Mat img = imread(filename);
        if (img.empty()) {
            cerr << "[??] 图像未找到: " << filename << endl;
            continue;
        }

        // 棋盘格检测
        vector<Point2f> imgPoints;
        bool found = findChessboardCorners(img, boardSize, imgPoints);
        if (!found) {
            cerr << "[??] 棋盘格未检测到: " << filename << endl;
            continue;
        }

        // 亚像素角点优化
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        cornerSubPix(gray, imgPoints, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));

        // solvePnP 计算 rvec 和 tvec
        Mat rvec, tvec;
        solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);

        // 保存结果
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);

        // 打印结果
        cout << "\n=== 样本 " << i + 1 << " ===" << endl;
        cout << "rvec (Rodrigues向量):\n" << rvec.t() << endl;
        cout << "tvec (平移向量):\n" << tvec.t() << endl;
    }

    // 保存所有 rvec 和 tvec
    FileStorage fs_out("D:\\kuka_rvec_tvec.yml", FileStorage::WRITE);
    fs_out << "rvecs" << rvecs;
    fs_out << "tvecs" << tvecs;
    fs_out.release();
    cout << "\n? 所有rvec和tvec已保存到 D:\\kuka_rvec_tvec.yml" << endl;

    return 0;
}
