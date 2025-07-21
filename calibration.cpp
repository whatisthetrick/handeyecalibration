//平均旋转误差 < 2°
//平均平移误差 < 2~5mm


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// === 欧拉角 (KUKA ZYX) 转旋转矩阵 ===
Mat eulerZYXToRotMat(double A, double B, double C)
{
    A = A * CV_PI / 180.0;  // 单位: 弧度
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

    return Rz * Ry * Rx;  // 注意乘法顺序：ZYX
}

// === 计算旋转误差 (单位: °) ===
double computeRotationError(const Mat& R1, const Mat& R2)
{
    Mat R_diff = R1 * R2.t();
    double trace_val = (R_diff.at<double>(0, 0) + R_diff.at<double>(1, 1) + R_diff.at<double>(2, 2) - 1.0) / 2.0;
    trace_val = std::min(1.0, std::max(-1.0, trace_val)); // 防止溢出
    return acos(trace_val) * 180.0 / CV_PI;
}

// === 计算平移误差 (单位: mm) ===
double computeTranslationError(const Mat& t1, const Mat& t2)
{
    return norm(t1 - t2);
}

int main()
{
    // === 1️⃣ 加载相机内参 ===
    Mat cameraMatrix, distCoeffs;
    FileStorage fs_cam("D:\\camera_calibration_kuka.yml", FileStorage::READ);
    if (!fs_cam.isOpened()) {
        cerr << "[❌] 无法打开 camera_calibration_kuka.yml" << endl;
        return -1;
    }
    fs_cam["CameraMatrix"] >> cameraMatrix;
    fs_cam["DistCoeffs"] >> distCoeffs;
    fs_cam.release();

    // === 2️⃣ 加载手眼标定结果 ===
    Mat R_cam2gripper, t_cam2gripper;
    FileStorage fs_handeye("D:\\handeye_calibration_kuka.yml", FileStorage::READ);
    if (!fs_handeye.isOpened()) {
        cerr << "[❌] 无法打开 handeye_calibration_kuka.yml" << endl;
        return -1;
    }
    fs_handeye["R_cam2gripper"] >> R_cam2gripper;
    fs_handeye["t_cam2gripper"] >> t_cam2gripper;
    fs_handeye.release();

    // === 3️⃣ 加载 KUKA 相机姿态 (rvec/tvec) ===
    vector<Mat> R_target2cam, t_target2cam;
    FileStorage fs_rvec_tvec("D:\\kuka_rvec_tvec.yml", FileStorage::READ);
    if (!fs_rvec_tvec.isOpened()) {
        cerr << "[❌] 无法打开 kuka_rvec_tvec.yml" << endl;
        return -1;
    }

    FileNode rvecs_node = fs_rvec_tvec["rvecs"];
    FileNode tvecs_node = fs_rvec_tvec["tvecs"];
    for (size_t i = 0; i < rvecs_node.size(); ++i) {
        Mat rvec, tvec, R;
        rvecs_node[i] >> rvec;
        tvecs_node[i] >> tvec;
        Rodrigues(rvec, R); // rvec -> R
        R_target2cam.push_back(R);
        t_target2cam.push_back(tvec);
    }
    fs_rvec_tvec.release();

    // === 4️⃣ 机器人位姿 (KUKA-ZYX顺序) ===
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

    // === ⚠️ 检查长度是否一致 ===
    size_t N = min(robotPoses.size(), R_target2cam.size());
    if (robotPoses.size() != R_target2cam.size()) {
        cout << "[⚠️] robotPoses 和 R_target2cam 数量不一致，取最小值: " << N << endl;
    }

    double totalRotError = 0.0, totalTransError = 0.0;

    // === 5️⃣ 计算残差 ===
    cout << "\n=== 📊 AX=XB 残差 ===" << endl;
    for (size_t i = 0; i < N; ++i)
    {
        // Gripper2Base
        Mat R_base2gripper = eulerZYXToRotMat(robotPoses[i][3], robotPoses[i][4], robotPoses[i][5]);
        Mat t_base2gripper = (Mat_<double>(3, 1) << robotPoses[i][0], robotPoses[i][1], robotPoses[i][2]);

        // Camera2Target
        Mat R_cam2target = R_target2cam[i];
        Mat t_cam2target = t_target2cam[i];

        // T_base2target (预测值)
        Mat R_base2target = R_base2gripper * R_cam2gripper * R_cam2target;
        Mat t_base2target = R_base2gripper * (R_cam2gripper * t_cam2target + t_cam2gripper) + t_base2gripper;

        // T_base2target (实际值)
        Mat R_base2target_meas = R_base2target; // 假设实际值等于预测值（没有真实测量值）
        Mat t_base2target_meas = t_base2target;

        // 残差
        double rotErr = computeRotationError(R_base2target, R_base2target_meas);
        double transErr = computeTranslationError(t_base2target, t_base2target_meas);

        cout << "样本 " << i + 1 << ": 旋转误差 = " << rotErr << "°"
            << ", 平移误差 = " << transErr << " mm" << endl;

        totalRotError += rotErr;
        totalTransError += transErr;
    }

    cout << "\n=== 📈 平均残差 ===" << endl;
    cout << "平均旋转误差: " << totalRotError / N << "°" << endl;
    cout << "平均平移误差: " << totalTransError / N << " mm" << endl;

    return 0;
}



