#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// KUKA������ŷ����ZYX˳��ת��Ϊ��ת����
Mat eulerZYXToRotMat(double A, double B, double C)
{
    A = A * CV_PI / 180.0; // ��λת����
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

    return Rz * Ry * Rx;  // ZYX˳����ת����˷�
}

int main()
{
    // ��ȡ����ڲκͻ������
    FileStorage fs("D:\\camera_calibration_kuka.yml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "[Error] �޷�������ڲ��ļ�" << endl;
        return -1;
    }
    Mat cameraMatrix, distCoeffs;
    fs["CameraMatrix"] >> cameraMatrix;
    fs["DistCoeffs"] >> distCoeffs;
    fs.release();
    cout << "? �Ѽ�������ڲ�\n" << cameraMatrix << endl;

    // KUKA������λ��(X,Y,Z,A,B,C) �Ƕȵ�λΪ�ȣ�A,B,CΪZYX˳��ŷ����
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

    // ���̸�������ǵ����ͳߴ磩
    Size boardSize(6, 5);
    float squareSize = 30.f; // mm
    vector<Point3f> objPoints;
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            objPoints.emplace_back(j * squareSize, i * squareSize, 0);

    // �洢��������̬��������ϵ�£���ƽ��
    vector<Mat> R_gripper2base, t_gripper2base;
    // �洢����۲����̸���̬���������ϵ�£���ƽ��
    vector<Mat> R_target2cam, t_target2cam;

    for (size_t i = 0; i < robotPoses.size(); ++i)
    {
        // ��ȡ��ӦͼƬ
        char filename[100];
        sprintf(filename, "D:\\rgb_image_%05d.png", 10000 + int(i));
        Mat img = imread(filename);
        if (img.empty()) {
            cerr << "[Warning] ͼƬȱʧ: " << filename << endl;
            continue;
        }

        // �����̸�ǵ�
        vector<Point2f> imgPoints;
        bool found = findChessboardCorners(img, boardSize, imgPoints);
        if (!found) {
            cerr << "[Warning] ���̸�δ��⵽: " << filename << endl;
            continue;
        }

        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        cornerSubPix(gray, imgPoints, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));

        // ��������̬ת��ת�����ƽ������
        Mat R = eulerZYXToRotMat(robotPoses[i][3], robotPoses[i][4], robotPoses[i][5]);
        Mat t = (Mat_<double>(3, 1) << robotPoses[i][0], robotPoses[i][1], robotPoses[i][2]);
        R_gripper2base.push_back(R);
        t_gripper2base.push_back(t);

        // ���PnP �õ����̸��������ϵ����תƽ��
        Mat rvec, tvec;
        solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);
        Mat R_cam;
        Rodrigues(rvec, R_cam);
        R_target2cam.push_back(R_cam);
        t_target2cam.push_back(tvec);
    }

    // ���۱궨 (Tsai�㷨)
    Mat R_cam2gripper, t_cam2gripper;
    calibrateHandEye(R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        R_cam2gripper, t_cam2gripper,
        CALIB_HAND_EYE_TSAI);

    cout << "\n=== ���۱궨��� ===" << endl;
    cout << "Rotation Matrix (R_cam2gripper):\n" << R_cam2gripper << endl;
    cout << "Translation Vector (t_cam2gripper):\n" << t_cam2gripper << endl;

    // ������
    FileStorage fs_out("D:\\handeye_calibration_kuka.yml", FileStorage::WRITE);
    fs_out << "R_cam2gripper" << R_cam2gripper;
    fs_out << "t_cam2gripper" << t_cam2gripper;
    fs_out.release();

    cout << "? �궨����ѱ��浽 D:\\handeye_calibration_kuka.yml" << endl;

    return 0;
}
