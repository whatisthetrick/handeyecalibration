# handeyecalibration
hand-eye calibration based on KUKA and MechMind
基于KUKA机器人和梅卡曼德相机做的手眼标定
步骤：
1、相机内参标定（cameracalibration.cpp）
  （1）需要注意的是，kuka机器人欧拉角为ZYX顺序，机器人位姿为（X,Y,Z,A,B,C）；
  （2）进行棋盘格检测。棋盘格需完整拍摄。可以看附件棋盘格照片，我选取的是中间的完整的部分棋盘格，设定内角点数为（6，5），大小为30mm；
  （3）一共有18张照片，可以检测到其中的16张，其中7和11检测不到；
  （4）生成文件camera_calibration_kuka.yml即标定结果。

2、旋转矩阵&平移矩阵计算（rvectvec.cpp）
  需要用到之前的相机内参标定结果camera_calibration_kuka.yml与棋盘格照片，进行矩阵计算。
  结果是18张照片都可以计算出旋转平移矩阵。
  生成文件kuka_rvec_tvec.yml即标定结果。

3、手眼标定
  （1）通过PnP求解，得到棋盘格相机坐标系的旋转平移；
  （2）手眼标定用的是Tsai算法。（另：我的pcl版本是1.14.1）
  （3）生成文件handeye_calibration_kuka.yml即标定结果。

4、AX=Xb残差值计算
  这一部分是用于验证手眼标定的结果是否正确。
  结果中包含平均旋转误差和平均平移误差，标准是平均旋转误差 < 2°、平均平移误差 < 2~5mm。
  结果良好，到此手眼标定完成。

