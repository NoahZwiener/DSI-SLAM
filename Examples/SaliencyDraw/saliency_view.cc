#include <iostream>
#include <algorithm>
#include <fstream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ORBextractor.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Converter.h"
// 引入相机模型头文件
#include "CameraModels/Pinhole.h"
#include "ImuTypes.h" // 引入IMU相关定义

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << endl
             << "Usage: ./saliency_view <path_to_image> <output_file_path>" << endl;
        return 1;
    }

    string imagePath = argv[1];
    string outputPath = argv[2];

    // 1. 读取图片
    Mat im = imread(imagePath, IMREAD_UNCHANGED);
    if (im.empty())
    {
        cerr << endl
             << "Failed to load image at: " << imagePath << endl;
        return 1;
    }

    Mat imGray;
    if (im.channels() == 3)
    {
        cvtColor(im, imGray, COLOR_BGR2GRAY);
    }
    else if (im.channels() == 4)
    {
        cvtColor(im, imGray, COLOR_BGRA2GRAY);
    }
    else
    {
        imGray = im;
    }

    // 2. 初始化组件
    ORB_SLAM3::ORBextractor *mpORBextractor = new ORB_SLAM3::ORBextractor(1000, 1.2f, 8, 20, 7);
    ORB_SLAM3::ORBVocabulary *mpVocabulary = new ORB_SLAM3::ORBVocabulary();

    // 3. 创建相机模型 (修复构造函数报错的关键)
    // 模拟一个针孔相机参数: fx, fy, cx, cy
    float width = (float)imGray.cols;
    float height = (float)imGray.rows;
    vector<float> vCamCalib;
    vCamCalib.push_back(width);         // fx
    vCamCalib.push_back(width);         // fy
    vCamCalib.push_back(width / 2.0f);  // cx
    vCamCalib.push_back(height / 2.0f); // cy

    // 实例化 Pinhole 相机对象
    ORB_SLAM3::Pinhole *pCamera = new ORB_SLAM3::Pinhole(vCamCalib);

    // 畸变参数 (假设无畸变)
    Mat DistCoef = Mat::zeros(4, 1, CV_32F);
    float bf = 40.0f;
    float thDepth = 50.0f;

    cout << "Start processing..." << endl;

    // 4. 创建 Frame (使用新的构造函数签名)
    // Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc,
    //       GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth,
    //       Frame* pPrevF, const IMU::Calib &ImuCalib);

    ORB_SLAM3::Frame mFrame = ORB_SLAM3::Frame(
        imGray,
        0.0,
        mpORBextractor,
        mpVocabulary,
        pCamera, // 传入相机对象指针
        DistCoef,
        bf,
        thDepth,
        nullptr,                // pPrevF: 第一帧没有前一帧，传空指针
        ORB_SLAM3::IMU::Calib() // ImuCalib: 传入默认构造的IMU标定对象
    );

    // 5. 导出数据
#ifdef USE_SALIENCY
    // 只有定义了宏，才能访问 mvSaliencySpatial
    // 注意：这里需要检查是否为空，如果 Frame 构造函数里没计算，这里会崩
    size_t numPoints = mFrame.mvKeys.size();
    size_t numSaliency = mFrame.mvSaliencySpatial.size();

    cout << "Features: " << numPoints << ", Saliency Values: " << numSaliency << endl;

    ofstream outfile(outputPath);
    if (!outfile.is_open())
        return -1;

    for (size_t i = 0; i < numPoints; ++i)
    {
        float x = mFrame.mvKeys[i].pt.x;
        float y = mFrame.mvKeys[i].pt.y;
        float s = 0.0f;
        if (i < numSaliency)
            s = mFrame.mvSaliencySpatial[i];

        outfile << x << " " << y << " " << s << endl;
    }
    outfile.close();
    cout << "Saved to " << outputPath << endl;
#else
    cerr << "Error: USE_SALIENCY is not defined! Cannot access saliency data." << endl;
#endif

    // 清理内存
    delete mpORBextractor;
    delete mpVocabulary;
    delete pCamera;

    return 0;
}