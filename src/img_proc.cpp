//
// Created by irving on 23-10-9.
//
#include "pcd_to_image/img_proc.h"

namespace img_proc
{
    void Img_Proc::onMouse(int event, int x, int y, int flags, void *param) {
///用于绘制画笔线段
        if (event == cv::EVENT_RBUTTONDOWN) {
            // 左键按下，记录起点坐标
            startPoint = cv::Point(x, y);
        } else if (event == cv::EVENT_RBUTTONUP) {
            // 左键释放，记录终点坐标并绘制直线
            endPoint = cv::Point(x,y);
            cv::line(image_, startPoint, endPoint, cv::Scalar(255), 2);
        } else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_RBUTTON)) {
            // 鼠标拖动，创建临时图像实时显示线段
            cv::Point currentPoint(x, y);
            cv::Mat img_temp = image_.clone();
            endPoint = currentPoint;
            cv::line(img_temp, startPoint, endPoint, cv::Scalar(255), 2);
            cv::imshow("Image", img_temp);
            return;
        }

///用于绘制橡皮擦区域
        if (event == cv::EVENT_LBUTTONDOWN) {
            // 左键按下，记录起点坐标
            startPoint = cv::Point(x, y);
        } else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) {
            // 鼠标拖动，实时显示轨迹
            cv::Point currentPoint(x, y);
            cv::Mat img_temp = image_.clone();
            endPoint = currentPoint;
            cv::line(image_, startPoint, endPoint, cv::Scalar(0), 2);
        }
        cv::imshow("Image", image_); // 显示掩码图像
    }

    cv::Mat Img_Proc::image_{};
    cv::Rect Img_Proc::selection_rect{};
    cv::Point Img_Proc::startPoint{};
    cv::Point Img_Proc::endPoint{};
    std::vector<cv::Point> Img_Proc::points{};
}

int main()
{
    using namespace img_proc;
    cv::Mat img = cv::imread("/media/irving/UBUNTU 20_0/robomaster/哨兵静态图层/test_big1.png",cv::IMREAD_GRAYSCALE);
    Img_Proc imgProc;

///gamma
    cv::Mat look_up_table_ = cv::Mat::ones(1, 256, CV_8U);
    uchar* p = look_up_table_.ptr();
    for (int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 0.1) * 255.0);
//    cv::LUT(img,look_up_table_,img);

/// 对比度
    double g_nContrastValue = 55;
    double g_nBrightValue = 1;
        for(int y = 0; y < img.rows; y++ )
    {
      for(int x = 0; x < img.cols; x++ )
      {
          img.at<uchar>(y,x)= cv::saturate_cast<uchar>( (g_nContrastValue)*(img.at<uchar>(y,x) ) + g_nBrightValue );
      }
    }
/// 标准化
    cv::normalize(img, img, 0, 255, cv::NORM_MINMAX, CV_8UC1);

///滤波
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 结构元素
//    cv::threshold(img, img, 120, 255, cv::THRESH_BINARY);
//    cv::morphologyEx(img, img, cv::MORPH_OPEN, kernel);
//    cv::threshold(img, img, 50, 255, cv::THRESH_BINARY);


///颜色翻转
//    img = 255 - img;

///整体翻转
// 0为上下，1为左右，-1全
//    cv::flip(img,img,0);

// 转置
//    cv::transpose(img,img);

///添加边界以及尺寸调整
//    cv::copyMakeBorder(img, img, 5, 5, 5, 5, cv::BORDER_CONSTANT, cv::Scalar(255));
//    cv::resize(img,img,cv::Size(414,276));

    imgProc.image_ = img;
    cv::imshow("Image",img);

///鼠标操作回调
    cv::namedWindow("Image");
    cv::imshow("Image",img);
    cv::setMouseCallback("Image", imgProc.onMouse);
    cv::waitKey(0);
    img = imgProc.image_;

    cv::imwrite("/media/irving/UBUNTU 20_0/robomaster/哨兵静态图层/test3.png",img);
}