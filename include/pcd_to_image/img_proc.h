//
// Created by irving on 23-10-9.
//

#ifndef SRC_IMG_PROC_H
#define SRC_IMG_PROC_H
#include <iostream>
#include <opencv2/opencv.hpp>
#endif //SRC_IMG_PROC_H

namespace img_proc
{
    class Img_Proc {
    public:
        static cv::Mat image_;
        static cv::Rect selection_rect;

        static void onMouse(int event, int x, int y, int flags, void *param);
        // 新增成员变量存储选择的矩形区域

        static void onMouse2(int event, int x, int y, int flags, void *param)
        {
          // ... 已有事件处理代码 ...
          // 开始选择矩形区域时记录起始点
          if (event == cv::EVENT_LBUTTONDOWN)
          {
            selection_rect = cv::Rect(x, y, 0, 0);
          }
          // 鼠标移动过程中更新矩形大小
          else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))
          {
            selection_rect.width = x - selection_rect.x;
            selection_rect.height = y - selection_rect.y;
            // 实时显示选区（可选）
            cv::Mat image_temp = image_.clone();
            cv::rectangle(image_temp, selection_rect, cv::Scalar(0), 1);
            cv::imshow("Image2", image_temp);
          }
          // 左键释放时完成矩形区域选择
          else if (event == cv::EVENT_LBUTTONUP)
          {
            selection_rect.width = x - selection_rect.x;
            selection_rect.height = y - selection_rect.y;
            // 不再实时显示选区
            image_ = image_(selection_rect);
//            cv::imshow("Image2", image_);
          }

          // ... 已有显示掩码图像的代码 ...
        }

        static cv::Point startPoint;
        static cv::Point endPoint;
        static std::vector<cv::Point> points;
    };
}
