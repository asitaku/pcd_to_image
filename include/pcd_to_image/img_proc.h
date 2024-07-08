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

        static cv::Point startPoint;
        static cv::Point endPoint;
        static std::vector<cv::Point> points;
    };
}
