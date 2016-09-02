#include <iostream>
#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include <boost/concept_check.hpp>

#include <opencv2/opencv.hpp>

#include "maker_binocular.h"


int main()
{
    makerbinocular m_makerbinocular;

    cv::Mat left_image(480, 640, CV_8UC1, cv::Scalar(0));
    cv::Mat right_image(480, 640, CV_8UC1, cv::Scalar(0));

    if (m_makerbinocular.is_initialized())
    {
        while (1)
        {
            m_makerbinocular.get_frame(left_image, right_image);

            if (m_makerbinocular.new_frame_arrived())
            {
                cv::imshow("left image", left_image);
                cv::waitKey(1);
                cv::imshow("right image" ,  right_image);
                cv::waitKey(1);
            }
            usleep(10000);
        }
    }

    return 0;
}
