#pragma once
#include <opencv2/opencv.hpp>
namespace detector
{
	/**
	 * @brief Creates an image with filled contours
	 * @param img_rgb Original RGB image
	 * @param contours Detected contours to be filled
	 * @return Image with filled contours
	 */
	cv::Mat FillContours(cv::Mat img, std::vector<std::vector<cv::Point>> contours);

	/**
	 * @brief Finds median depth of the contours based on the depth image
	 * @param contours Detected contours
	 * @param depth_img Original depth image
	 * @return Vector with median depths
	 */
	std::vector<float> DepthWithinPerimeter(std::vector<std::vector<cv::Point>> contours, cv::Mat &depth_img);

	/**
	 * @brief Finds median of an array
	 * @param a input array
	 * @param n size of input array
	 * @return median of array
	 */
	float FindMedian(std::vector<float> a, int n);

	/**
	 * @brief Fits rotated retangels on contours
	 * @param contours Detected contours
	 * @return Rotated rectangles
	 */
	std::vector<cv::RotatedRect> FindRotatedRects(std::vector<std::vector<cv::Point>> contours);

	/**
	 * @brief Saves detection
	 * @param img_rgb Original RGB image
   	 * @param img_depth Original depth image
	 * @return d
	 */
	void SaveToFile(cv::Mat img_rgb, cv::Mat img_depth, std::vector<cv::RotatedRect> rot_rects);

	/**
		 * @brief Draw and show detections
		 * @param contours Detected contours
		 * @param rot_rects Fitted rotated retangles
		 * @return
		 */
	void ShowDetections(cv::Mat img_rgb, std::vector<std::vector<cv::Point>> contours, std::vector<cv::RotatedRect> rot_rects);

	/**
	 * @brief Converts detection coordinates to 3D poses
	 * @param rot_rects Fitted rotated retangles
	 * @param depth Array with depths for detections
	 * @param img_w Image width
	 * @param img_h Image height
	 * @param f_x Focal length for x-direction
	 * @param f_y Focal length for y-direction
	 * @param camera2base Transformation matrix for camera-to-base
	 * @return Vector with 3D poses for detected objects
	 */
	std::vector<cv::Mat> convert_2_transforms(std::vector<cv::RotatedRect> rot_rect,
												std::vector<float> depth,
												int img_w,
												int img_h,
												float f_x,
												float f_y,
												cv::Mat camera2base);

	/**
	 * @brief Applies a rectangular mask
	 * @param original
	 * @param x The x coordinate for the upper left corner of the mask.
	 * @param y The y coordinate for the upper left corner of the mask.
	 * @param width Width of the mask.
	 * @param height Height of the mask
	 * @return masked image
	 */
	cv::Mat ApplyMask(cv::Mat original,
										const int& x = 0,
										const int& y = 0,
										const int& width = 10,
										const int& height = 10);

	/**
	 * @brief Erodes then dilates the image
	 * @param img Original RGB image
	 * @param erodeSize Amount of times to apply erosion
	 * @param dilateSize Amount of times to apply dilation
	 * @return Eroded and dilated image
	 */
	cv::Mat ErodeAndDilate(cv::Mat img, int erodeSize, int dilateSize);



	/**
	 * @brief Calculates the norm of the difference between the image and the background image
	 * @param img Original RGB image
	 * @param background Background image
	 * @return 
	 */
	cv::Mat DiffNorm(cv::Mat &img, cv::Mat background);

}

