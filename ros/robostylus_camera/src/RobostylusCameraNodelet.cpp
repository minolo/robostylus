#include <stdexcept>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <robostylus_camera/CalibrateCamera.h>
#include <robostylus_camera/TransferCalibrationData.h>
#include <robostylus_camera/TransformPoint.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace ros;
using namespace sensor_msgs;
using namespace cv_bridge;

typedef struct
{
	// Screen corners in the camera image
	cv::Point2f corners[4];

	// Desired width and height in the perspective-corrected image
	int width, height;

	// Transformation matrix to correct the image
	cv::Mat transform, transform_inv;
} screen_rectangle_t;

namespace robostylus_camera
{
    class RobostylusCameraNodelet : public nodelet::Nodelet
    {
        public:
    		RobostylusCameraNodelet()
            {

            }

        private:

    		NodeHandle nh;

            // Camera subscriber
			image_transport::ImageTransport *it;
			image_transport::CameraSubscriber sub_imageRect;

            // Camera model
			image_geometry::PinholeCameraModel cam_model;

			// Screen publishers
    		Publisher pub_topScreen;
    		Publisher pub_bottomScreen;

    		// Services
    		ServiceServer srv_calibrateCamera;
    		ServiceServer srv_transferCalibrationData;
    		ServiceServer srv_transformPoint;

    		Image::ConstPtr lastImage;

    		// Screen calibration data
    		screen_rectangle_t top_screen, bottom_screen;
    		bool calibrated;

            virtual void onInit()
            {
            	calibrated = false;

            	nh = getNodeHandle();

            	// Subscribe to camera image and info topic
            	it = new image_transport::ImageTransport(nh);
				sub_imageRect = it->subscribeCamera("image_rect", 10, &RobostylusCameraNodelet::imageCallback, this);

				// Screen publishers
				pub_topScreen = nh.advertise<Image>("top_screen", 10);
				pub_bottomScreen = nh.advertise<Image>("bottom_screen", 10);

				// Initialize services
				srv_calibrateCamera = nh.advertiseService("calibrate", &RobostylusCameraNodelet::service_calibrateCamera, this);
				srv_transferCalibrationData = nh.advertiseService("transfer_data", &RobostylusCameraNodelet::service_transferData, this);
				srv_transformPoint = nh.advertiseService("transform_point", &RobostylusCameraNodelet::service_transformPoint, this);
            }

            void calculateTransformationParameters()
            {
            	int distance1, distance2;
            	cv::Point2f corners[4];

            	// Calculate corrected image dimensions
            	distance1 = cv::norm(top_screen.corners[0] - top_screen.corners[1]);
            	distance2 = cv::norm(top_screen.corners[2] - top_screen.corners[3]);
            	top_screen.height = std::max(distance1, distance2);

            	distance1 = cv::norm(top_screen.corners[0] - top_screen.corners[3]);
				distance2 = cv::norm(top_screen.corners[1] - top_screen.corners[2]);
				top_screen.width = std::max(distance1, distance2);

            	distance1 = cv::norm(bottom_screen.corners[0] - bottom_screen.corners[1]);
            	distance2 = cv::norm(bottom_screen.corners[2] - bottom_screen.corners[3]);
            	bottom_screen.height = std::max(distance1, distance2);

            	distance1 = cv::norm(bottom_screen.corners[0] - bottom_screen.corners[3]);
				distance2 = cv::norm(bottom_screen.corners[1] - bottom_screen.corners[2]);
				bottom_screen.width = std::max(distance1, distance2);

				// Corners of the destination image for top screen
				corners[0] = cv::Point2f(top_screen.width - 1, 0                    );
				corners[1] = cv::Point2f(top_screen.width - 1, top_screen.height - 1);
				corners[2] = cv::Point2f(0,                    top_screen.height - 1);
				corners[3] = cv::Point2f(0,                    0                    );

				// Get transformation matrix for top screen
				top_screen.transform = cv::getPerspectiveTransform(top_screen.corners, corners);

				// Corners of the destination image for bottom screen
				corners[0] = cv::Point2f(bottom_screen.width - 1, 0                       );
				corners[1] = cv::Point2f(bottom_screen.width - 1, bottom_screen.height - 1);
				corners[2] = cv::Point2f(0,                       bottom_screen.height - 1);
				corners[3] = cv::Point2f(0,                       0                       );

				// Get transformation matrices for bottom screen
				bottom_screen.transform = cv::getPerspectiveTransform(bottom_screen.corners, corners);
				bottom_screen.transform_inv = cv::getPerspectiveTransform(corners, bottom_screen.corners);

				calibrated = true;
            }

            void imageCallback(const Image::ConstPtr& img_msg, sensor_msgs::CameraInfoConstPtr const& info_msg)
            {
            	cam_model.fromCameraInfo(info_msg);

            	lastImage = img_msg;

            	cv_bridge::CvImagePtr cv_image;

            	// Split top screen image
				if(pub_topScreen.getNumSubscribers() > 0)
				{
					if(calibrated)
					{
						// Convert to opencv format
						cv_image = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::RGB8);

						// Use transformation matrix for top screen to obtain top-down view
						cv::Mat mat_topScreen = cv::Mat::zeros(top_screen.height, top_screen.width, CV_8UC3);
						cv::warpPerspective(cv_image->image, mat_topScreen, top_screen.transform, mat_topScreen.size());

						// Create cv_bridge image
						cv_bridge::CvImage cv_image_topScreen;
						cv_image_topScreen.header.stamp = ros::Time::now();
						cv_image_topScreen.header.frame_id = "image_topScreen";
						cv_image_topScreen.encoding = sensor_msgs::image_encodings::RGB8;
						cv_image_topScreen.image = mat_topScreen;

						// Create image message and publish
						sensor_msgs::Image im_topScreen;
						cv_image_topScreen.toImageMsg(im_topScreen);
						pub_topScreen.publish(im_topScreen);
					}
					else
					{
						ROS_WARN_THROTTLE(10, "Not calibrated!");
					}
				}

				// Split bottom screen image
				if(pub_bottomScreen.getNumSubscribers() > 0)
				{
					if(calibrated)
					{
						// Check if the image has been converted to opencv format in the previous case
						if(cv_image == NULL)
						{
							cv_image = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::RGB8);
						}

						// Use transformation matrix for bottom screen to obtain top-down view
						cv::Mat mat_bottomScreen = cv::Mat::zeros(bottom_screen.height, bottom_screen.width, CV_8UC3);
						cv::warpPerspective(cv_image->image, mat_bottomScreen, bottom_screen.transform, mat_bottomScreen.size());

						// Create cv_bridge image
						cv_bridge::CvImage cv_image_bottomScreen;
						cv_image_bottomScreen.header.stamp = ros::Time::now();
						cv_image_bottomScreen.header.frame_id = "image_bottomScreen";
						cv_image_bottomScreen.encoding = sensor_msgs::image_encodings::RGB8;
						cv_image_bottomScreen.image = mat_bottomScreen;

						// Create image message and publish
						sensor_msgs::Image im_bottomScreen;
						cv_image_bottomScreen.toImageMsg(im_bottomScreen);
						pub_bottomScreen.publish(im_bottomScreen);
					}
					else
					{
						ROS_WARN_THROTTLE(10, "Not calibrated!");
					}
				}
            }

            bool service_transformPoint(robostylus_camera::TransformPoint::Request& request, robostylus_camera::TransformPoint::Response& response)
            {
            	if(calibrated)
            	{
					// Create point object
					cv::Point2f point(request.x, request.y);

					// Make sure the coordinates are inside the bottom screen
					point.x = std::min(point.x, (float)bottom_screen.width);
					point.y = std::min(point.y, (float)bottom_screen.height);

					// Transform the requested point from screen space to image space
					std::vector<cv::Point2f> point_vec, transformed_vec;
					point_vec.push_back(point);
					cv::perspectiveTransform(point_vec, transformed_vec, bottom_screen.transform_inv);

					// Project pixel in image space to 3d space
					cv::Point3d ray = cam_model.projectPixelTo3dRay(transformed_vec[0]);

					// Fill response data
					response.camera_frame = cam_model.tfFrame().c_str();
					response.x = ray.x;
					response.y = ray.y;
					response.z = ray.z;
            	}

            	response.success = calibrated;

            	return true;
            }

            bool service_transferData(robostylus_camera::TransferCalibrationData::Request& request, robostylus_camera::TransferCalibrationData::Response& response)
            {
            	// Load request data
            	if(request.load)
            	{
            		for(int i = 0; i < 4; i++)
					{
            			top_screen.corners[i].x = request.top_screen_x[i];
            			top_screen.corners[i].y = request.top_screen_y[i];

            			bottom_screen.corners[i].x = request.bottom_screen_x[i];
            			bottom_screen.corners[i].y = request.bottom_screen_y[i];
					}

            		calculateTransformationParameters();
            	}

            	// Fill response data
            	for(int i = 0; i < 4; i++)
            	{
            		response.top_screen_x[i] = top_screen.corners[i].x;
            		response.top_screen_y[i] = top_screen.corners[i].y;

            		response.bottom_screen_x[i] = bottom_screen.corners[i].x;
					response.bottom_screen_y[i] = bottom_screen.corners[i].y;
            	}

            	return true;
            }

            bool service_calibrateCamera(robostylus_camera::CalibrateCamera::Request& request, robostylus_camera::CalibrateCamera::Response& response)
            {
            	calibrated = false;
            	try
            	{
					// Copy image and convert to OpenCV format
					cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::RGB8);

					// Apply gaussian blur
					cv::Size size(7, 7);
					cv::GaussianBlur(cv_image->image, cv_image->image, size, 0);

					// Convert to HSV
					cv::Mat hsv_image;
					cv::cvtColor(cv_image->image, hsv_image, CV_RGB2HSV_FULL);

					// Extract channels
					cv::Mat hsv_channels[3];
					cv::split(hsv_image, hsv_channels);

					// Calculate the look-up table with differences with the selected hue
					cv::Mat lookUpTable(256, 1, CV_8U);
					uint8_t diff1, diff2, *p = lookUpTable.data;
					for(int i = 0; i < 256; i++)
					{
						diff1 = std::abs(i - request.target_hue.data);
						diff2 = 255 - diff1;
						p[i] = std::min(diff1, diff2);
					}

					// Apply look-up table
					cv::Mat diffs;
					cv::LUT(hsv_channels[0], lookUpTable, diffs);

					// Amplify and invert
					cv::multiply(diffs, cv::Scalar::all(request.mult_diffs.data), diffs);
					cv::bitwise_not(diffs, diffs);

					// Amplify saturation
					cv::Mat amplified_saturation;
					cv::multiply(hsv_channels[1], cv::Scalar::all(request.mult_sat.data), amplified_saturation);

					// Multiply diffs by saturation (helps with varying conditions of illumination)
					cv::Mat mask;
					cv::multiply(amplified_saturation, diffs, mask, 1.0 / 255);

					// Binarize using the Otsu method (good for bimodal images)
					cv::threshold(mask, mask, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

					// Copy mask matrix to find contours (as it is a destructive operation)
					cv::Mat mask_contours;
					mask.copyTo(mask_contours);

					// Find contours
					std::vector<std::vector<cv::Point> > contours;
					cv::findContours(mask_contours, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

					// At least two must be found
					if(contours.size() < 2)
					{
						throw std::logic_error("Found less than 2 contours");
					}

					// Calculate contour areas
					std::vector<double> sizes;
					for(int i = 0; i < contours.size(); i++)
					{
						sizes.push_back(cv::contourArea(contours[i]));
					}

					// Sort by contour size
					cv::Mat1i indexes_contours;
					cv::Mat1d size_mat(sizes);
					cv::sortIdx(size_mat, indexes_contours, CV_SORT_EVERY_COLUMN | CV_SORT_DESCENDING);

					// Calculate corners of the two biggest contours
					cv::Point2f contour_corners[2][4];
					for(int contour_i = 0; contour_i < 2; contour_i++)
					{
						// Compute the convex hull
						std::vector<cv::Point> hull;
						cv::convexHull(cv::Mat(contours[indexes_contours(contour_i,0)]), hull, false);

						// Approximate convex hull
						cv::Mat hull_mat = cv::Mat(hull);
						std::vector<cv::Point> screen_approx;
						cv::approxPolyDP(hull_mat, screen_approx, cv::arcLength(hull_mat, true) * 0.01, true);

						// Check if the approximation has 4 vertices
						if(screen_approx.size() != 4)
						{
							throw std::logic_error("One of the screen approximations doesn't have 4 corners");
						}

						// Calculate center
						cv::Point center(0, 0);
						for(int point_i = 0; point_i < 4; point_i++)
						{
							center += screen_approx[point_i];
						}
						center *= (1.0 / 4);

						// Calculate angle of the (point - center) vector to the horizontal line for each point
						std::vector<double> angles;
						for(int point_i = 0; point_i < 4; point_i++)
						{
							cv::Point d = screen_approx[point_i] - center;
							angles.push_back(atan2(d.y, d.x));
						}

						// Sort points by angle to the center
						cv::Mat1i indexes_points;
						cv::Mat1d angle_mat(angles);
						cv::sortIdx(angle_mat, indexes_points, CV_SORT_EVERY_COLUMN | CV_SORT_ASCENDING);

						// Save the points in order
						for(int point_i = 0; point_i < 4; point_i++)
						{
							contour_corners[contour_i][point_i] = screen_approx[indexes_points(point_i,0)];
						}
					}

					// Identify top and bottom screens
					if(contour_corners[0][0].x < contour_corners[1][0].x)
					{
						for(int point_i = 0; point_i < 4; point_i++)
						{
							top_screen.corners[point_i]    = contour_corners[0][point_i];
							bottom_screen.corners[point_i] = contour_corners[1][point_i];
						}
					}
					else
					{
						for(int point_i = 0; point_i < 4; point_i++)
						{
							top_screen.corners[point_i]    = contour_corners[1][point_i];
							bottom_screen.corners[point_i] = contour_corners[0][point_i];
						}
					}

					// Calculate transformation parameters for the corrected image of each screen
					calculateTransformationParameters();

					// Paint debug image
					cv_bridge::CvImagePtr cv_image_debug = cv_bridge::toCvCopy(lastImage, sensor_msgs::image_encodings::RGB8);

					// Draw contour lines
					for(int point_i = 0; point_i < 4; point_i++)
					{
						cv::line(cv_image_debug->image, top_screen.corners[point_i],    top_screen.corners[(point_i+1) % 4],    cv::Scalar(0, 255, 0), 2);
						cv::line(cv_image_debug->image, bottom_screen.corners[point_i], bottom_screen.corners[(point_i+1) % 4], cv::Scalar(0, 255, 0), 2);
					}

					// Draw corner points
					cv::circle(cv_image_debug->image, top_screen.corners[0],    10, cv::Scalar(255,   0,   0), 2);
					cv::circle(cv_image_debug->image, bottom_screen.corners[0], 10, cv::Scalar(255,   0,   0), 2);

					cv::circle(cv_image_debug->image, top_screen.corners[1],    10, cv::Scalar(  0, 255,   0), 2);
					cv::circle(cv_image_debug->image, bottom_screen.corners[1], 10, cv::Scalar(  0, 255,   0), 2);

					cv::circle(cv_image_debug->image, top_screen.corners[2],    10, cv::Scalar(  0,   0, 255), 2);
					cv::circle(cv_image_debug->image, bottom_screen.corners[2], 10, cv::Scalar(  0,   0, 255), 2);

					cv::circle(cv_image_debug->image, top_screen.corners[3],    10, cv::Scalar(255, 255,   0), 2);
					cv::circle(cv_image_debug->image, bottom_screen.corners[3], 10, cv::Scalar(255, 255,   0), 2);

					// Draw text
					cv::putText(cv_image_debug->image, "TOP",    top_screen.corners[0]    + cv::Point2f(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
					cv::putText(cv_image_debug->image, "BOTTOM", bottom_screen.corners[0] + cv::Point2f(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

					// Draw dividing lines
					//cv::line(cv_image_debug->image, cv::Point(cv_image_debug->image.cols / 2, 0), cv::Point(cv_image_debug->image.cols / 2, cv_image_debug->image.rows), cv::Scalar(0, 0, 0), 2);
					//cv::line(cv_image_debug->image, cv::Point(0, cv_image_debug->image.rows / 2), cv::Point(cv_image_debug->image.cols, cv_image_debug->image.rows / 2), cv::Scalar(0, 0, 0), 2);

					// Draw camera center
					//cv::circle(cv_image_debug->image, cv::Point(cam_model.cx(), cam_model.cy()), 10, cv::Scalar(255, 93, 0), 2);

					// Report successful calibration
					response.success.data = true;
					response.image_debug = *cv_image_debug->toImageMsg();
            	}
            	catch(const std::logic_error& e)
            	{
            		// Report unsuccessful calibration
            		response.success.data = false;
					response.image_debug = *lastImage;
            	}

            	return true;
            }
    };
}

PLUGINLIB_DECLARE_CLASS(robostylus_camera, RobostylusCameraNodelet, robostylus_camera::RobostylusCameraNodelet, nodelet::Nodelet);
