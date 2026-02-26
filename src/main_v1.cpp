// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <Utils.hpp>
// #include "example.hpp"              // Include short list of convenience functions for rendering

#include <map>
#include <vector>

// Struct to hold extra data for each window
struct MouseData
{
    std::string windowName;
    Camera *opened_cam; // pointer to the image (if needed)
    // Camera opened_cam; // pointer to the image (if needed)
    int id; // some identifier
};

static void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        MouseData *data = reinterpret_cast<MouseData *>(userdata);
        if (data->id == 1)
        {
            // std::cout << "Left button of mouse is clicked - position (" << x << ", " << y << ") in window " << data->windowName << std::endl;
            std::cout << std::fixed << std::setprecision(2) << "Position at this pixel: "<<x << "," <<y <<" is " << data->opened_cam->pixel_to_global(x, y, data->opened_cam->depth_frame->get_distance(x, y)*1000, data->opened_cam->get_color_intrinsics(data->opened_cam->get_pipeline())) << std::endl;
        }
        if (data->id == 2)
        {
            // std::cout << "Left button of mouse is clicked - position (" << x << ", " << y << ") in window " << data->windowName << std::endl;
            std::cout << std::fixed << std::setprecision(2) << "Depth at this pixel: " << data->opened_cam->depth_frame->get_distance(x, y) * 1000 << std::endl;
        }
    }
}

int main(int argc, char *argv[])
{
    // Create a simple OpenGL window for rendering:
    // window app(1280, 960, "CPP Multi-Camera Example");

    rs2::context ctx; // Create librealsense context for managing devices

    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)

    std::vector<rs2::pipeline> pipelines;

    // Capture serial numbers before opening streaming
    std::vector<std::string> serials;
    std::vector<Camera> cameras;
    std::vector<camera_frames> frames_vec;

    // Create two windows
    std::string win1 = "Camera 1 - RGB";
    std::string win2 = "Camera 2 - RGB";

    // Example: let’s allow user choice
    color_frame_info cinfo = {1280, 720, 30}; // default
    depth_frame_info dinfo = {1280, 720, 30}; // default
    // Camera cam;
    for (auto &&dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    // Start a streaming pipe per each connected device
    for (auto &&serial : serials)
    {

        rs2::config cfg;

        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_COLOR, cinfo.color_width, cinfo.color_height, RS2_FORMAT_BGR8, cinfo.color_fps);
        cfg.enable_stream(RS2_STREAM_DEPTH, dinfo.depth_width, dinfo.depth_height, RS2_FORMAT_Z16, dinfo.depth_fps);
        cameras.emplace_back(serial, "Camera_" + serial, cinfo, dinfo, ctx, cfg);

        frames_vec.emplace_back(serial);
        // serials.push_back(serial);
        std::cout << "Started streaming from device " << serial << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(350));
    }

    // // Main app loop
    while (true)
    {

        for (int i = 0; i < cameras.size(); i++)
        {
            rs2::frameset fs = cameras[i].get_frames();
            if (fs.size() > 0)
            {
                // std::cout<< "Got frames from camera " << cameras[i].serial << std::endl;
                cameras[i].camera_operation(fs, cameras[i], frames_vec[i]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (cameras.size() < 2)
        {
            // Setup mouse data for each window

            cv::namedWindow(win1);

            MouseData data1{win1, &cameras[0], 1};
            // Register callbacks with userdata
            cv::setMouseCallback(win1, onMouse, &data1);

            // Show both RGB images in separate windows
            // if (!frames_vec[0].color_image.empty()){
            //     cv::line (frames_vec[0].color_image, cv::Point(360, 718), cv::Point(524,298), cv::Scalar(0, 255, 0), 2);
            //     cv::line (frames_vec[0].color_image, cv::Point(1100, 718), cv::Point(805,303), cv::Scalar(0, 255, 0), 2);
            //     cv::line (frames_vec[0].color_image, cv::Point(805,303), cv::Point(524,298), cv::Scalar(0, 255, 0), 2);
            //     cv::imshow("Camera 1 - RGB", frames_vec[0].color_image);
            // }
            if (!frames_vec[0].color_image.empty()){
                
                // 1. Define the 4 corners of your green boundary in image pixels
                // (Using the exact coordinates from your existing cv::line calls)
                std::vector<cv::Point2f> image_pts = {
                    cv::Point2f(524, 298),  // Top-Left 
                    cv::Point2f(805, 303),  // Top-Right 
                    cv::Point2f(1100, 718), // Bottom-Right 
                    cv::Point2f(360, 718)   // Bottom-Left 
                };

                // 2. Define the REAL-WORLD dimensions of that green rectangle in mm
                // IMPORTANT: Replace these with your actual physical measurements of the frame!
                float phys_width_mm = 467.0f;  // Example: 600mm wide
                float phys_height_mm = 990.0f; // Example: 800mm deep

                std::vector<cv::Point2f> world_pts = {
                    cv::Point2f(0, 0),                            // Top-Left
                    cv::Point2f(phys_width_mm, 0),                // Top-Right
                    cv::Point2f(phys_width_mm, phys_height_mm),   // Bottom-Right
                    cv::Point2f(0, phys_height_mm)                // Bottom-Left
                };

                // 3. Compute the Homography matrix (Maps World coordinates TO Image coordinates)
                cv::Mat H = cv::findHomography(world_pts, image_pts);

                // 4. Draw the grid in 10cm (100mm) increments
                int grid_size_mm = 100; 

                // Draw Vertical Grid Lines
                for (int x = 0; x <= phys_width_mm; x += grid_size_mm) {
                    std::vector<cv::Point2f> line_world = { cv::Point2f(x, 0), cv::Point2f(x, phys_height_mm) };
                    std::vector<cv::Point2f> line_image;
                    
                    // Transform the straight world line into the skewed perspective of the camera
                    cv::perspectiveTransform(line_world, line_image, H);
                    cv::line(frames_vec[0].color_image, line_image[0], line_image[1], cv::Scalar(0, 150, 0), 1);
                }

                // Draw Horizontal Grid Lines
                for (int y = 0; y <= phys_height_mm; y += grid_size_mm) {
                    std::vector<cv::Point2f> line_world = { cv::Point2f(0, y), cv::Point2f(phys_width_mm, y) };
                    std::vector<cv::Point2f> line_image;
                    
                    cv::perspectiveTransform(line_world, line_image, H);
                    cv::line(frames_vec[0].color_image, line_image[0], line_image[1], cv::Scalar(0, 150, 0), 1);
                }

                // 5. Draw the outer green boundary box
                std::vector<cv::Point> int_image_pts;
                for (const auto& pt : image_pts) int_image_pts.push_back(pt); // Convert to int for polylines
                cv::polylines(frames_vec[0].color_image, int_image_pts, true, cv::Scalar(0, 255, 0), 2);

                cv::imshow("Camera 1 - RGB", frames_vec[0].color_image);
            }
                
        }

        else
        {

            cv::namedWindow(win1);
            cv::namedWindow(win2);
            // Setup mouse data for each window
            MouseData data1{win1, &cameras[0], 1};
            MouseData data2{win2, &cameras[1], 2};

            // Register callbacks with userdata
            cv::setMouseCallback(win1, onMouse, &data1);
            cv::setMouseCallback(win2, onMouse, &data2);

            // Show both RGB images in separate windows
            if (!frames_vec[0].color_image.empty())
                cv::imshow("Camera 1 - RGB", frames_vec[0].color_image);

            if (!frames_vec[1].color_image.empty())
                cv::imshow("Camera 2 - RGB", frames_vec[1].color_image);
        }

        // Update every frame, press ESC to exit
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }

    return EXIT_SUCCESS;
}
