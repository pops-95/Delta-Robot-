// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <Utils.hpp>
#include "calculation.cuh"
// #include "example.hpp"              // Include short list of convenience functions for rendering

#include <map>
#include <vector>
#include <random>

// Struct to hold extra data for each window
struct MouseData
{
    std::string windowName;
    Camera *opened_cam; // pointer to the image (if needed)
    // Camera opened_cam; // pointer to the image (if needed)
    int id; // some identifier
};

std::random_device rd;
// 2. Initialize the Standard Mersenne Twister engine with the seed
std::mt19937 gen(rd());

Camera_Point points;
cv::Point3f temp_data;
DeltaCalculation delta_calculator;
std::vector<ValidPosition> final_solutions;
std::vector<Position> selected_points; // Store selected points for later use
std::vector<std::vector<ValidPosition>>all_solutions;

struct Range { float start; float end; bool isValid; };

// A simple struct to hold just the X/Y coordinates
struct SliderCoordinate {
    float slider_x;
    float slider_y;
};

// The final result structure containing the chosen index and its corresponding coordinates
struct RandomSelectionResult {
    int chosen_idx;
    std::vector<SliderCoordinate> coordinates;
    bool success;
};

RandomSelectionResult get_random_slider_points(
    const std::vector<std::vector<ValidPosition>>& all_solutions, 
    const Range& common_range) 
{
    RandomSelectionResult result;
    result.success = false;

    // 1. Validate the range
    if (!common_range.isValid || all_solutions.empty()) {
        std::cerr << "Invalid range or empty datasets provided." << std::endl;
        return result;
    }

    // Convert the float range back to integers since they represent thread indices
    int min_idx = static_cast<int>(common_range.start);
    int max_idx = static_cast<int>(common_range.end);

    if (min_idx > max_idx) {
        std::cerr << "Error: min_idx is greater than max_idx." << std::endl;
        return result;
    }

    // 2. Setup the Random Number Generator
    std::random_device rd;  // Obtain a random number from hardware
    std::mt19937 gen(rd()); // Seed the standard Mersenne Twister engine
    std::uniform_int_distribution<> distrib(min_idx, max_idx);

    // Pick our random original_idx
    result.chosen_idx = distrib(gen);
    // std::cout << "Randomly selected original_idx: " << result.chosen_idx << std::endl;

    // 3. Search each dataset for the chosen index
    for (size_t i = 0; i < all_solutions.size(); ++i) {
        const auto& dataset = all_solutions[i];
        bool found = false;

        for (const auto& pos : dataset) {
            // Because original_idx is a float, we use a small epsilon (0.1f) for safe comparison, 
            // even though it was originally cast from an integer.
            if (std::abs(pos.original_idx - static_cast<float>(result.chosen_idx)) < 0.1f) {
                result.coordinates.push_back({pos.slider_x, pos.slider_y});
                found = true;
                break; // Stop searching this dataset once we found the match
            }
        }

        if (!found) {
            std::cerr << "Warning: Index " << result.chosen_idx << " was not found in dataset " << i 
                      << "! (This shouldn't happen if the common range is calculated correctly)." << std::endl;
        }
    }

    result.success = true;
    return result;
}

Range find_common_idx_range(const std::vector<std::vector<ValidPosition>>& all_solutions) {
    // If we didn't process any datasets, return invalid
    if (all_solutions.empty()) {
        std::cerr << "No datasets provided to find_common_idx_range." << std::endl;
        return {0.0f, 0.0f, false};
    }

    // Initialize our "common" range to extremely wide values.
    // As we process each dataset, this window will shrink.
    float common_start = -999999.0f; 
    float common_end   = 999999.0f;  

    // Loop through the separate results for each dataset
    for (size_t i = 0; i < all_solutions.size(); ++i) {
        const std::vector<ValidPosition>& current_dataset = all_solutions[i];
        
        // If ANY dataset has 0 valid positions, it's impossible to have a common overlap
        if (current_dataset.empty()) {
            return {0.0f, 0.0f, false};
        }

        // 1. Find the min and max original_idx for THIS specific dataset
        float min_idx = current_dataset[0].original_idx;
        float max_idx = current_dataset[0].original_idx;

        for (const auto& pos : current_dataset) {
            if (pos.original_idx < min_idx) min_idx = pos.original_idx;
            if (pos.original_idx > max_idx) max_idx = pos.original_idx;
        }

        // 2. Shrink our common overlapping window
        // The new common start is the MAXIMUM of the current starts
        // The new common end is the MINIMUM of the current ends
        if (common_start == -999999.0f) {
            common_start = min_idx;
            common_end = max_idx;
        } else {
            common_start = std::max(common_start, min_idx);
            common_end   = std::min(common_end, max_idx);
        }
    }

    // 3. Final Check: Does a valid overlap actually exist?
    Range final_overlap;
    if (common_start <= common_end) {
        final_overlap.start = common_start;
        final_overlap.end = common_end;
        final_overlap.isValid = true;
    } else {
        final_overlap.start = 0.0f;
        final_overlap.end = 0.0f;
        final_overlap.isValid = false;
    }
    std::cout << "Calculated common index range: [" << common_start << ", " << common_end << "]" 
              << " - Valid Overlap: " << (final_overlap.isValid ? "Yes" : "No") << std::endl;

    return final_overlap;
}

static void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {   
        if (!userdata) return;
        MouseData *data = reinterpret_cast<MouseData *>(userdata);
        if (data->id == 1)
        {   
            temp_data = data->opened_cam->pixel_to_global(x, y, data->opened_cam->depth_frame->get_distance(x, y)*1000, data->opened_cam->get_color_intrinsics(data->opened_cam->get_pipeline()));
            
            points.x = temp_data.x;
            points.y = temp_data.y;
            if (temp_data.z != 0.0f) {
                points.z = temp_data.z;
                 // std::cout<<"POINTS: "<<points.x<<","<<points.y<<","<<points.z<<std::endl;
                delta_calculator.calculate_object_position(&delta_calculator.object_position, &points);
                selected_points.push_back(delta_calculator.object_position); // Store the selected point
                std::cout << "Total selected points: " << selected_points.size() << std::endl;
            }
            else{
                std::cout<<"Depth data is zero at this pixel. Skipping position calculation."<<std::endl;
            }
           
            // final_solutions = delta_calculator.get_possible_sliders(&delta_calculator.object_position);
            // cudaDeviceSynchronize();
            // // std::cout << "Left button of mouse is clicked - position (" << x << ", " << y << ") in window " << data->windowName << std::endl;
            // // std::cout << std::fixed << std::setprecision(2) << "Position at this pixel: "<<x << "," <<y <<" is " << data->opened_cam->pixel_to_global(x, y, data->opened_cam->depth_frame->get_distance(x, y)*1000, data->opened_cam->get_color_intrinsics(data->opened_cam->get_pipeline())) << std::endl;
            // // std::cout << std::fixed << std::setprecision(2) << "Position at this pixel: "<<x << "," <<y <<" is " << points.x<<","<<points.y<<","<<points.z<<std::endl;
            // std::cout << std::fixed << std::setprecision(2) << "Object position: " << delta_calculator.object_position.x<<","<<delta_calculator.object_position.y<<std::endl;
            // std::cout << "Found " << final_solutions.size() << " possible slider positions." << std::endl;
            // // std::cout<< std::fixed << std::setprecision(2) << "Possible slider positions (x, y):" << std::endl;
            // // 3. Define the distribution range (inclusive)
            // std::uniform_int_distribution<int> distrib(0, final_solutions.size());
            //  // 4. Generate the number
            // int random_number = distrib(gen);
            // std::cout << "Randomly selected slider position: (" << final_solutions[random_number].slider_x << ", " << final_solutions[random_number].slider_y << ")" <<"At location " <<final_solutions[random_number].original_idx << std::endl;
            
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
    //delta_calculator.cuda_information();
    rs2::context ctx; // Create librealsense context for managing devices
    std::cout << "Process Started" << std::endl;
    std::cout <<"Press 'C' to clear selected points, 'S' to compute solutions and select random slider positions."<<std::endl;

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
    MouseData data1;
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

            data1 = {win1, &cameras[0], 1};
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


        char key = (char)cv::waitKey(1);
        // Update every frame, press ESC to exit
        if (key == 27)
        {
            break;
        }

        else if (key == 'c' || key == 'C')
        {
            // Clear selected points and solutions
            if(!selected_points.empty()){
                selected_points.pop_back();
                std::cout << "Last selected point removed. Remaining points: " << selected_points.size() << std::endl;
            }
        }

        else if(key== 's' || key == 'S')
        {
           all_solutions=delta_calculator.get_multiple_possible_sliders(selected_points);
            Range common_range = find_common_idx_range(all_solutions);
            if(common_range.isValid) {
                std::cout << "Common index range across all datasets: [" << common_range.start << ", " << common_range.end << "]" << std::endl;
                RandomSelectionResult random_result = get_random_slider_points(all_solutions, common_range);
                if(random_result.success) {
                    std::cout << "Randomly selected original_idx: " << random_result.chosen_idx << std::endl;
                    for(size_t i = 0; i < random_result.coordinates.size(); ++i) {
                        std::cout << "Dataset " << i << " - Slider Coordinates: (" 
                                  << random_result.coordinates[i].slider_x << ", " 
                                  << random_result.coordinates[i].slider_y << ")" << std::endl;
                    }
                selected_points.clear(); // Clear selected points after processing
                } else {
                    std::cerr << "Failed to select random slider points." << std::endl;
                }
            } else {
                std::cerr << "No common index range found across datasets. Cannot select random slider points." << std::endl;
            }
        }



    }

    return EXIT_SUCCESS;
}


