#include "Utils.hpp"


Camera::Camera(const std::string& serial, const std::string& name,
       color_frame_info color_info, depth_frame_info depth_info,rs2::context& ctx,
           rs2::config cfg)
{
    // Default Constructor
    this->serial = serial;
    this->name = name;
    this->color_info = color_info;
    this->depth_info = depth_info;

    this->pipe = rs2::pipeline(ctx);
    pipe.start(cfg);
}


// Camera::~Camera()
// {
//     // Destructor
//     pipe.stop();
// }

Camera::Camera(color_frame_info color_info, depth_frame_info depth_info)
{
    this->color_info = color_info;
    this->depth_info = depth_info;
}

void Camera::change_depth_frame(depth_frame_info &depth_info)
{
    this->depth_info = depth_info;
}

void Camera::change_color_frame(color_frame_info &color_info)
{
    this->color_info = color_info;
}

rs2::frameset Camera::get_frames()
{
    try
    {
        return pipe.wait_for_frames();
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error calling wait_for_frames(): " << e.what() <<"From camera " << serial << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Standard exception calling wait_for_frames(): " << e.what() << std::endl;
    }
    // Return an empty frameset if an error occurs
    return rs2::frameset();
}

rs2::frameset Camera::aligned_frames(rs2::frameset frames)
{
    if (frames.size() == 0)
    {
        std::cout << "No frames " << std::endl; // Return empty frameset if no frames are available
    }
    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2::frameset aligned_frames = align_to_color.process(frames);
    return aligned_frames;
}

void Camera::set_exposure(int value, bool auto_exposure)
{
    // Get device from pipeline
    rs2::pipeline_profile profile = pipe.get_active_profile();
    rs2::device dev = profile.get_device();

    // Find the first sensor that supports color stream
    for (auto &&sensor : dev.query_sensors())
    {
        if (sensor.supports(RS2_OPTION_EXPOSURE))
        {
            if (auto_exposure)
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
            }
            else
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
                sensor.set_option(RS2_OPTION_EXPOSURE, static_cast<float>(value));
            }
        }
    }
}

cv::Mat Camera::get_rgb_image(rs2::frameset frames)
{
    // rs2::frameset frames = get_frames();
    // rs2::frameset aligned = aligned_frames(frames);
    rs2::video_frame color_frame = frames.get_color_frame();

    if (!color_frame)
    {
        std::cerr << "No color frame available!" << std::endl;
        return cv::Mat();
    }

    // Create OpenCV matrix from color frame data
    cv::Mat image(
        color_info.color_height,
        color_info.color_width,
        CV_8UC3,
        (void *)color_frame.get_data(),
        color_frame.get_stride_in_bytes());

    // Convert from BGR to RGB if needed
   // cv::Mat rgb_image;
    // cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);

    return image;
}

cv::Mat Camera::get_colorized_depth_image(rs2::depth_frame depth_frame)
{
    if (!depth_frame)
    {
        std::cerr << "No depth frame available!" << std::endl;
        return cv::Mat();
    }

    // Colorize the depth frame using RealSense colorizer
    rs2::colorizer color_map;
    rs2::frame colorized = color_map.process(depth_frame);

    // Cast to rs2::video_frame for get_stride_in_bytes()
    rs2::video_frame colorized_video = colorized.as<rs2::video_frame>();

    cv::Mat colorized_image(
        depth_info.depth_height,
        depth_info.depth_width,
        CV_8UC3,
        (void *)colorized_video.get_data(),
        colorized_video.get_stride_in_bytes());

    return colorized_image;
}

cv::Mat Camera::threshold_depth_frame(const rs2::depth_frame &depth_frame, uint16_t threshold)
{
    int width = depth_frame.get_width();
    int height = depth_frame.get_height();

    // Create a CV_16U Mat from depth frame data
    cv::Mat depth_mat(height, width, CV_16U, (void*)depth_frame.get_data(), depth_frame.get_stride_in_bytes());

    // Clone to avoid modifying the original frame
    cv::Mat thresholded = depth_mat.clone();

    for (int y = 0; y < thresholded.rows; ++y)
    {
        uint16_t* row_ptr = thresholded.ptr<uint16_t>(y);
        for (int x = 0; x < thresholded.cols; ++x)
        {
            if (row_ptr[x] > threshold)
                row_ptr[x] = 0;
        }
    }

    return thresholded;
}


intrinsics_info Camera::get_color_intrinsics(rs2::pipeline &pipe)
{
    rs2::pipeline_profile profile = pipe.get_active_profile();
    rs2::video_stream_profile color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = color_stream.get_intrinsics();

    intrinsics_info info;
    info.fx = intr.fx;
    info.fy = intr.fy;
    info.ppx = intr.ppx;
    info.ppy = intr.ppy;
    info.width = intr.width;
    info.height = intr.height;
    for (int i = 0; i < 5; ++i)
        info.coeffs[i] = intr.coeffs[i];
    return info;
}


cv::Point3f Camera::pixel_to_global(int u, int v, float depth, const intrinsics_info& intr)
{
    // Convert pixel (u,v) and depth to camera coordinates (X,Y,Z)
    float X = (u - intr.ppx) / intr.fx * depth;
    float Y = (v - intr.ppy) / intr.fy * depth;
    float Z = depth;
    return cv::Point3f(X, Y, Z);
}

rs2::depth_frame Camera::disparity_to_depth(const rs2::depth_frame& input_disparity_frame)
{
    rs2::disparity_transform disp_to_depth(false); // false = disparity to depth
    rs2::frame depth = disp_to_depth.process(input_disparity_frame);
    return depth;
}

rs2::depth_frame Camera::dpeth_to_disparity(const rs2::depth_frame& input_disparity_frame)
{
    rs2::disparity_transform disp_to_depth(true); // false = disparity to depth
    rs2::frame depth = disp_to_depth.process(input_disparity_frame);
    return depth;
}


rs2::depth_frame Camera::process_depth_filters(const rs2::depth_frame& input_depth_frame)
{
    // Create filter objects
    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1); // Keep original size

    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;

    // Apply decimation filter
    rs2::frame decimated = dec_filter.process(input_depth_frame);

    rs2::frame desp_frame= dpeth_to_disparity(decimated);

    // Apply spatial filter
    rs2::frame spatial = spatial_filter.process(desp_frame);

    // Apply temporal filter
    rs2::frame temporaled = temporal_filter.process(spatial);

    rs2::frame des_depth_frame= disparity_to_depth(temporaled);

    // Return the processed frame as a depth_frame
    return des_depth_frame.as<rs2::depth_frame>();
}



void Camera::camera_operation(rs2::frameset &frames, Camera &camera,camera_frames &frames_out) {
    // rs2::frameset frames = camera.get_frames();
    if (frames.size() == 0) {
        std::cout << "No frames received from camera." << std::endl;
        return;
    }
    rs2::frameset aligned = aligned_frames(frames);

    cv::Mat color_img = camera.get_rgb_image(aligned);
    if (color_img.empty()) {
        std::cout << "Color image is empty." << std::endl;
    }

    rs2::depth_frame depth_frame = aligned.get_depth_frame();
    if (!depth_frame) {
        std::cout << "No depth frame available." << std::endl;
    }

    if (depth_frame)
        depth_frame =camera.process_depth_filters(depth_frame);
    
    camera.depth_frame = depth_frame;

    cv::Mat depth_thresh_img;
    if (depth_frame)
        depth_thresh_img = camera.threshold_depth_frame(depth_frame, threshold_value);

    cv::Mat depth_display;
    if (!depth_thresh_img.empty())
        cv::normalize(depth_thresh_img, depth_display, 0, 255, cv::NORM_MINMAX, CV_8U);

    // std::cout << "Color: " << color_img.cols << "x" << color_img.rows
    //           << ", Depth: " << depth_display.cols << "x" << depth_display.rows << std::endl;

    if (!color_img.empty()){
        // cv::cvtColor(color_img, color_img, cv::COLOR_RGB2BGR);
        frames_out.color_image = color_img;
    }
        
    else
        frames_out.color_image.release();

    if (!depth_display.empty())
        frames_out.depth_image = depth_display;
    else
        frames_out.depth_image.release();
}

bool Camera::verify_pipeline_serial()
{
    try {
        rs2::pipeline_profile profile = pipe.get_active_profile();
        rs2::device dev = profile.get_device();
        std::string p_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        if (p_serial == this->serial) {
            std::cout << "Camera serial matches: " << p_serial << std::endl;
            return true;
        } else {
            std::cerr << "Serial mismatch! Expected " << this->serial
                      << " but pipeline is associated with " << p_serial << std::endl;
            return false;
        }
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error in verify_pipeline_serial(): "
                  << e.what() << std::endl;
        return false;
    }
    catch (const std::exception& e) {
        std::cerr << "Standard exception in verify_pipeline_serial(): "
                  << e.what() << std::endl;
        return false;
    }
}





















