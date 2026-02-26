#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>

typedef struct{
    int depth_width;
    int depth_height;
    int depth_fps;
}depth_frame_info;

typedef struct{
    int color_width;
    int color_height;
    int color_fps;
}color_frame_info;

typedef struct {
    float fx, fy;      // Focal lengths
    float ppx, ppy;    // Principal points
    float coeffs[5];   // Distortion coefficients
    int width, height; // Image dimensions
} intrinsics_info;

struct camera_frames {
    cv::Mat color_image;
    cv::Mat depth_image;

    camera_frames() = default;
    camera_frames(const std::string& name) {
        std::cout << "Camera frames struct created for " << name << std::endl;
    }
};





class Camera{
    public:
        Camera(const std::string& serial, const std::string& name,
        color_frame_info color_info, depth_frame_info depth_info,rs2::context& ctx,
           rs2::config cfg);
        // ~Camera();
        Camera (color_frame_info color_info, depth_frame_info depth_info);
        Camera(const std::string& serial) {this->serial = serial; };
        Camera() = default;
        std::optional<rs2::depth_frame> depth_frame;
        // void enable_streams();
        // void start_camera();
        void get_serial(std::string& serial_out) { serial_out = serial; }
        void change_depth_frame(depth_frame_info& depth_info); 
        void change_color_frame(color_frame_info& color_info);
   
        rs2::frameset aligned_frames(rs2::frameset frames);
        rs2::frameset get_frames();

        void set_exposure(int value, bool auto_exposure);

        cv::Mat Camera::get_rgb_image(rs2::frameset frames);
        cv::Mat Camera::get_colorized_depth_image(rs2::depth_frame depth_frame);
        cv::Point3f pixel_to_global(int u, int v, float depth, const intrinsics_info& intr);
        cv::Mat threshold_depth_frame(const rs2::depth_frame& depth_frame, uint16_t threshold);

        rs2::depth_frame Camera::process_depth_filters(const rs2::depth_frame& input_depth_frame);
        void Camera::camera_operation(rs2::frameset &frames,Camera &camera, camera_frames &frames_out);
        void set_pipeline(rs2::pipeline& pipeline) { this->pipe = pipeline; };
        void get_color_frame_info(color_frame_info& color_info_out) { color_info_out = color_info; }
        void get_depth_frame_info(depth_frame_info& depth_info_out) { depth_info_out = depth_info; }
        intrinsics_info get_color_intrinsics(rs2::pipeline &pipe);
        std::string serial;
        bool Camera::verify_pipeline_serial();
        rs2::pipeline get_pipeline() { return this->pipe; };


    private:
        rs2::pipeline pipe;
        // Show color and thresholded depth images from the first camera
        uint16_t threshold_value = 2000; // Example: 2000 units (usually mm, i.e. 2 meters)
        // rs2::config cfg;
        // rs2::context ctx;
        color_frame_info color_info;
        depth_frame_info depth_info;
        
        std::string name;
        rs2::depth_frame Camera::disparity_to_depth(const rs2::depth_frame& input_disparity_frame);
        rs2::depth_frame Camera::dpeth_to_disparity(const rs2::depth_frame& input_disparity_frame);
        // rs2::depth_frame depth_frame;
        

        
};

