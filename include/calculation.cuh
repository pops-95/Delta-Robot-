#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <array>
#include <vector>

#define INITIAL_X_DELTA 1101.0f
#define INITIAL_Y_DELTA 210.0f

struct Position {
    float x;
    float y;
};

struct Camera_Point {
    float x;
    float y;
    float z;
};

// 1. Define a struct to hold your grouped data cleanly
struct ValidPosition {
    float original_idx;
    float slider_x;
    float slider_y;
};


class DeltaCalculation {
    public:
        DeltaCalculation() {
            initialize();
        }
        void initialize();
        void calculate_object_position(Position* object_position,Camera_Point* camera_point);
        Position get_object_position() ;
        Position object_position;
        void cuda_information();
        std::vector<ValidPosition> get_possible_sliders(Position* host_object_position);
        std::vector<std::vector<ValidPosition>> get_multiple_possible_sliders(const std::vector<Position>& host_object_positions);

    private:
        Position current_position;
        cudaDeviceProp deviceProp;
        int device_id;
        Camera_Point camera_point;
        std::array<float,4> real_world_coefficients_x={-0.01199f,0.5892f,1.125f,-345.707f};
        std::array<float,4> real_world_coefficients_y={1.029f,-0.941f,-0.3271f,481.2f};
        std::array<float,6> delta_coefficients={0.47f,0.906f,4.391f,1.033f,-0.495f,13.42f};





        




};