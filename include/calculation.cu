#include <calculation.cuh>
#include <iostream>

// 1. THE ERROR CHECKER MACRO
#define CUDA_CHECK(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA Error at " << __FILE__ << ":" << __LINE__ \
                      << " - " << cudaGetErrorString(err) << std::endl; \
        } \
    } while (0)

void DeltaCalculation::initialize() {
    current_position.x = INITIAL_X_DELTA;
    current_position.y = INITIAL_Y_DELTA;
}

__global__ void calulate_object_position_x(std::array<float,4> coeffecient,Camera_Point* camera_point,Position* object_position) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < 1) { // Only one thread needed for this calculation
        float x = camera_point->x;
        float y = camera_point->y;
        float z = camera_point->z;

        // Calculate the real-world position using the coefficients
        object_position->x = coeffecient[0] * x + coeffecient[1] * y + coeffecient[2] * z + coeffecient[3];
        
    }
}
__global__ void calulate_object_position_y(std::array<float,4> coeffecient,Camera_Point* camera_point,Position* object_position) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < 1) { // Only one thread needed for this calculation
        float x = camera_point->x;
        float y = camera_point->y;
        float z = camera_point->z;

        // Calculate the real-world position using the coefficients
        object_position->y = coeffecient[0] * x + coeffecient[1] * y + coeffecient[2] * z + coeffecient[3];
        
    }
}


void DeltaCalculation::calculate_object_position(Position* object_position,Camera_Point* camera_point) {
    std::cout<<"Calculating object position on GPU..."<<std::endl;
    Position* d_object_position;
    Camera_Point* d_camera_point;

    // Allocate memory on the device
    CUDA_CHECK(cudaMalloc(&d_object_position, sizeof(Position)));
    CUDA_CHECK(cudaMalloc(&d_camera_point, sizeof(Camera_Point)));

    // Copy data from host to device
    CUDA_CHECK(cudaMemcpy(d_camera_point, camera_point, sizeof(Camera_Point), cudaMemcpyHostToDevice));

    // Launch the kernel to calculate the object position
    calulate_object_position_x<<<1, 1>>>(real_world_coefficients_x, d_camera_point, d_object_position);
    calulate_object_position_y<<<1, 1>>>(real_world_coefficients_y, d_camera_point, d_object_position);

    // 3. CHECK FOR KERNEL LAUNCH ERRORS
    CUDA_CHECK(cudaGetLastError()); 
    CUDA_CHECK(cudaDeviceSynchronize());

    CUDA_CHECK(cudaMemcpy(object_position, d_object_position, sizeof(Position), cudaMemcpyDeviceToHost));

    CUDA_CHECK(cudaFree(d_object_position));
    CUDA_CHECK(cudaFree(d_camera_point));
    
    // std::cout<<"Object position calculated: "<<object_position->x<<","<<object_position->y<<std::endl;
}


Position DeltaCalculation::get_object_position() {
    return object_position;
}

// A dummy kernel to test (You would replace this with your actual Delta Calculation kernel)
__global__ void exampleKernel(float* data, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) {
        data[idx] += 1.0f; 
    }
}

void DeltaCalculation::cuda_information(){
    CUDA_CHECK(cudaGetDevice(&device_id));
    CUDA_CHECK(cudaGetDeviceProperties(&deviceProp, device_id));
    std::cout << "=== GPU Hardware Limits ===" << std::endl;
    std::cout << "Device Name: " << deviceProp.name << std::endl;
    std::cout << "Max threads per block: " << deviceProp.maxThreadsPerBlock << std::endl;
    std::cout << "Max threads per Streaming Multiprocessor (SM): " << deviceProp.maxThreadsPerMultiProcessor << std::endl;
    std::cout << "Number of SMs: " << deviceProp.multiProcessorCount << std::endl;
    
    // Total theoretical concurrent threads across the entire GPU
    int maxTotalThreads = deviceProp.maxThreadsPerMultiProcessor * deviceProp.multiProcessorCount;
    std::cout << "Theoretical max concurrent threads on GPU: " << maxTotalThreads << std::endl;

    // ==========================================
    // 2. FINDING THE OPTIMAL BLOCK SIZE
    // ==========================================
    int minGridSize;   // The minimum number of blocks needed to max out the GPU
    int bestBlockSize; // The optimal number of threads per block

    // The Occupancy API automatically calculates the perfect block size 
    // by analyzing how many registers and memory 'exampleKernel' uses.
    cudaOccupancyMaxPotentialBlockSize(&minGridSize, &bestBlockSize, exampleKernel, 0, 0);

    std::cout << "\n=== Optimal Launch Configuration ===" << std::endl;
    std::cout << "For 'exampleKernel' to achieve maximum active threads:" << std::endl;
    std::cout << "-> Recommended Block Size: " << bestBlockSize << " threads per block." << std::endl;
    std::cout << "-> Minimum Grid Size: " << minGridSize << " blocks." << std::endl;
}

__global__ void possible_slider_pos(std::array<float,6> coeffecient,Position* object_position,ValidPosition* possible_positions,int* d_count, int max_capacity){ 
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx<600){
        float obj_x=object_position->x;
        float obj_y=object_position->y;
        float change_x=1101.0f-idx-obj_x;
        float change_y=210.0f-obj_y;

        float slider_x=coeffecient[0]*change_x+coeffecient[1]*change_y+coeffecient[2];
        float slider_y=coeffecient[3]*change_x+coeffecient[4]*change_y+coeffecient[5];

        if(slider_x>-150 && slider_x<150 && slider_y>-150 && slider_y<150){
            // THE FIX: Safely get a unique index for this specific thread
            int write_idx = atomicAdd(d_count, 1);
            // Safety check to ensure we don't write past the end of our allocated array
            if (write_idx < max_capacity) {
                possible_positions[write_idx].original_idx = (float)idx;
                possible_positions[write_idx].slider_x = slider_x;
                possible_positions[write_idx].slider_y = slider_y;
            }
        }    

    }
}

// A wrapper function to call the kernel and return a C++ vector
std::vector<ValidPosition> DeltaCalculation::get_possible_sliders(Position* host_object_position) {
    
    int max_solutions = 600; // The maximum possible number of loops your kernel runs
    
    // 1. ALLOCATE GPU MEMORY
    ValidPosition* d_possible_positions;
    int* d_count;
    
    // Allocate space for the maximum possible solutions
    cudaMalloc(&d_possible_positions, max_solutions * sizeof(ValidPosition));
    
    // Allocate space for our counter and initialize it to 0
    cudaMalloc(&d_count, sizeof(int));
    cudaMemset(d_count, 0, sizeof(int)); // Sets the value on the GPU to 0

    // (Assume you also allocate and copy your object_position here)
    Position* d_object_position;
    cudaMalloc(&d_object_position, sizeof(Position));
    cudaMemcpy(d_object_position, host_object_position, sizeof(Position), cudaMemcpyHostToDevice);

    // 2. LAUNCH KERNEL
    // Calculate optimal grid/block size for 600 threads
    int threadsPerBlock = 256;
    int blocksPerGrid = (max_solutions + threadsPerBlock - 1) / threadsPerBlock;
    
    possible_slider_pos<<<blocksPerGrid, threadsPerBlock>>>(
        delta_coefficients, 
        d_object_position, 
        d_possible_positions, 
        d_count, 
        max_solutions
    );
    
    cudaDeviceSynchronize();

    // 3. READ THE COUNTER BACK FIRST
    int host_count = 0;
    cudaMemcpy(&host_count, d_count, sizeof(int), cudaMemcpyDeviceToHost);
    
    std::cout << "The GPU found " << host_count << " valid solutions." << std::endl;

    // 4. PREPARE THE C++ VECTOR
    std::vector<ValidPosition> final_solutions;
    
    if (host_count > 0) {
        // Resize the vector to exactly the number of valid solutions found
        final_solutions.resize(host_count);
        
        // 5. COPY ONLY THE EXACT AMOUNT OF VALID DATA
        // We only copy host_count * sizeof(ValidPosition) bytes, ignoring the empty space!
        cudaMemcpy(final_solutions.data(), d_possible_positions, host_count * sizeof(ValidPosition), cudaMemcpyDeviceToHost);
    }

    // 6. CLEANUP GPU MEMORY
    cudaFree(d_possible_positions);
    cudaFree(d_count);
    cudaFree(d_object_position);

    // Return the perfectly sized dynamic vector back to your main program
    return final_solutions;
}


// Add this declaration to your calculation.cuh header:
// std::vector<std::vector<ValidPosition>> get_multiple_possible_sliders(const std::vector<Position>& host_object_positions);

std::vector<std::vector<ValidPosition>> DeltaCalculation::get_multiple_possible_sliders(const std::vector<Position>& host_object_positions) {
    
    int num_datasets = host_object_positions.size();
    if (num_datasets == 0) return {};

    int max_solutions = 600; 
    int threadsPerBlock = 256;
    int blocksPerGrid = (max_solutions + threadsPerBlock - 1) / threadsPerBlock;

    // 1. ALLOCATE VECTORS TO HOLD POINTERS AND STREAMS
    std::vector<cudaStream_t> streams(num_datasets);
    std::vector<ValidPosition*> d_possible_positions(num_datasets);
    std::vector<int*> d_counts(num_datasets);
    std::vector<Position*> d_object_positions(num_datasets);
    
    // We use pinned host memory (cudaMallocHost) for the counts. 
    // This is strictly required for asynchronous Device-to-Host copies to work efficiently.
    int* h_counts;
    CUDA_CHECK(cudaMallocHost(&h_counts, num_datasets * sizeof(int)));

    // 2. INITIALIZE STREAMS AND DEVICE MEMORY
    for (int i = 0; i < num_datasets; ++i) {
        CUDA_CHECK(cudaStreamCreate(&streams[i]));
        
        CUDA_CHECK(cudaMalloc(&d_possible_positions[i], max_solutions * sizeof(ValidPosition)));
        
        CUDA_CHECK(cudaMalloc(&d_counts[i], sizeof(int)));
        // Note: Using cudaMemsetAsync to zero out the counter within the specific stream
        CUDA_CHECK(cudaMemsetAsync(d_counts[i], 0, sizeof(int), streams[i])); 
        
        CUDA_CHECK(cudaMalloc(&d_object_positions[i], sizeof(Position)));
    }

    // 3. PHASE 1: ASYNC MEMCPY (IN), KERNEL EXECUTION, ASYNC MEMCPY (OUT - COUNT ONLY)
    for (int i = 0; i < num_datasets; ++i) {
        // Copy the specific position to the device asynchronously
        CUDA_CHECK(cudaMemcpyAsync(d_object_positions[i], &host_object_positions[i], sizeof(Position), cudaMemcpyHostToDevice, streams[i]));

        // Launch kernel in the specific stream (Notice the 4th launch parameter: streams[i])
        // Assumes delta_coefficients is already on device or accessible
        possible_slider_pos<<<blocksPerGrid, threadsPerBlock, 0, streams[i]>>>(
            delta_coefficients, 
            d_object_positions[i], 
            d_possible_positions[i], 
            d_counts[i], 
            max_solutions
        );

        // Copy the result count back to pinned host memory asynchronously
        CUDA_CHECK(cudaMemcpyAsync(&h_counts[i], d_counts[i], sizeof(int), cudaMemcpyDeviceToHost, streams[i]));
    }

    // 4. SYNCHRONIZE DEVICE TO READ COUNTS
    // We must wait for the counts to arrive before we know how much actual data to copy back.
    CUDA_CHECK(cudaDeviceSynchronize());

    // // --- ADD THIS DEBUG BLOCK ---
    // for (int i = 0; i < num_datasets; ++i) {
    //     std::cout << "Dataset " << i << " -> GPU found: " << h_counts[i] << " valid solutions." << std::endl;
    // }
    // //

    // Prepare the final 2D vector for our results
    std::vector<std::vector<ValidPosition>> final_all_solutions(num_datasets);

    // 5. PHASE 2: ASYNC MEMCPY (OUT - ACTUAL DATA)
    for (int i = 0; i < num_datasets; ++i) {
        if (h_counts[i] > 0) {
            final_all_solutions[i].resize(h_counts[i]);
            
            // Copy exactly the number of valid solutions found for this specific dataset
            CUDA_CHECK(cudaMemcpyAsync(final_all_solutions[i].data(), 
                                       d_possible_positions[i], 
                                       h_counts[i] * sizeof(ValidPosition), 
                                       cudaMemcpyDeviceToHost, 
                                       streams[i]));
        }
    }

    // 6. FINAL SYNCHRONIZATION AND CLEANUP
    // Wait for all the data copies to finish before destroying streams and memory
    CUDA_CHECK(cudaDeviceSynchronize());

    for (int i = 0; i < num_datasets; ++i) {
        CUDA_CHECK(cudaFree(d_possible_positions[i]));
        CUDA_CHECK(cudaFree(d_counts[i]));
        CUDA_CHECK(cudaFree(d_object_positions[i]));
        CUDA_CHECK(cudaStreamDestroy(streams[i]));
    }
    CUDA_CHECK(cudaFreeHost(h_counts)); // Free the pinned memory

    return final_all_solutions;
}