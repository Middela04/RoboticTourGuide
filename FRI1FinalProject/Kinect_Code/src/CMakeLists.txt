
# List all the source files
set(SOURCES
    main.cpp
    HandRaiseDetector.cpp
)

# List all header files
set(HEADERS
    HandRaiseDetector.hpp
    Client.hpp
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${K4A_INCLUDE_DIRS} ${K4ABT_INCLUDE_DIRS})

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE k4a k4abt)

# Optional: Link CUDA libraries if your project uses GPU features
# target_link_libraries(${PROJECT_NAME} PRIVATE ${CUDA_LIBRARIES})  # Uncomment if using CUDA
# target_include_directories(${PROJECT_NAME} PRIVATE ${CUDA_INCLUDE_DIRS})  # Uncomment if using CUDA
