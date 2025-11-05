project(libs)

# eigen-5.0.0
add_library(Eigen INTERFACE) # Header only

target_include_directories(Eigen INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/eigen-5.0.0
)

# eigen-5.0.0 unsupported
add_library(EigenUnsupported INTERFACE) # Header only

target_include_directories(Eigen INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/eigen-5.0.0/unsupported
)

target_link_libraries(EigenUnsupported INTERFACE 
    Eigen
)
