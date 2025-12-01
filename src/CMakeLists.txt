project(src)

# main
add_executable(main 
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/true_range_multilateration_methods.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_helpers.cpp
)

target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_link_libraries(main
    Eigen
    EigenUnsupported
)
