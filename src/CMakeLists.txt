add_library(multilat_core
    ${CMAKE_CURRENT_SOURCE_DIR}/true_range_multilateration_methods.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_helpers.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/core/algorithm_dispatch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/core/simulation_runner.cpp
)

target_include_directories(multilat_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(multilat_core PUBLIC
    Eigen
    EigenUnsupported
)

if(MULTILAT_BUILD_CLI)
    add_executable(multilat_cli ${CMAKE_CURRENT_SOURCE_DIR}/cli/main.cpp)
    target_link_libraries(multilat_cli PRIVATE multilat_core)
endif()

if(MULTILAT_BUILD_WEBAPP)
    if(EXISTS ${CMAKE_SOURCE_DIR}/external/raylib/CMakeLists.txt
       AND EXISTS ${CMAKE_SOURCE_DIR}/external/imgui/imgui.cpp
       AND EXISTS ${CMAKE_SOURCE_DIR}/external/rlimgui/CMakeLists.txt)
        set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
        add_subdirectory(${CMAKE_SOURCE_DIR}/external/raylib ${CMAKE_BINARY_DIR}/external/raylib)
        add_subdirectory(${CMAKE_SOURCE_DIR}/external/rlimgui ${CMAKE_BINARY_DIR}/external/rlimgui)

        add_executable(multilat_webapp
            ${CMAKE_CURRENT_SOURCE_DIR}/web/web_main.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/web/web_app.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/web/viewport2d.cpp
        )
        target_include_directories(multilat_webapp PRIVATE
            ${CMAKE_SOURCE_DIR}/external/imgui
            ${CMAKE_SOURCE_DIR}/external/rlimgui
        )
        target_link_libraries(multilat_webapp PRIVATE multilat_core raylib rlImGui)

        if(EMSCRIPTEN)
            set_target_properties(multilat_webapp PROPERTIES OUTPUT_NAME index SUFFIX ".html")
            target_link_options(multilat_webapp PRIVATE
                "-sUSE_GLFW=3"
                "-sASYNCIFY"
                "-sALLOW_MEMORY_GROWTH=1"
                "-sASSERTIONS=1"
            )
        endif()
    else()
        message(WARNING "Web dependencies missing. Initialize submodules under external/.")
    endif()
endif()
