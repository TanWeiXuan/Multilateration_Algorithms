#include <iostream>
#include <format>

#include <Eigen/Dense>

int main(int argc, char const *argv[])
{
    std::cout << std::format("Hello world!\n");

    Eigen::Vector3d vec(1.0, 2.0, 3.0);
    std::cout << vec.transpose() << "\n";

    return 0;
}
