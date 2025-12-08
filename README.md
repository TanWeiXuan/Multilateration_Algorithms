# Multilateration Algorithms
A collection of the implementation of various [True-range Multilateration](https://en.wikipedia.org/wiki/True-range_multilateration) algoritms in C++ using Eigen.


**True-range multilateration** is a method to determine the position of a point in space by using multiple ranges between the point and known locations (anchors).

## Methods

This repository implements the following multilateration methods:

### 1. ordinaryLeastSquaresWikipedia
Uses ordinary least squares to solve the linearised multilateration problem. This method is based on the general multilateration approach described in the [Wikipedia article](https://en.wikipedia.org/wiki/True-range_multilateration#General_Multilateration). 

**Note:** The solver fails if anchors are coplanar.

**Signature:**
```cpp
Eigen::Vector3d ordinaryLeastSquaresWikipedia(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);
```

### 2. ordinaryLeastSquaresWikipedia2
Uses ordinary least squares to solve the linearised multilateration problem with Eigen's BDCSVD (Bidiagonal Divide and Conquer Singular Value Decomposition). This method is based on the same approach as `ordinaryLeastSquaresWikipedia` but uses a more numerically stable solver.

**Note:** This method works even if anchors are coplanar, making it more robust than the first method.

**Signature:**
```cpp
Eigen::Vector3d ordinaryLeastSquaresWikipedia2(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);
```

### 3. nonLinearLeastSquaresEigenLevenbergMarquardt
Uses Eigen's Levenberg-Marquardt implementation to solve the non-linear least squares problem. This method provides an iterative optimization approach that refines the position estimate by minimizing the residual errors between measured and modeled ranges.

**Signature:**
```cpp
Eigen::Vector3d nonLinearLeastSquaresEigenLevenbergMarquardt(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);
```

### 4. robustNonLinearLeastSquaresEigenLevenbergMarquardt
A robust version of the non-linear least squares method that uses Eigen's Levenberg-Marquardt implementation with robust loss functions using an iteratively reweighted least squares (IRLS) approach. This method is more resilient to outliers in the range measurements by applying Cauchy weighting to reduce the influence of erroneous data points.

**Signature:**
```cpp
Eigen::Vector3d robustNonLinearLeastSquaresEigenLevenbergMarquardt(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges,
    const double rangeStdDev,
    const double robustLossParam
);
```

### 5. linearLeastSquaresI_YueWang
A linear least squares method (LLS-I) based on the paper "Linear least squares localization in sensor networks" by Yue Wang (2015). This method solves a linearized version of the multilateration problem by formulating it as an overdetermined system.

**Signature:**
```cpp
Eigen::Vector3d linearLeastSquaresI_YueWang(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);
```


### 6. linearLeastSquaresII_2_YueWang
A linear least squares method (LLS-II-2) also from "Linear least squares localization in sensor networks" by Yue Wang (2015). This method solves a linearized version of the multilateration problem by formulating it as an overdetermined system. The system is linearized by subtracting the nonlinear expression for the shortest range measurement to its corresponding anchor.

**Signature:**
```cpp
Eigen::Vector3d linearLeastSquaresII_2_YueWang(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges
);
```

### 7. twoStepWeightedLinearLeastSquaresI_YueWang
A two-step weighted linear least squares method (TS-WLLS-I) from "Linear least squares localization in sensor networks" by Yue Wang (2015), originally developed and proposed in "A Simple and Efficient Estimator for Hyperbolic Location" by Y. T. Chan and K. C. Ho (1994). This method performs position estimation in two steps: the first step uses weighted linear least squares to obtain an initial estimate, and the second step refines the solution by utilizing the constraint of the dummy variable (R² = x² + y² + z²). The weighting takes into account the standard deviations of range measurements to improve estimation accuracy.

**Signature:**
```cpp
Eigen::Vector3d twoStepWeightedLinearLeastSquaresI_YueWang(
    const std::vector<Eigen::Vector3d>& anchorPositions,
    const std::vector<double>& ranges,
    const std::vector<double>& rangeStdDevs
);
```



