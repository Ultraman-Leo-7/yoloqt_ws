// electric_field.h
#ifndef ELECTRIC_FIELD_H
#define ELECTRIC_FIELD_H

#include <cmath>
#include <vector>
#include <complex>
#include <Eigen/Dense>

struct Line {
    double x1, y1, z1, x2, y2, z2, r;
    std::complex<double> V;
};

struct Point {
    double x, y, z, xm, ym, zm;
    std::complex<double> V;
};

// 函数声明
void readConfig(const std::string& filename, std::vector<Line>& lines, std::vector<Point>& points, double& epsilon, double& Px, double& Py, double& Pz);
void readConfig_kaiguan(const std::string& filename, std::vector<Line>& lines, std::vector<Point>& points, double& epsilon, double& Px, double& Py, double& Pz);

std::vector<double> calculateElectricField(double Px, double Py, double Pz, const std::string& config_file);
std::vector<double> calculateElectricField_kaiguan(double Px, double Py, double Pz, const std::string& config_file);


#endif  // ELECTRIC_FIELD_H