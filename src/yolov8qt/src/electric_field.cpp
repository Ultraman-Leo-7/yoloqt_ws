#define _USE_MATH_DEFINES
#include "../include/yolov8qt/electric_field.h"
#include <iostream>
#include <cstdlib> 
#include <fstream>
#include <sstream>
#include <vector>
#include <complex>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <functional>
#include <numeric>

using namespace std;
using namespace Eigen;


// 辅助函数：计算两点间距离
double distance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

// 辅助函数：计算点电荷电势
complex<double> pointChargePotential(double x, double y, double z, double xd, double yd, double zd, complex<double> q, double epsilon) {
    double d = distance(x, y, z, xd, yd, zd);
    double d_ = distance(x, y, z, xd, yd, -zd);
    return q / (4 * M_PI * epsilon) * (1 / d - 1 / d_);
}

// 辅助函数：计算点电荷电场
vector<complex<double>> pointChargeElectricField(double x, double y, double z, double xd, double yd, double zd, complex<double> q, double epsilon) {
    double dx_real = x - xd;
    double dy_real = y - yd;
    double dz_real = z - zd;
    double r_real = distance(x, y, z, xd, yd, zd);
    double dx_image = x - xd;
    double dy_image = y - yd;
    double dz_image = z - (-zd);
    double r_image = distance(x, y, z, xd, yd, -zd);

    vector<complex<double>> E(3, 0.0);
    if (r_real > 1e-10) {
        E[0] = (q / (4 * M_PI * epsilon)) * dx_real / (r_real * r_real * r_real);
        E[1] = (q / (4 * M_PI * epsilon)) * dy_real / (r_real * r_real * r_real);
        E[2] = (q / (4 * M_PI * epsilon)) * dz_real / (r_real * r_real * r_real);
    }
    if (r_image > 1e-10) {
        E[0] -= (q / (4 * M_PI * epsilon)) * dx_image / (r_image * r_image * r_image);
        E[1] -= (q / (4 * M_PI * epsilon)) * dy_image / (r_image * r_image * r_image);
        E[2] -= (q / (4 * M_PI * epsilon)) * dz_image / (r_image * r_image * r_image);
    }
    return E;
}

// 辅助函数：数值积分（自适应辛普森法）
double integrate(function<double(double)> f, double a, double b, double tol);

// 辅助函数：计算直线电荷电势
complex<double> lineChargePotential(double x, double y, double z, double x1, double y1, double z1, double x2, double y2, double z2, complex<double> lambda, double epsilon) {
    auto integrand = [&](double t) {
        double xt = x1 + t * (x2 - x1);
        double yt = y1 + t * (y2 - y1);
        double zt = z1 + t * (z2 - z1);
        double r_real = distance(x, y, z, xt, yt, zt);
        double r_image = distance(x, y, z, xt, yt, -zt);
        return (1.0 / r_real - 1.0 / r_image);
    };

    double result = integrate(integrand, 0.0, 1.0, 1e-6);
    return lambda * result / (4 * M_PI * epsilon);
}

// 辅助函数：计算直线电荷电场
vector<complex<double>> lineChargeElectricField(double x, double y, double z, double x1, double y1, double z1, double x2, double y2, double z2, complex<double> lambda, double epsilon) {
    vector<complex<double>> E(3, 0.0);
    for (int i = 0; i < 3; ++i) {
        auto integrand = [&](double t) {
            double xt = x1 + t * (x2 - x1);
            double yt = y1 + t * (y2 - y1);
            double zt = z1 + t * (z2 - z1);
            double dx_real = x - xt;
            double dy_real = y - yt;
            double dz_real = z - zt;
            double r_real = distance(x, y, z, xt, yt, zt);
            double dx_image = x - xt;
            double dy_image = y - yt;
            double dz_image = z - (-zt);
            double r_image = distance(x, y, z, xt, yt, -zt);

            if (i == 0) {
                return (dx_real / (r_real * r_real * r_real) - dx_image / (r_image * r_image * r_image));
            } else if (i == 1) {
                return (dy_real / (r_real * r_real * r_real) - dy_image / (r_image * r_image * r_image));
            } else {
                return (dz_real / (r_real * r_real * r_real) - dz_image / (r_image * r_image * r_image));
            }
        };
        E[i] = lambda * integrate(integrand, 0.0, 1.0, 1e-6) / (4 * M_PI * epsilon);
    }
    return E;
}

// 数值积分函数实现
double integrate(function<double(double)> f, double a, double b, double tol) {
    function<double(double, double)> simpson = [&](double a, double b) {
        double c = (a + b) / 2.0;
        return (b - a) / 6.0 * (f(a) + 4.0 * f(c) + f(b));
    };

    function<pair<double, double>(double, double, double)> integrateRec = [&](double a, double b, double tol) {
        double c = (a + b) / 2.0;
        double S = simpson(a, b);
        double SL = simpson(a, c);
        double SR = simpson(c, b);
        if (abs(S - SL - SR) <= 15.0 * tol) {
            return make_pair(SL + SR, abs(S - SL - SR));
        } else {
            auto leftResult = integrateRec(a, c, tol / 2.0);
            auto rightResult = integrateRec(c, b, tol / 2.0);
            return make_pair(leftResult.first + rightResult.first, leftResult.second + rightResult.second);
        }
    };

    auto result = integrateRec(a, b, tol);
    return result.first;
}

// 从文件读取配置
void readConfig(const string& filename, vector<Line>& lines, vector<Point>& points, double& epsilon, double& Px, double& Py, double& Pz, double voltage_kv) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        exit(1);
    }

    int n_line, n_point;
    file >> n_line >> n_point;

    lines.resize(n_line);
    points.resize(n_point);
    double phase_angle;

    // 将电压从kV转换为V
    double voltage_v = voltage_kv * 1000.0;

    for (int i = 0; i < n_line; ++i) {
        file >> lines[i].x1 >> lines[i].y1 >> lines[i].z1;
        file >> lines[i].x2 >> lines[i].y2 >> lines[i].z2;
        file >> lines[i].r;
        file >> phase_angle;

        // 验证相位角是否有效
        if (file.fail()) {
            cerr << "错误：无效的相位角度值在第 " << (i + 1) << " 条导线配置中" << endl;
            exit(1);
        }

        // 将角度转换为弧度
        double phase_rad = phase_angle * M_PI / 180.0;

        // 计算复数电压：实部 = V * cos(θ), 虚部 = V * sin(θ)
        double realPart = voltage_v * cos(phase_rad);
        double imagPart = voltage_v * sin(phase_rad);
        lines[i].V = complex<double>(realPart, imagPart);
    }

    for (int i = 0; i < n_point; ++i) {
        file >> points[i].x >> points[i].y >> points[i].z;
        file >> points[i].xm >> points[i].ym >> points[i].zm;
        file >> phase_angle;

        // 验证相位角是否有效
        if (file.fail()) {
            cerr << "错误：无效的相位角度值在第 " << (i + 1) << " 个点电荷配置中" << endl;
            exit(1);
        }

        // 将角度转换为弧度
        double phase_rad = phase_angle * M_PI / 180.0;

        // 计算复数电压
        double realPart = voltage_v * cos(phase_rad);
        double imagPart = voltage_v * sin(phase_rad);
        points[i].V = complex<double>(realPart, imagPart);
    }

    file >> epsilon >> Px >> Py >> Pz;
}

void readConfig_kaiguan(const string& filename, vector<Line>& lines, vector<Point>& points, double& epsilon, double& Px, double& Py, double& Pz, double voltage_kv) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        exit(1);
    }

    int n_line, n_point;
    file >> n_line >> n_point;

    lines.resize(n_line);
    points.resize(n_point);
    double phase_angle;

    // 将电压从kV转换为V
    double voltage_v = voltage_kv * 1000.0;

    for (int i = 0; i < n_line; ++i) {
        file >> lines[i].x1 >> lines[i].y1 >> lines[i].z1;
        file >> lines[i].x2 >> lines[i].y2 >> lines[i].z2;
        file >> lines[i].r;
        file >> phase_angle;

        // 验证相位角是否有效
        if (file.fail()) {
            cerr << "错误：无效的相位角度值在第 " << (i + 1) << " 条导线配置中" << endl;
            exit(1);
        }

        // 将角度转换为弧度
        double phase_rad = phase_angle * M_PI / 180.0;

        // 计算复数电压：实部 = V * cos(θ), 虚部 = V * sin(θ)
        double realPart = voltage_v * cos(phase_rad);
        double imagPart = voltage_v * sin(phase_rad);
        lines[i].V = complex<double>(realPart, imagPart);
        // lines[i].z1 = Pz + 1;  // 假设开关长度=2m，开关端点1的z坐标为Pz+1
        // lines[i].z2 = Pz - 1;  // 开关端点2的z坐标为Pz-1
    }

    for (int i = 0; i < n_point; ++i) {
        file >> points[i].x >> points[i].y >> points[i].z;
        file >> points[i].xm >> points[i].ym >> points[i].zm;
        file >> phase_angle;

        // 验证相位角是否有效
        if (file.fail()) {
            cerr << "错误：无效的相位角度值在第 " << (i + 1) << " 个点电荷配置中" << endl;
            exit(1);
        }

        // 将角度转换为弧度
        double phase_rad = phase_angle * M_PI / 180.0;

        // 计算复数电压
        double realPart = voltage_v * cos(phase_rad);
        double imagPart = voltage_v * sin(phase_rad);
        points[i].V = complex<double>(realPart, imagPart);
    }

    file >> epsilon >> Px >> Py >> Pz;
}

// 封装的函数：计算电场
vector<double> calculateElectricField(double Px, double Py, double Pz, const string& config_file, double voltage_kv) {
    vector<Line> lines;
    vector<Point> points;
    double epsilon;

    // 读取配置文件
    readConfig(config_file, lines, points, epsilon, Px, Py, Pz, voltage_kv);

    // 构建电位系数矩阵并求解电荷密度
    MatrixXcd A(lines.size() + points.size(), lines.size() + points.size());
    VectorXcd V_target(lines.size() + points.size());

    for (int i = 0; i < lines.size(); ++i) {
        double xm = (lines[i].x1 + lines[i].x2) / 2.0;
        double ym = (lines[i].y1 + lines[i].y2) / 2.0;
        double zm = (lines[i].z1 + lines[i].z2) / 2.0;
        double dx = lines[i].x2 - lines[i].x1;
        double dy = lines[i].y2 - lines[i].y1;
        double dz = lines[i].z2 - lines[i].z1;
        Vector3d tangent(dx, dy, dz);
        tangent.normalize();
        Vector3d radial = tangent.cross(Vector3d(0, 0, 1));
        radial.normalize();
        Vector3d match_point = Vector3d(xm, ym, zm) + radial * lines[i].r;
        V_target(i) = lines[i].V;
        for (int j = 0; j < lines.size() + points.size(); ++j) {
            if (j < lines.size()) {
                A(i, j) = lineChargePotential(match_point[0], match_point[1], match_point[2],
                                              lines[j].x1, lines[j].y1, lines[j].z1,
                                              lines[j].x2, lines[j].y2, lines[j].z2,
                                              1.0, epsilon);
            } else {
                A(i, j) = pointChargePotential(match_point[0], match_point[1], match_point[2],
                                               points[j - lines.size()].xm, points[j - lines.size()].ym, points[j - lines.size()].zm,
                                               1.0, epsilon);
            }
        }
    }

    for (int i = 0; i < points.size(); ++i) {
        int idx = lines.size() + i;
        V_target(idx) = points[i].V;
        for (int j = 0; j < lines.size() + points.size(); ++j) {
            if (j < lines.size()) {
                A(idx, j) = lineChargePotential(points[i].xm, points[i].ym, points[i].zm,
                                                lines[j].x1, lines[j].y1, lines[j].z1,
                                                lines[j].x2, lines[j].y2, lines[j].z2,
                                                1.0, epsilon);
            } else {
                A(idx, j) = pointChargePotential(points[i].xm, points[i].ym, points[i].zm,
                                                 points[j - lines.size()].xm, points[j - lines.size()].ym, points[j - lines.size()].zm,
                                                 1.0, epsilon);
            }
        }
    }

    VectorXcd lambda = A.fullPivLu().solve(V_target);

    // 计算测点电场
    vector<complex<double>> E(3, 0.0);
    for (int j = 0; j < lines.size(); ++j) {
        auto E_line = lineChargeElectricField(Px, Py, Pz,
                                              lines[j].x1, lines[j].y1, lines[j].z1,
                                              lines[j].x2, lines[j].y2, lines[j].z2,
                                              lambda[j], epsilon);
        E[0] += E_line[0];
        E[1] += E_line[1];
        E[2] += E_line[2];
    }

    for (int j = 0; j < points.size(); ++j) {
        auto E_point = pointChargeElectricField(Px, Py, Pz,
                                                points[j].x, points[j].y, points[j].z,
                                                lambda[lines.size() + j], epsilon);
        E[0] += E_point[0];
        E[1] += E_point[1];
        E[2] += E_point[2];
    }

    // 计算模长
    double Ex = std::abs(E[0]);
    double Ey = std::abs(E[1]);
    double Ez = std::abs(E[2]);
    double normE = sqrt(Ex * Ex + Ey * Ey + Ez * Ez);

    // 返回结果
    return {Ex, Ey, Ez, normE};
}




vector<double> calculateElectricField_kaiguan(double Px, double Py, double Pz, const string& config_file, double voltage_kv) {
    vector<Line> lines;
    vector<Point> points;
    double epsilon;

    // 读取配置文件
    readConfig_kaiguan(config_file, lines, points, epsilon, Px, Py, Pz, voltage_kv);
    std::cout << "kaiguan_config 读取完成" << std::endl; 
    // 构建电位系数矩阵并求解电荷密度
    MatrixXcd A(lines.size() + points.size(), lines.size() + points.size());
    VectorXcd V_target(lines.size() + points.size());

    for (int i = 0; i < lines.size(); ++i) {
        double xm = (lines[i].x1 + lines[i].x2) / 2.0;
        double ym = (lines[i].y1 + lines[i].y2) / 2.0;
        double zm = (lines[i].z1 + lines[i].z2) / 2.0;
        double dx = lines[i].x2 - lines[i].x1;
        double dy = lines[i].y2 - lines[i].y1;
        double dz = lines[i].z2 - lines[i].z1;
        Vector3d tangent(dx, dy, dz);
        tangent.normalize();
        Vector3d radial = tangent.cross(Vector3d(0, 0, 1));
        radial.normalize();
        Vector3d match_point = Vector3d(xm, ym, zm) + radial * lines[i].r;
        V_target(i) = lines[i].V;
        for (int j = 0; j < lines.size() + points.size(); ++j) {
            if (j < lines.size()) {
                A(i, j) = lineChargePotential(match_point[0], match_point[1], match_point[2],
                                              lines[j].x1, lines[j].y1, lines[j].z1,
                                              lines[j].x2, lines[j].y2, lines[j].z2,
                                              1.0, epsilon);
            } else {
                A(i, j) = pointChargePotential(match_point[0], match_point[1], match_point[2],
                                               points[j - lines.size()].xm, points[j - lines.size()].ym, points[j - lines.size()].zm,
                                               1.0, epsilon);
            }
        }
    }
    std::cout << "线电势计算完成" << std::endl;
    for (int i = 0; i < points.size(); ++i) {
        int idx = lines.size() + i;
        V_target(idx) = points[i].V;
        for (int j = 0; j < lines.size() + points.size(); ++j) {
            if (j < lines.size()) {
                A(idx, j) = lineChargePotential(points[i].xm, points[i].ym, points[i].zm,
                                                lines[j].x1, lines[j].y1, lines[j].z1,
                                                lines[j].x2, lines[j].y2, lines[j].z2,
                                                1.0, epsilon);
            } else {
                A(idx, j) = pointChargePotential(points[i].xm, points[i].ym, points[i].zm,
                                                 points[j - lines.size()].xm, points[j - lines.size()].ym, points[j - lines.size()].zm,
                                                 1.0, epsilon);
            }
        }
    }
    std::cout << "点电势计算完成" << std::endl;
    VectorXcd lambda = A.fullPivLu().solve(V_target);

    // 计算测点电场
    vector<complex<double>> E(3, 0.0);
    for (int j = 0; j < lines.size(); ++j) {
        auto E_line = lineChargeElectricField(Px, Py, Pz,
                                              lines[j].x1, lines[j].y1, lines[j].z1,
                                              lines[j].x2, lines[j].y2, lines[j].z2,
                                              lambda[j], epsilon);
        E[0] += E_line[0];
        E[1] += E_line[1];
        E[2] += E_line[2];
    }
    std::cout << "线电场计算完成" << std::endl;
    for (int j = 0; j < points.size(); ++j) {
        auto E_point = pointChargeElectricField(Px, Py, Pz,
                                                points[j].x, points[j].y, points[j].z,
                                                lambda[lines.size() + j], epsilon);
        E[0] += E_point[0];
        E[1] += E_point[1];
        E[2] += E_point[2];
    }
    std::cout << "点电场计算完成" << std::endl;
    // 计算模长
    double Ex = std::abs(E[0]);
    double Ey = std::abs(E[1]);
    double Ez = std::abs(E[2]);
    double normE = sqrt(Ex * Ex + Ey * Ey + Ez * Ez);

    // 返回结果
    return {Ex, Ey, Ez, normE};
}