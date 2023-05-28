#include <iostream>
#include <cmath>

// 计算目标框与相机光轴的夹角
double calculateAngle(double centerX, double centerY, double focalLength, double principalPointX, double principalPointY) {
    // 将目标框中心点的像素坐标转换为相机坐标系
    double cameraX = (centerX - principalPointX) / focalLength;
    double cameraY = (centerY - principalPointY) / focalLength;
    
    // 计算目标框中心点与相机光轴的夹角
    double angle = std::atan(std::sqrt(cameraX * cameraX + cameraY * cameraY));
    
    // 将弧度转换为角度
    angle = angle * 180.0 / M_PI;
    
    return angle;
}

int main() {
    // 相机参数
    double focalLength = 500.0; // 焦距
    double principalPointX = 320.0; // 光心 X 坐标
    double principalPointY = 240.0; // 光心 Y 坐标
    
    // 目标框中心点像素坐标
    double centerX = 400.0;
    double centerY = 300.0;
    
    // 计算目标框与相机光轴的夹角
    double angle = calculateAngle(centerX, centerY, focalLength, principalPointX, principalPointY);
    
    std::cout << "目标框与相机光轴的夹角: " << angle << "度" << std::endl;
    
    return 0;
}