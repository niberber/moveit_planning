// FK (Forward Kinematics) 从零实现
// 目标: 给定 7 个关节角 q[0..6], 计算末端 TCP 在基座标系下的位姿
#include <iostream>
#include <cmath>
#include <array>
struct Pose{
    double x, y, z;
    double roll, pitch, yaw;
};
struct Transform{
    double m[4][4];
    Transform(){
        for(int i = 0; i < 4; i++)m[i][i] = 1.0;
    }
    Transform operator*(const Transform& other) const {
        Transform ans;
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                ans.m[i][j] = 0;
                for(int k = 0; k < 4; k++){
                    ans.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return ans;
    }
};
Transform rotate_x(double theta){
    Transform x;
    x.m[1][1] = cos(theta);
    x.m[1][2] = -sin(theta);
    x.m[2][1] = sin(theta);
    x.m[2][2] = cos(theta);
    return x;
}
Transform rotate_y(double theta){
    Transform y;
    y.m[0][0] = cos(theta);
    y.m[0][2] = sin(theta);
    y.m[2][0] = -sin(theta);
    y.m[2][2] = cos(theta);
    return y;
}
Transform rotate_z(double theta){
    Transform z;
    z.m[0][0] = cos(theta);
    z.m[1][1] = cos(theta);
    z.m[0][1] = -sin(theta);
    z.m[1][0] = sin(theta);
    return z;
}
Transform translate(double x, double y, double z){
    Transform t;
    t.m[0][3] = x;
    t.m[1][3] = y;
    t.m[2][3] = z;
    return t;
}
Pose fk_solve(const std::array<double, 7>& q){
    Transform T = Transform();
        // joint5: rpy=(0,0,0), xyz=(0.24933,-0.15,0.19895), axis=(0,0,-1)
    T = T * translate(0.24933, -0.15, 0.19895) * rotate_z(0) * rotate_y(0) * rotate_x(0) * rotate_z(-q[0]);
    std::cout << "after joint5: (" << T.m[0][3] << ", " << T.m[1][3] << ", " << T.m[2][3] << ")\n";

    // joint6: rpy=(1.5708,0,-1.5708), xyz=(0.04475,0.000117,0.083117), axis=(0,0,-1)
    T = T * translate(0.04475, 0.000117, 0.083117) * rotate_z(-1.5708) * rotate_y(0) * rotate_x(1.5708) * rotate_z(-q[1]);
    std::cout << "after joint6: (" << T.m[0][3] << ", " << T.m[1][3] << ", " << T.m[2][3] << ")\n";
    
    // joint7: rpy=(-3.1416,0,3.1416), xyz=(0.29299,0.064883,-0.006), axis=(0,0,-1)
    T = T * translate(0.29299, 0.064883, -0.006) * rotate_z(3.1416) * rotate_y(0) * rotate_x(-3.1416) * rotate_z(-q[2]);
    std::cout << "after joint7: (" << T.m[0][3] << ", " << T.m[1][3] << ", " << T.m[2][3] << ")\n";
    
    // joint8: rpy=(-3.1416,0,0), xyz=(0.15943,0.013538,-0.109), axis=(0,0,-1)
    T = T * translate(0.15943, 0.013538, -0.109) * rotate_z(0) * rotate_y(0) * rotate_x(-3.1416) * rotate_z(-q[3]);
    std::cout << "after joint8: (" << T.m[0][3] << ", " << T.m[1][3] << ", " << T.m[2][3] << ")\n";
    
    // joint9: rpy=(0.0847,1.5708,0), xyz=(0.11955,-0.01043,-0.0588), axis=(0,0,-1)
    T = T * translate(0.11955, -0.01043, -0.0588) * rotate_z(0) * rotate_y(1.5708) * rotate_x(0.0847) * rotate_z(-q[4]);
    std::cout << "after joint9: (" << T.m[0][3] << ", " << T.m[1][3] << ", " << T.m[2][3] << ")\n";
    
    // joint10: rpy=(-1.5708,-0.9582,1.5708), xyz=(-0.0575,0.056189,0.09), axis=(0,0,1)
    T = T * translate(-0.0575, 0.056189, 0.09) * rotate_z(1.5708) * rotate_y(-0.9582) * rotate_x(-1.5708) * rotate_z(q[5]);
    std::cout << "after joint10: (" << T.m[0][3] << ", " << T.m[1][3] << ", " << T.m[2][3] << ")\n";
    
    // joint11: rpy=(-0.1958,1.5708,0), xyz=(0.00647,0.00128,-0.05705), axis=(0,0,1)
    T = T * translate(0.00647, 0.00128, -0.05705) * rotate_z(0) * rotate_y(1.5708) * rotate_x(-0.1958) * rotate_z(q[6]);
    std::cout << "after joint11: (" << T.m[0][3] << ", " << T.m[1][3] << ", " << T.m[2][3] << ")\n";
    
    Pose result;
    result.x = T.m[0][3];
    result.y = T.m[1][3];
    result.z = T.m[2][3];

    const double R00 = T.m[0][0];
    const double R01 = T.m[0][1];
    const double R02 = T.m[0][2];
    const double R10 = T.m[1][0];
    const double R11 = T.m[1][1];
    const double R12 = T.m[1][2];
    const double R20 = T.m[2][0];
    const double R21 = T.m[2][1];
    const double R22 = T.m[2][2];
    double sy = sqrt(R00 * R00 + R10 * R10);
    if(sy > 1e-6){
        result.roll = atan2(R21, R22);
        result.pitch = atan2(-R20, sy);
        result.yaw = atan2(R10, R00);
    } else {
        result.roll = atan2(-R12, R11);
        result.pitch = atan2(R20, sy);
        result.yaw = 0;
    }
    return result;
}
int main(){
    // 我们将一步步在这里构建 FK 求解器
    std::array<double, 7> q = {0, 0, -0.3, -0.3, 0, 0, 0};
    Pose p = fk_solve(q);

    std::cout << "=== FK result ===" << "\n";
    std::cout << "q = [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[4] << ", " << q[5] << ", " << q[6] << "]"<< "\n";
    std::cout << "TCP = (" << p.x << ", " << p.y << ", " << p.z << ")" << "\n";
    std::cout << "roll = " << p.roll * 180.0 / M_PI << "°" << ", pitch = " << p.pitch * 180.0 / M_PI << "°" << ", yaw = " << p.yaw * 180.0 / M_PI << "\n";
    // q = {0};
    // p = fk_solve(q);
    // std::cout << "=== FK result0 ===" << "\n";
    // std::cout << "q = [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[4] << ", " << q[5] << ", " << q[6] << "]"<< "\n";
    // std::cout << "TCP = (" << p.x << ", " << p.y << ", " << p.z << ")" << "\n";
    // std::cout << "roll = " << p.roll << ", pitch = " << p.pitch << ", yaw = " << p.yaw << "\n";
    return 0;
}
