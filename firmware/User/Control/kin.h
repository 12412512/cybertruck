#ifndef KIN_H
#define KIN_H

typedef enum
{
    KIN_UNKNOWN = 0,
    KIN_MECANUM_X = 1
} KinLayout;

typedef struct
{
    float radius_m;
    float half_length_m;
    float half_width_m;
    KinLayout layout;
} KinGeometry;

typedef struct
{
    float vx_m_s;
    float vy_m_s;
    float wz_rad_s;
} KinBody;

/* X前、Y左、逆时针正；轮序FL/FR/RL/RR。理论轮正转为向车前滚动。
 * 不含驱动器地址/安装极性，不提供车辆默认几何，不发送任何命令。 */
int Kin_Inverse(KinGeometry geometry, KinBody body, float rpm[4]);
int Kin_Forward(KinGeometry geometry, const float rpm[4], KinBody *body);
/* 四轮同比缩放。非法返回-1且不改输入；合法返回(0,1]缩放比例。 */
float Kin_Limit(float rpm[4], float max_rpm);

#endif
