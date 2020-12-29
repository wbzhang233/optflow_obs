std_msgs/Header Header

## 位置移动参考控制量
uint8 MoveMode

uint8 XYZ_POS       = 0 #0b00,XYZ位置控制
uint8 XYZ_VEL       = 1 #0b01,XYZ速度控制
uint8 XY_VEL_Z_POS  = 2 #Ob02,XY速度控制，定高飞行
uint8 XY_POS_Z_VEL  = 3 #0b03,xy位置控制，z方向速度控制，调整高度
uint8 TRAJECTORY    = 4

## 移动所参考的导航坐标系
uint8 MoveFrame

uint8 ENU_FRAME     = 0 #东北天
uint8 NED_FRAME     = 1 #北东地
uint8 BODY_FRAME    = 2 #机体系控制

# 控制参考量
float32[3] position_ref         ## [m]
float32[3] velocity_ref         ## [m/s]

## 角度参考量：偏航角、偏航角速度、偏航角加速度
float32 yaw_ref                  ## [rad]
## float32 yaw_rate_ref             ## [rad/s] 
## float32 yaw_acceleration_ref     ## [rad/s] 