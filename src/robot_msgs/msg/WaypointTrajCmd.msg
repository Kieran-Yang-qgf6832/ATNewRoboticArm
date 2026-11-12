# 多路点轨迹：控制器自动规划 C3（位置/速度/加速度/jerk 连续）曲线。
# 用户只给路点，时间由控制器按关节限位自动分配。
uint8 SPACE_JOINT=0
uint8 SPACE_CARTESIAN=1

uint8 space            # 0 = 关节空间（每点长度 = 关节数）；1 = 笛卡尔空间（每点 7 值 [x,y,z,qx,qy,qz,qw]）
float32 total_duration # >0 表示覆盖自动分配的总时长（秒）；<=0 表示自动
Vector[] waypoints     # 路点列表，最多 8 个用户路点
