# 基座运动状态（world/惯性系），由仿真侧基座驱动插件按解析轨迹以控制周期发布。
#
# 只在「平面基座」模型下使用：x/y 平移 + 绕 world 的 z 轴 yaw，基座始终水平。
# 控制器（inertial_hold 状态）用它把 world 系保持目标变换到 base 系，并用速度/
# 加速度做时间戳外推与关节速度前馈。

std_msgs/Header header

# 基座位姿（world 系）：position = base 原点在世界坐标下的平移（m），
# orientation = base → world 的姿态（单位四元数）
geometry_msgs/Pose pose

# 基座速度（world 系）：linear = 线速度（m/s），angular = 角速度（rad/s）
geometry_msgs/Twist twist

# 基座加速度（world 系）：linear = 线加速度（m/s^2），angular = 角加速度（rad/s^2）
geometry_msgs/Accel accel

# 基座轨迹是否已启动（start_delay 之前为 false）
# false 时 pose 停在初始位姿、twist/accel 全零
bool motion_active
