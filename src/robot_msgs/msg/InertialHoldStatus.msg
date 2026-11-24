# 惯性空间保持（inertial_hold）状态的实时诊断，由控制器以固定频率发布。
#
# 误差定义：目标与实测末端位姿都在 world 系里比较，与控制器内部使用的坐标系一致。

std_msgs/Header header

# 保持目标（world 系，进入状态那一刻的实测末端位姿快照）
geometry_msgs/Pose target_pose
# 本周期实测末端位姿（world 系 = 基座位姿 × 关节角正解）
geometry_msgs/Pose actual_pose

# 位置误差范数（m）
float64 position_error
# 姿态误差角（rad），即相对旋转 log(R_target · R_actualᵀ) 的模长
float64 orientation_error

# 基座状态是否可用（话题超时后为 false，此时关节指令被冻结）
bool base_state_valid
# 本周期逆解是否收敛（false 时沿用上一拍指令）
bool ik_converged
