# 任务空间模式。默认 0，旧发布者不填该字段时行为与重构前完全一致。
# 2 = 位置 3 维 + 末端 link6 的 z 轴方向 2 维；绕 z 轴的滚转不约束。
uint8 MODE_FULL_POSE=0
uint8 MODE_POSITION_ONLY=1
uint8 MODE_POSITION_AND_TOOL_AXIS=2

uint8 mode
float32[] seconds
Vector[] position
Vector[] velocity
Vector[] acceleration

# 位置**通过**容差（米）。0 = 沿用控制器 pos_move_tolerance（严格，与收敛判据相同）。
# 只对位置约束模式（mode=1/2）有效：迭代收敛判据仍是 pos_move_tolerance，
# 跑完仍未收敛时，停滞点（最近可达位姿）的位置误差在该值以内即判为成功并执行；
# 超过该值则整条轨迹在下发前被拒绝。box_move 用它做「允许停在离目标多远」。
float32 position_pass_tolerance
