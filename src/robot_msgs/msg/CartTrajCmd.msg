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