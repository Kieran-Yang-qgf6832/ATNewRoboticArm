# 任务空间模式。默认 0，旧发布者不填该字段时行为与重构前完全一致。
uint8 MODE_FULL_POSE=0
uint8 MODE_POSITION_ONLY=1

uint8 mode
float32[] seconds
Vector[] position
Vector[] velocity
Vector[] acceleration