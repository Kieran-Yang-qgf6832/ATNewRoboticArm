# 视觉目标面：与 TF 帧同帧发布的有效性与质量信息。
# header.stamp 与对应 TF 变换的 stamp 完全一致；header.frame_id 是父帧（相机光学系）。
# 下游必须先看 valid 与 stamp 新鲜度，再决定能不能把该目标用于运动。
std_msgs/Header header
string child_frame_id
bool valid

uint8 MODE_NONE=0
uint8 MODE_SINGLE=1
uint8 MODE_DUAL=2
uint8 MODE_TRIPLE=3
uint8 mode

# 与 TF 一致的面上坐标系：position 为面心（相机系，米）；
# orientation 的 +Z = -normal，即指向箱内、远离相机，供末端工具对齐。
geometry_msgs/Pose pose
# 面外单位法线，朝相机；接近方向为 -normal
geometry_msgs/Vector3 normal
# 面内尺寸 (width, height, 0)，米；单面模式为 PnP 假定边长
geometry_msgs/Vector3 size

# 质量指标：单面用重投影误差(px)，分面用平面内点 RMS(m)，另一项置 0
float32 reprojection_error_px
float32 plane_rms_m
float32 incidence_deg

uint8 POS_PNP=0
uint8 POS_DEPTH_CORRECTED=1
uint8 POS_DEPTH_CLOUD=2
uint8 position_source
# 新候选面连续占优的帧数，0 表示当前锁定稳定
uint8 hysteresis_count
string[] issues

# 体心只作诊断，不发布独立 TF
bool has_box
geometry_msgs/Point box_center
geometry_msgs/Vector3 box_size
