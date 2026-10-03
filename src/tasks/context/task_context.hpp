/**
 * @file task_context.hpp
 * @brief 任务层与 arm_controller 之间的 ROS 接口集合声明
 *
 * 所有任务共享同一个 TaskContext，用于：
 * - 通过 @c exp_state 参数服务请求控制器切换控制模式（idel/reset/cart_traj/...）；
 * - 订阅 @c /joint_states 获取当前关节角；
 * - 通过 TF 查询末端位姿（base_frame -> ee_frame），作为笛卡尔轨迹起点；
 * - 向 @c /arm_cart_traj、@c /arm_joint_traj 分别发布笛卡尔/关节轨迹指令，
 *   向 @c /arm_admittance 发布导纳期望轨迹指令；
 * - 记录任务链的运行失败信息（共享黑板）。
 *
 * @see TaskFSM
 *
 * @author lyz
 * @date 2026-09-30
 * @version 0.0.0
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_msgs/msg/admittance_cmd.hpp>
#include <robot_msgs/msg/cart_traj_cmd.hpp>
#include <robot_msgs/msg/joint_traj_cmd.hpp>

/**
 * @brief 任务层共享上下文
 *
 * @note 本类持有话题订阅、服务客户端与 TF 监听，生命周期应与 runner 节点一致。
 * @warning 不提供任何实时保证，仅可在普通 ROS 回调/主线程中使用。
 */
class TaskContext {
public:
    /**
     * @brief 上下文配置
     */
    struct Options {
        std::string controller_node{"arm_controller"}; ///< 控制器节点名，用于拼接参数服务名
        std::vector<std::string> joints{                ///< 关节顺序，需与 ros2_controller.yaml 一致
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        std::string cart_traj_topic{"arm_cart_traj"};                  ///< 笛卡尔轨迹话题
        std::string joint_traj_topic{"arm_joint_traj"};                ///< 关节轨迹话题
        std::string admittance_topic{"arm_admittance"};                ///< 导纳期望轨迹话题
        std::string base_frame{"base_link"};                           ///< 末端位姿参考系
        std::string ee_frame{"link6"};                                 ///< 末端执行器坐标系
    };

    /**
     * @brief 构造函数，创建订阅/发布/服务客户端与 TF 监听
     * @param node    runner 节点
     * @param options 上下文配置
     */
    TaskContext(const rclcpp::Node::SharedPtr& node, Options options);

    /// @return runner 节点
    rclcpp::Node::SharedPtr node() const { return node_; }

    /// @return 上下文配置
    const Options& options() const { return options_; }

    /**
     * @brief 等待 arm_controller 的参数服务可用
     * @param timeout_sec 超时时间（秒）
     * @return true 服务可用
     */
    bool wait_for_controller(double timeout_sec) const;

    /**
     * @brief 通过参数服务把 arm_controller 的 @c exp_state 置为指定状态
     * @param state_name  目标状态名，如 "idel" / "reset" / "cart_traj"
     * @param timeout_sec 服务调用超时（秒）
     * @return true 设置成功
     */
    bool set_exp_state(const std::string& state_name, double timeout_sec = 5.0);

    /**
     * @brief 自旋等待至少一帧可用的 /joint_states
     * @param timeout_sec 超时时间（秒）
     * @return true 已收到有效关节状态
     */
    bool wait_for_joint_states(double timeout_sec);

    /// @return 是否已收到过有效关节状态
    bool have_joint_states() const { return have_joint_states_; }

    /// @return 最近一帧与 @c Options::joints 顺序对齐的关节角（弧度）
    const std::vector<double>& joint_positions() const { return joint_positions_; }

    /**
     * @brief 查询末端执行器当前位姿
     * @param pose 输出：6 维任务空间向量 [x, y, z, rx, ry, rz]，旋转部分为旋转向量
     * @return true 查询成功；false TF 不可用（未收到 TF 或坐标系不存在）
     */
    bool end_effector_pose(std::vector<double>* pose) const;

    /**
     * @brief 发布笛卡尔轨迹指令
     * @param points  轨迹点，每个点为 6 维任务空间向量
     * @param seconds 与 @p points 一一对应的时刻（相对轨迹起点，秒）
     * @return true 发布成功；false 参数非法
     * @note 速度/加速度前馈留空，控制器按 0 处理。
     */
    bool publish_cart_traj(const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) const;

    /**
     * @brief 发布关节空间轨迹指令
     * @param points  轨迹点，每个点为关节角向量（长度等于关节数）
     * @param seconds 与 @p points 一一对应的时刻（相对轨迹起点，秒）
     * @return true 发布成功；false 参数非法
     * @note 速度/加速度前馈留空，控制器按 0 处理。
     */
    bool publish_joint_traj(const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) const;

    /**
     * @brief 发布导纳控制的期望轨迹指令
     * @param points  轨迹点，每个点为 6 维任务空间向量
     * @param seconds 与 @p points 一一对应的时刻（相对轨迹起点，秒）
     * @return true 发布成功；false 参数非法
     * @note 只填充 @c seconds 与 @c position；@c AdmittanceCmd.force 当前控制器未使用
     *       （外部力由控制器用实测力矩与模型力矩之差自行估计）。
     */
    bool publish_admittance(const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) const;

    /**
     * @brief 记录任务链不可恢复错误
     * @param message 错误描述
     */
    void fail(const std::string& message);

    /// @return 任务链是否已出现不可恢复错误
    bool failed() const { return failed_; }

    /// @return 错误描述
    const std::string& failure_message() const { return failure_message_; }

    /// 非阻塞处理一次回调（订阅与 TF）。
    void spin_some();

private:
    /**
     * @brief /joint_states 回调，缓存与 @c Options::joints 顺序对齐的关节角
     * @param msg 关节状态消息
     * @note 缺失任一关节或长度不足时忽略本帧。
     */
    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Node::SharedPtr node_; ///< runner 节点
    Options options_;              ///< 上下文配置

    std::vector<double> joint_positions_;  ///< 最近一帧对齐后的关节角
    bool have_joint_states_{false};        ///< 是否已收到有效关节状态

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_; ///< /joint_states 订阅
    rclcpp::Publisher<robot_msgs::msg::CartTrajCmd>::SharedPtr cart_traj_pub_;       ///< 笛卡尔轨迹发布
    rclcpp::Publisher<robot_msgs::msg::JointTrajCmd>::SharedPtr joint_traj_pub_;     ///< 关节轨迹发布
    rclcpp::Publisher<robot_msgs::msg::AdmittanceCmd>::SharedPtr admittance_pub_;    ///< 导纳期望轨迹发布
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_cli_;    ///< exp_state 参数服务客户端

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                 ///< TF 缓冲
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;    ///< TF 监听

    bool failed_{false};            ///< 任务链是否失败
    std::string failure_message_;   ///< 失败描述
};
