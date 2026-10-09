/**
 * @file task_executor.hpp
 * @brief 配置驱动的任务执行器
 *
 * 替代原先「每个任务一个 C++ 类 + TaskFSMFactory + create_task」的结构：
 * 任务定义写在 @c tasks/config/tasks.yaml 里，本类负责加载、校验与执行。
 *
 * 一条**序列**由若干**步骤**组成，步骤通过 @ref TaskContext 驱动 arm_controller
 * 切换控制模式并下发指令。新增任务 = 编辑 YAML，不需要重新编译。
 *
 * @note 本类运行在普通 ROS 节点线程，允许阻塞式服务调用与内存分配，
 *       不受 arm_controller 实时控制路径的约束。
 *
 * @see TaskContext
 * @see step_types.hpp
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "step_types.hpp"
#include "task_context.hpp"

/**
 * @brief 配置驱动的任务执行器
 *
 * 生命周期：构造 → @ref load_config → 多次 @ref run_sequence。
 * 配置在 @ref load_config 中一次性解析并**全部校验**，任何非法配置都会在
 * 机械臂运动之前被拒绝。
 */
class TaskExecutor {
public:
    /**
     * @brief 构造函数
     * @param node runner 节点，用于日志与参数
     * @param ctx  任务上下文，持有与 arm_controller 的全部 ROS 接口
     * @note 不持有 @p ctx 的所有权，其生命周期必须覆盖本对象
     */
    TaskExecutor(const rclcpp::Node::SharedPtr& node, TaskContext* ctx);

    /**
     * @brief 加载并校验任务配置
     *
     * 解析 @c tool / @c points / @c sequences 三段，随后对每个序列的每个步骤做
     * 语义校验（目标点存在、关节数匹配、字段与类型匹配等）。任一步骤不合法即整体失败，
     * 保证不会出现「执行到一半才发现配置错误」。
     *
     * @param path  YAML 配置文件路径
     * @param error 输出：失败原因
     * @return true 加载并校验成功
     */
    bool load_config(const std::string& path, std::string* error);

    /// @return 配置中全部序列名，用于错误提示
    std::vector<std::string> sequence_names() const;

    /**
     * @brief 执行一条序列
     *
     * 失败时切回 @c idel 并返回 false，**不抛出异常、不结束进程**，
     * 调用方可以继续下发新的序列。
     *
     * @param name 序列名
     * @return true 序列全部步骤正常完成
     */
    bool run_sequence(const std::string& name);

private:
    /// 校验并解析单个步骤（把 @c to 解析成绝对关节角）
    bool validate_step(Step* step, std::string* error) const;

    /// 执行单个步骤
    bool run_step(const Step& step, std::string* error);

    /// 执行关节空间两点轨迹（joint_move）
    bool run_joint_move(const Step& step, std::string* error);

    /// 执行任务空间两点轨迹（cart_move / admittance_move）
    bool run_task_space_move(const Step& step, const std::string& state_name, std::string* error);

    /**
     * @brief 执行「切入某状态 -> 保持 duration -> 切回 idel」型步骤
     *
     * 适用于 teach（示教）与 measure（参数辨识）：控制器在该状态下自行完成全部工作，
     * 任务层只负责按时长守候，并在结束后把控制器交还给 idel。
     *
     * @param step       步骤（需含正 duration）
     * @param state_name 控制器状态名
     * @param error      输出：失败原因
     */
    bool run_timed_state(const Step& step, const std::string& state_name, std::string* error);

    /// 执行 tool 步骤：向 /tool_cmd 发布开关量
    bool run_tool(const Step& step, std::string* error);

    /// 自旋等待控制器进入指定状态；若控制器回退到 idel 则立即判失败
    bool wait_for_state(const std::string& expected, std::string* error);

    /// 自旋等待控制器回到 idel
    bool wait_for_idel(const std::string& context, std::string* error);

    /// 失败收尾：请求控制器切回 idel
    void fall_back_to_idel();

    /// 判断 tool 的 action 是否等价于「开」
    bool tool_action_is_on(const std::string& action) const;

    /// 判断 tool 的 action 是否等价于「关」
    bool tool_action_is_off(const std::string& action) const;

    rclcpp::Node::SharedPtr node_; ///< runner 节点
    TaskContext* ctx_{nullptr};    ///< 任务上下文（不持有所有权）

    /// 末端执行器开关量发布器（std_msgs/Bool，见 tool 步骤）
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tool_pub_;

    std::map<std::string, std::vector<double>> points_;      ///< 命名目标点
    std::map<std::string, std::vector<Step>> sequences_;     ///< 序列名 → 步骤列表
    std::string tool_name_{"tool"};                          ///< 当前末端执行器名（仅日志）
    std::string tool_on_{"on"};                              ///< 「开」的语义名
    std::string tool_off_{"off"};                            ///< 「关」的语义名
    double step_timeout_{30.0};                              ///< 单步骤超时（秒）
    double idel_hold_{0.3};                                  ///< 切到 idel 后的稳定时长（秒）
    double settle_duration_{0.3};                            ///< 切入运动状态后的稳定时长（秒）
    /**
     * @brief 切换状态后的稳定期（秒）
     *
     * 控制器把**实际**状态发布到 @c current_state 参数是由 500 ms wall timer 驱动的，
     * 因此 @c set_exp_state 之后立刻读该参数会读到切换前的旧值：例如请求 reset 时仍读到
     * @c idel，@ref wait_for_idel 会立刻返回假成功，步骤被跳过。
     *
     * 本值必须**大于控制器的状态刷新周期**（当前 500 ms）。若控制器侧的
     * @c state_timer_ 周期调整，这里必须同步调整，可通过 @c state_settle 参数覆盖。
     */
    double state_settle_{0.6};
};
