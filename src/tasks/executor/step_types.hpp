/**
 * @file step_types.hpp
 * @brief 任务步骤的类型定义、静态属性表与语法解析
 *
 * 任务层是配置驱动的：一条「序列」由若干「步骤」组成，每个步骤对应控制器的一个能力。
 * 本文件定义步骤的词汇表与解析规则，**不涉及执行**（执行见 task_executor.hpp）。
 *
 * 关键设计：@c requires_idel 等约束是**步骤类型的属性**（见 @ref step_spec），
 * 而不是 runner 中的硬编码名单。控制器新增「只能 idel 进出」的状态时，只需在这里
 * 给对应类型打上标记，不会出现「任务层忘记同步」的静默失效。
 *
 * @see TaskExecutor
 * @see tasks/config/tasks.yaml
 */

#pragma once

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

/**
 * @brief 单个步骤的解析结果（语法层）
 *
 * 仅承载 YAML 中直接写出的字段；@c to 到绝对关节角的解析、关节数校验等语义处理
 * 由 @ref TaskExecutor 在配置加载阶段完成。
 */
struct Step {
    std::string type;           ///< 步骤类型名，必须属于 @ref known_step_types
    double duration{0.0};       ///< 运动/等待时长（秒）
    double tolerance{0.0};      ///< 到位容差；0 表示由执行器取默认值
    std::vector<double> target; ///< joint_move：绝对关节角（由 to 解析或直接给出）
    std::vector<double> offset; ///< cart_move / admittance_move：相对末端位移（3 维）
    std::string action;         ///< tool：语义名（如 suck / release）
    std::string to;             ///< joint_move：引用的命名目标点
    int yaml_line{0};           ///< 在 YAML 中的行号（1 起），用于报错定位
};

/**
 * @brief 步骤类型的静态属性
 *
 * @see step_spec
 */
struct StepSpec {
    bool known{false};          ///< 是否为已知类型
    bool requires_idel{false};  ///< 执行前是否要求控制器处于 idel
    bool needs_duration{false}; ///< 是否必须提供正的 duration
};

/**
 * @brief 查询步骤类型的静态属性
 * @param type 步骤类型名
 * @return 属性；类型未知时 @c known 为 false
 */
StepSpec step_spec(const std::string& type);

/// @return 全部已知步骤类型名，用于错误提示
std::vector<std::string> known_step_types();

/**
 * @brief 从 YAML 节点解析一个步骤（仅语法层）
 *
 * 校验内容：必须是映射、必须含 @c type、类型必须已知、不得出现未知字段、
 * @c needs_duration 的类型必须给出正的 @c duration、数值字段类型正确。
 *
 * @param node  YAML 节点（序列中的一个元素）
 * @param step  输出：解析结果
 * @param error 输出：失败原因
 * @return true 解析成功
 */
bool parse_step(const YAML::Node& node, Step* step, std::string* error);
