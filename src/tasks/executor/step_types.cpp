#include "step_types.hpp"

#include <algorithm>
#include <exception>
#include <string>
#include <vector>

namespace {

/**
 * @brief 步骤词汇表条目
 *
 * 词汇表是**封闭集合**：因为本项目不新增控制算法，步骤类型基本不会增长；
 * 新增任务 = 组合已有类型 + 改目标点，不需要写 C++。
 */
struct Entry {
    const char* type;
    bool requires_idel;
    bool needs_duration;
    const char* exp_state; ///< 控制器状态名；nullptr 表示本步骤不切换控制器状态
};

/// 全部步骤类型。requires_idel 对应控制器侧「只能从 idel 进入、只能切回 idel」的约束。
const Entry kStepTable[] = {
    {"wait", false, true, nullptr},            // 不切换控制器状态，纯等待
    {"idel", false, false, "idel"},            // 切回 idel 并锁定当前位置
    {"reset", true, false, "reset"},           // 复位；ResetState::enter 要求 last_state == idel
    {"joint_move", false, true, "joint_traj"}, // 关节空间轨迹
    {"joint_console", true, false, "joint_traj"}, // 终端交互式关节运动；用户接管，要求 idel 进出
    {"cart_move", false, true, "cart_traj"},   // 笛卡尔空间轨迹
    {"admittance_move", false, true, "admittance"}, // 导纳控制下的期望轨迹
    {"tool", false, false, nullptr},           // 末端执行器开关量
    {"teach", true, true, "teach_pendant"},    // 示教；TeachPendantState 要求 idel 进出
    {"measure", true, true, "measure"},        // 参数辨识；ParamterMeasureState 要求 idel 进出
};

constexpr std::size_t kStepTableSize = sizeof(kStepTable) / sizeof(kStepTable[0]);

const Entry* find_entry(const std::string& type) {
    for (std::size_t i = 0; i < kStepTableSize; ++i) {
        if (type == kStepTable[i].type) {
            return &kStepTable[i];
        }
    }
    return nullptr;
}

/// 字段白名单：出现白名单外的键直接拒绝，避免拼错字段被静默忽略。
bool is_allowed_field(const std::string& key) {
    static const char* kAllowed[] = {"type", "duration", "tolerance", "to", "target", "offset", "action"};
    for (const char* allowed : kAllowed) {
        if (key == allowed) {
            return true;
        }
    }
    return false;
}

}  // namespace

StepSpec step_spec(const std::string& type) {
    StepSpec spec;
    const Entry* entry = find_entry(type);
    if (entry != nullptr) {
        spec.known          = true;
        spec.requires_idel  = entry->requires_idel;
        spec.needs_duration = entry->needs_duration;
        spec.exp_state      = entry->exp_state;
    }
    return spec;
}

std::vector<std::string> known_step_types() {
    std::vector<std::string> names;
    names.reserve(kStepTableSize);
    for (std::size_t i = 0; i < kStepTableSize; ++i) {
        names.emplace_back(kStepTable[i].type);
    }
    return names;
}

bool parse_step(const YAML::Node& node, Step* step, std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (step == nullptr) {
        return fail("internal error: null step");
    }
    if (!node.IsMap()) {
        return fail("each step must be a mapping, e.g. {type: idel}");
    }

    try {
        *step           = Step{};
        step->yaml_line = node.Mark().line + 1; // yaml-cpp 行号从 0 起，转成 1 起便于对日志

        if (!node["type"] || !node["type"].IsScalar()) {
            return fail("step is missing the required scalar field 'type'");
        }
        step->type = node["type"].as<std::string>();

        for (const auto& field : node) {
            if (!field.first.IsScalar()) {
                return fail("step fields must be scalar keys");
            }
            const std::string key = field.first.as<std::string>();
            if (!is_allowed_field(key)) {
                return fail("unknown field '" + key + "' in step of type '" + step->type + "'");
            }
        }

        const StepSpec spec = step_spec(step->type);
        if (!spec.known) {
            std::string known;
            for (const auto& name : known_step_types()) {
                known += (known.empty() ? "" : ", ") + name;
            }
            return fail("unknown step type '" + step->type + "'; known types: " + known);
        }

        if (node["duration"]) {
            if (!node["duration"].IsScalar()) {
                return fail("'duration' must be a number");
            }
            step->duration = node["duration"].as<double>();
        }
        if (spec.needs_duration && !(step->duration > 0.0)) {
            return fail("step type '" + step->type + "' requires a positive 'duration'");
        }

        if (node["tolerance"]) {
            if (!node["tolerance"].IsScalar()) {
                return fail("'tolerance' must be a number");
            }
            step->tolerance = node["tolerance"].as<double>();
            if (step->tolerance < 0.0) {
                return fail("'tolerance' must be non-negative");
            }
        }

        if (node["to"]) {
            if (!node["to"].IsScalar()) {
                return fail("'to' must be a point name");
            }
            step->to = node["to"].as<std::string>();
        }
        if (node["target"]) {
            if (!node["target"].IsSequence()) {
                return fail("'target' must be a list of joint angles");
            }
            step->target = node["target"].as<std::vector<double>>();
        }
        if (node["offset"]) {
            if (!node["offset"].IsSequence()) {
                return fail("'offset' must be a list");
            }
            step->offset = node["offset"].as<std::vector<double>>();
        }
        if (node["action"]) {
            if (!node["action"].IsScalar()) {
                return fail("'action' must be a string");
            }
            step->action = node["action"].as<std::string>();
        }
    } catch (const std::exception& exception) {
        return fail(std::string("malformed step: ") + exception.what());
    }

    return true;
}
