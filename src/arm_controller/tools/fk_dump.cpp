/**
 * @file fk_dump.cpp
 * @brief 打印指定关节角下的末端位姿，用于与 TF 逐项对比（排查 docs/bugs.md O1）
 *
 * 排查动机：任务层经 TF 读到的末端位姿与控制器 Pinocchio 正运动学不一致约 1e-3，
 * 使笛卡尔逆解的目标落在可达集之外、误差降不到零。要区分两种可能的原因——
 *   1) 姿态表示的转换差异（四元数 -> 旋转向量 vs Eigen::AngleAxisd）；
 *   2) 真实位姿差异（坐标系 / URDF 不一致）；
 * 必须同时看到四元数、旋转矩阵与旋转向量，因此单独做这个工具。
 *
 * 用法：
 * @code{.sh}
 * fk_dump <urdf> <frame> <q1> ... <qN>
 * @endcode
 *   frame = 末端坐标系名（4 轴模型 link4，6 轴模型 link6）；关节角个数必须等于模型关节数。
 *
 * @note 只用于离线排查，不参与控制器运行。
 */

#include "default4dof_task.hpp"
#include "default6dof_task.hpp"
#include "model_from_urdf.hpp"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: fk_dump <urdf> <frame> <q1> ... <qN>\n");
        return 1;
    }

    const std::string urdf_path = argv[1];
    const std::string frame     = argv[2];

    std::shared_ptr<ModelBase> model;
    try {
        model = std::make_shared<ModelFromURDF>(urdf_path, frame);
    } catch (const std::exception& error) {
        fprintf(stderr, "failed to load model: %s\n", error.what());
        return 1;
    }

    const int dof = model->dof();
    if (argc != 3 + dof) {
        fprintf(stderr, "expected %d joint values, got %d arguments in total\n", dof, argc - 1);
        return 1;
    }

    Eigen::VectorXd q(dof);
    for (int i = 0; i < dof; ++i) {
        q(i) = std::atof(argv[3 + i]);
    }

    const Eigen::Isometry3d pose = model->forward_kinematics(q);
    const Eigen::Quaterniond quat(pose.linear());

    printf("q           = [");
    for (int i = 0; i < dof; ++i) {
        printf("%s%.9f", i == 0 ? "" : ", ", q(i));
    }
    printf("]\n");

    printf(
        "translation = [%.9f, %.9f, %.9f]\n", pose.translation().x(), pose.translation().y(), pose.translation().z());
    printf(
        "quaternion  = [x %.9f, y %.9f, z %.9f, w %.9f]\n", quat.x(), quat.y(), quat.z(), quat.w());

    printf("R           =\n");
    for (int row = 0; row < 3; ++row) {
        printf(
            "  [%+.9f %+.9f %+.9f]\n", pose.linear()(row, 0), pose.linear()(row, 1), pose.linear()(row, 2));
    }

    // 6 维任务坐标（位置 + 旋转向量）：任何模型都能算，方便与 6 轴时期的数据对比。
    Default6DofTaskSpaceMapping mapping6;
    Eigen::VectorXd task6(6);
    if (mapping6.position_map(q, pose, &task6)) {
        printf(
            "task6       = [%.9f, %.9f, %.9f | %.9f, %.9f, %.9f]   |rv| = %.9f\n", task6(0), task6(1), task6(2),
            task6(3), task6(4), task6(5), task6.tail<3>().norm());
    } else {
        printf("task6       = <position_map failed>\n");
    }

    // 4 维任务坐标（位置 + joint4 自转）：4 轴模型实际使用的任务空间。
    Default4DofTaskSpaceMapping mapping4;
    Eigen::VectorXd task4(4);
    if (mapping4.position_map(q, pose, &task4)) {
        printf("task4       = [%.9f, %.9f, %.9f | q4 %.9f]\n", task4(0), task4(1), task4(2), task4(3));
    } else {
        printf("task4       = <position_map failed>\n");
    }

    return 0;
}
