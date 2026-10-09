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
 * fk_dump <urdf> <q1> <q2> <q3> <q4> <q5> <q6>
 * @endcode
 *
 * @note 只用于离线排查，不参与控制器运行。
 */

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
    if (argc < 8) {
        fprintf(stderr, "usage: fk_dump <urdf> <q1> <q2> <q3> <q4> <q5> <q6>\n");
        return 1;
    }

    const std::string urdf_path = argv[1];

    std::shared_ptr<ModelBase> model;
    try {
        model = std::make_shared<ModelFromURDF>(urdf_path, "link6");
    } catch (const std::exception& error) {
        fprintf(stderr, "failed to load model: %s\n", error.what());
        return 1;
    }

    Eigen::VectorXd q(model->dof());
    for (Eigen::Index i = 0; i < q.size(); ++i) {
        q(i) = std::atof(argv[2 + i]);
    }

    const Eigen::Isometry3d pose = model->forward_kinematics(q);
    const Eigen::Quaterniond quat(pose.linear());

    printf("q           = [");
    for (Eigen::Index i = 0; i < q.size(); ++i) {
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

    Default6DofTaskSpaceMapping mapping;
    Eigen::VectorXd task(6);
    if (mapping.position_map(q, pose, &task)) {
        printf(
            "task6       = [%.9f, %.9f, %.9f | %.9f, %.9f, %.9f]   |rv| = %.9f\n", task(0), task(1), task(2),
            task(3), task(4), task(5), task.tail<3>().norm());
    } else {
        printf("task6       = <position_map failed>\n");
    }

    return 0;
}
