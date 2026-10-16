"""URDF 逆运动学工具。

给定末端目标位置（可选姿态），反解出满足目标的关节角。
基于 scipy.optimize 做数值优化，依赖同目录下的 urdf_fk.py。

用法示例：
    from urdf_fk import URDFRobot
    from urdf_ik import inverse_kinematics

    robot = URDFRobot('/home/yang/ros2_ws/genkiarm.urdf')

    target = (0.05, 0.10, 0.35)
    joints, success = inverse_kinematics(robot, target)
    print(joints)  # [q1, q2, q3, q4, q5, q6]  顺序同 URDF joint_names()

    # 反解后用正解验证
    print(robot.get_end_position(joints))  # 应接近 target
"""

import numpy as np
from scipy.optimize import minimize

from urdf_fk import URDFRobot


def inverse_kinematics(robot, target_pos, target_rpy=None,
                       initial_guess=None, end_link='zhua',
                       tol=1e-6, max_iter=200):
    """数值法逆解。

    Args:
        robot: URDFRobot 实例（已解析 URDF）。
        target_pos: 末端目标位置 (x, y, z) 或 np.array。
        target_rpy: 可选，末端目标姿态 (roll, pitch, yaw)，弧度。
        initial_guess: 可选，关节角数组作为初始猜测。不提供时
                       自动取各关节 range 中点（无 limit 则 0）。
        end_link: 末端 link 名称，默认 'zhua'。
        tol: 优化收敛阈值（位置误差平方）。
        max_iter: 最大迭代次数。

    Returns:
        (joints, success)
        joints: 关节角数组，顺序与 robot.joint_names() 一致。
        success: bool，优化是否收敛。
    """
    target_pos = np.asarray(target_pos, dtype=float)
    chain = robot.chain_to(end_link)

    # 读取关节 limits（从 XML 节点解析）
    import xml.etree.ElementTree as ET
    tree = ET.parse(robot._urdf_path)
    xml_limits = {}
    for jn in tree.getroot().findall('joint'):
        lim = jn.find('limit')
        if lim is not None:
            xml_limits[jn.get('name')] = (
                float(lim.get('lower', -np.pi)),
                float(lim.get('upper',  np.pi)),
            )
        else:
            xml_limits[jn.get('name')] = (-np.pi, np.pi)

    # bounds：每个关节的搜索范围
    bounds = [xml_limits[j.name] for j in chain]

    # 初始猜测列表：用户提供 + 零位 + 中点 + 随机多起点
    guesses = []
    if initial_guess is not None:
        guesses.append(list(initial_guess))
    guesses.append([0.0] * len(chain))
    guesses.append([(lo + hi) / 2.0 for lo, hi in bounds])
    # 随机起点（关节 limits 内均匀采样），增加多解覆盖
    rng = np.random.default_rng(42)
    for _ in range(30):
        guesses.append([rng.uniform(lo, hi) for lo, hi in bounds])

    def objective(q):
        pos, quat = robot.forward_kinematics(end_link, q)
        pos_err = np.sum((np.array(pos) - target_pos) ** 2)
        if target_rpy is not None:
            from urdf_fk import euler_to_matrix
            R_target = euler_to_matrix(target_rpy)
            x, y, z, w = quat
            R_current = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ])
            rot_err = np.sum((R_target - R_current) ** 2)
            return pos_err + rot_err
        return pos_err

    best_result = None
    best_obj = float('inf')

    for x0 in guesses:
        res = minimize(objective, x0, bounds=bounds,
                       method='SLSQP', tol=tol,
                       options={'maxiter': max_iter})
        if res.fun < best_obj:
            best_obj = res.fun
            best_result = res

    return best_result.x.tolist(), best_result.success


# --- 自测试 ---
if __name__ == '__main__':
    robot = URDFRobot('/home/yang/ros2_ws/genkiarm.urdf')

    # 先从正解拿一个已知可达的位置
    known_joints = [0.2, -0.3, 0.4, 0.0, 0.0, 0.0]
    pos_true = np.array(robot.get_end_position(known_joints))
    print(f'正解已知关节 -> 末端位置: {pos_true}')

    # 用 IK 反解（从零位开始猜）
    joints_sol, ok = inverse_kinematics(robot, pos_true,
                                        initial_guess=[0.0] * 6)
    print(f'IK 成功: {ok}')
    print(f'IK 关节角: {joints_sol}')

    # 反解得到的关节角用正解验证
    pos_verify = np.array(robot.get_end_position(joints_sol))
    print(f'反解关节 -> 正得末端: {pos_verify}')
    print(f'位置误差: {np.linalg.norm(pos_verify - pos_true):.6f} m')
