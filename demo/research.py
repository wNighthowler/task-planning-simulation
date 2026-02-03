from __future__ import print_function
import argparse
from itertools import product, zip_longest
from math import inf
import math

import pybullet as p
import time
import numpy as np
from examples.test_door import create_door, solve_collision_free
from pybullet_tools.pr2_primitives import Attach, Command, Conf, Detach, GripperCommand, State, Trajectory, apply_commands, get_grasp_gen, get_ik_fn, get_ik_ir_gen, get_motion_gen, Pose
from pybullet_tools.pr2_problems import Problem, create_pr2
from pybullet_tools.pr2_utils import TOP_HOLDING_LEFT_ARM, PR2_URDF, DRAKE_PR2_URDF, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, arm_conf, close_arm, get_arm_joints, get_carry_conf, get_gripper_joints, get_group_conf, get_group_joints, get_other_arm, open_arm, get_disabled_collisions, REST_LEFT_ARM, rightarm_from_leftarm, set_arm_conf

from pybullet_tools.utils import BLUE, YELLOW, LockRenderer, WorldSaver, base_aligned, base_aligned_z, child_link_from_joint, create_attachment, create_box, enable_gravity, connect, dump_world, get_aabb, get_extend_fn, get_joint_positions, get_max_limit, get_movable_joints, get_pose, get_quat, joint_from_name, joints_from_names, plan_joint_motion, set_configuration, set_euler, set_joint_positions, set_point, set_pose, \
    draw_global_system, draw_pose, set_camera_pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_for_duration, wait_if_gui, disconnect, DRAKE_IIWA_URDF, wait_if_gui, update_state, disable_real_time, HideOutput, \
    load_pybullet, set_quat, Euler, PI, RED, add_line, quat_from_euler, dump_body

SLEEP = 0.01
            
def plan(problem, custom_limits, **kwargs):
    robot = problem.robot
    arms = problem.arms
    arm = arms[0]
    grasp_gen = get_grasp_gen(problem)
    ik_fn = get_ik_ir_gen(problem, custom_limits=custom_limits)
    motion_fn = get_motion_gen(problem, custom_limits=custom_limits, **kwargs)
    # Stream里的内容，这里用不到
    # pose_free_fn = get_cfree_pose_pose_test()
    # approach_free_fn = get_cfree_approach_pose_test()
    # traj_free = get_cfree_traj_pose_test()
    
    initial_bq = Conf(robot, get_group_joints(robot, 'base'), get_group_conf(robot, 'base'))
    joints = get_arm_joints(robot, arm)
    base_conf = Conf(robot, joints, get_joint_positions(robot, joints))
    
    
    
    
    # movable_thing = problem.movable[0]
    # goal_obj_pose = problem.goal_obj_pose
    # pose0 = Pose(movable_thing, get_pose(movable_thing))
    
    movable_thing = problem.movable[0]
    goal_obj_pose = problem.goal_obj_pose
    pose0 = Pose(movable_thing, get_pose(movable_thing))
    
    for grasp, in grasp_gen(movable_thing):
        for bq1, cmd2 in ik_fn(arm, movable_thing, pose0, grasp):
            motion_result = motion_fn(base_conf, bq1)
            if motion_result is None:
                continue
            cmd1, = motion_result
            for bq2, cmd4 in ik_fn(arm, movable_thing, goal_obj_pose, grasp): 
                motion_result2 = motion_fn(bq1, bq2)
                if motion_result2 is None:
                    continue
                cmd3, = motion_result2
            
                [m] = cmd1.commands
                [t] = cmd2.commands
                [m2] = cmd3.commands
                [t2] = cmd4.commands
                close_gripper = GripperCommand(robot, arm, grasp.grasp_width, teleport = False)
                attach = Attach(robot, arm, grasp, movable_thing)
                
                gripper_joint = get_gripper_joints(problem.robot, arm)[0]
                position = get_max_limit(problem.robot, gripper_joint)
                
                open_gripper = GripperCommand(robot, arm, position, teleport=False)
                detach = Detach(robot, arm, movable_thing)
                # 查看输出变量类型
                # print('!!!!!!!!!!!!!!!!!!!!!!!', type(arm), arm)
                # print('!!!!!!!!!!!!!!!!!!!!!!!', type(movable_thing), movable_thing)
                # print('!!!!!!!!!!!!!!!!!!!!!!!', type(grasp), grasp)
                # print('!!!!!!!!!!!!!!!!!!!!!!!', type(cmd2), cmd2)
                
                # cmd1和cmd2都是tuple，cmd的commands才是class Commands，而m和t是继承Commands的Class Trajectory
                
                # print(type(cmd1))
                # print(type(cmd1.commands))
                # print(type(m))
                # print(type(t))
                # 类型分别是，Trajectory, Trajectory, GripperCommand, Attach, Trajectory
                # 只有Trajectory有path变量，其他的没有。这几个都是Command类的子类
                return  [m, t, close_gripper, attach, t.reverse(), m2, t2, detach, open_gripper, t2.reverse()]
    return None


def main(display = 'execute'):
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=11)
    parser.add_argument('--y', type=float, default=6)
    
    args = parser.parse_args()
    
    connect(use_gui=True)
    disable_real_time()
    draw_global_system()
    set_camera_pose(camera_point=Point(args.x, args.y-5, +5),
                    target_point=Point(args.x, args.y, 0))
    
    # 摄像头1参数
    camera1_eye = [2, 2, 2]
    camera1_target = [0, 0, 0]
    camera1_up = [0, 0, 1]

    # 摄像头2参数
    camera2_eye = [-2, -2, 2]
    camera2_target = [0, 0, 0]
    camera2_up = [0, 0, 1]

    # 获取图像函数
    def capture_camera_image(camera_eye, camera_target, camera_up, width=640, height=480, fov=60, near_val=0.1, far_val=100):
        view_matrix = p.computeViewMatrix(camera_eye, camera_target, camera_up)
        projection_matrix = p.computeProjectionMatrixFOV(fov, width / height, near_val, far_val)
        _, _, px, _, _ = p.getCameraImage(width, height, view_matrix, projection_matrix)
        return px  # 返回图像像素数据

    # 获取两个摄像头的图像
    image1 = capture_camera_image(camera1_eye, camera1_target, camera1_up)
    image2 = capture_camera_image(camera2_eye, camera2_target, camera2_up)
    
    
    
    
    line_length = 5.0
    p.addUserDebugLine([0, 0, 0], [line_length, 0, 0], [1, 0, 0], 2)  # X轴，红色
    p.addUserDebugLine([0, 0, 0], [0, line_length, 0], [0, 1, 0], 2)  # Y轴，绿色
    p.addUserDebugLine([0, 0, 0], [0, 0, line_length], [0, 0, 1], 2)  # Z轴，蓝色
    ####################################################### 
    
    
    def rotateOrientation(robot, x, y, yaw):
        # 原位置
        yaw = 3.1415926  # 弧度

        # 旋转公式
        new_x = x * math.cos(yaw) - y * math.sin(yaw)
        new_y = x * math.sin(yaw) + y * math.cos(yaw)
        
        # 设置新位置和方向
        new_position = (x-new_x, y-new_y, 0)
        new_orientation = p.getQuaternionFromEuler([0, 0, yaw])
        # 重置基座
        p.resetBasePositionAndOrientation(robot, new_position, new_orientation)
        print("New base position:", new_position)
        
        
    # 定义机器人1
    arm='left'
    grasp_type='top'
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2_1 = create_pr2()
    set_arm_conf(pr2_1, arm, initial_conf)
    open_arm(pr2_1, arm)
    set_arm_conf(pr2_1, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2_1, other_arm)
    
    base_joints = [joint_from_name(pr2_1, name) for name in PR2_GROUPS['base']]
    # base_joints 是什么？估计是机械臂的joints的代号，例如1号关节，2号关节
    base_start = (22, 2, 0)
    
    set_joint_positions(pr2_1, base_joints, base_start)
    # 设置关节的位置
    base_joints = base_joints[:2]
    # rotateOrientation(pr2_1, 22, 2, 3.1415926)
    
    
    
    # 定义机器人2
    arm='left'
    grasp_type='top'
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2_2 = create_pr2()
    set_arm_conf(pr2_2, arm, initial_conf)
    open_arm(pr2_2, arm)
    set_arm_conf(pr2_2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2_2, other_arm)
    
    base_joints = [joint_from_name(pr2_2, name) for name in PR2_GROUPS['base']]
    # base_joints 是什么？估计是机械臂的joints的代号，例如1号关节，2号关节
    base_start = (22, 4, 0)
    set_joint_positions(pr2_2, base_joints, base_start)
    # 设置关节的位置
    base_joints = base_joints[:2]
    
    # rotateOrientation(pr2_2, 22, 4, 3.1415926)
    # 定义机器人3
    arm='left'
    grasp_type='top'
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2_3 = create_pr2()
    set_arm_conf(pr2_3, arm, initial_conf)
    open_arm(pr2_3, arm)
    set_arm_conf(pr2_3, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2_3, other_arm)
    
    base_joints = [joint_from_name(pr2_3, name) for name in PR2_GROUPS['base']]
    # base_joints 是什么？估计是机械臂的joints的代号，例如1号关节，2号关节
    base_start = (21, 3, 0)
    set_joint_positions(pr2_3, base_joints, base_start)
    # 设置关节的位置
    base_joints = base_joints[:2]
    
    # rotateOrientation(pr2_3, 21, 3, 3.1415926)
    ####################################################### 
    
    #######################################################
    # 定义问题的其他模型
    w = 0.5
    h = 0.7
    # wall_w = 0.2
    # wall_l = 5
    wall_h = 2
    floor = p.loadURDF('plane.urdf')
    wallcolor = (0.87, 0.72, 0.53, 1)
    # 房间的墙
    # 外墙
    wall1 = create_box(0.2, 6, wall_h,color=wallcolor)
    set_point(wall1, (0, 3, wall_h/2))
    
    wall2 = create_box(6, 0.2, wall_h,color=wallcolor)
    set_point(wall2, (3, 0, wall_h/2))
    
    wall3 = create_box(10, 0.2, wall_h,color=wallcolor)
    set_point(wall3, (11, 0, wall_h/2))
    
    wall4 = create_box(8, 0.2, wall_h,color=wallcolor)
    set_point(wall4, (20, 0, wall_h/2))
    
    wall5 = create_box(0.2, 8, wall_h,color=wallcolor)
    set_point(wall5, (24, 4, wall_h/2))
    
    wall6 = create_box(4, 0.2, wall_h,color=wallcolor)
    set_point(wall6, (22, 8, wall_h/2))
    
    wall7 = create_box(0.2, 4, wall_h,color=wallcolor)
    set_point(wall7, (20, 10, wall_h/2))
    
    wall8 = create_box(25, 0.2, wall_h,color=wallcolor)
    set_point(wall8, (7.5, 12, wall_h/2))
    
    wall9 = create_box(0.2, 6, wall_h,color=wallcolor)
    set_point(wall9, (-5, 9, wall_h/2))
    
    wall10 = create_box(5, 0.2, wall_h,color=wallcolor)
    set_point(wall10, (-2.5, 6, wall_h/2))
    # 内墙
    tmp = 2
    wall11 = create_box(0.2, tmp, wall_h,color=wallcolor)
    set_point(wall11, (0, 11, wall_h/2))
    
    wall12 = create_box(0.2, tmp, wall_h,color=wallcolor)
    set_point(wall12, (0, 7, wall_h/2))
    
    wall13 = create_box(0.2, tmp, wall_h,color=wallcolor)
    set_point(wall13, (11, 11, wall_h/2))
    
    wall14 = create_box(0.2, tmp, wall_h,color=wallcolor)
    set_point(wall14, (11, 7, wall_h/2))
    
    wall15 = create_box(2, 0.2, wall_h,color=wallcolor)
    set_point(wall15, (1, 6, wall_h/2))
    
    wall16 = create_box(3.5, 0.2, wall_h,color=wallcolor)
    set_point(wall16, (5.75, 6, wall_h/2))
    
    wall17 = create_box(3, 0.2, wall_h,color=wallcolor)
    set_point(wall17, (11, 6, wall_h/2))
    
    wall18 = create_box(9.5, 0.2, wall_h,color=wallcolor)
    set_point(wall18, (19.25, 6, wall_h/2))
    
    wall19 = create_box(0.2, 2, wall_h,color=wallcolor)
    set_point(wall19, (6, 5, wall_h/2))

    wall20 = create_box(0.2, 2, wall_h,color=wallcolor)
    set_point(wall20, (6, 1, wall_h/2))
    
    wall21 = create_box(0.2, 2, wall_h,color=wallcolor)
    set_point(wall21, (16, 5, wall_h/2))
    
    wall22 = create_box(0.2, 2, wall_h,color=wallcolor)
    set_point(wall22, (16, 1, wall_h/2))
    
    
    # 门
    # 有门则没法做运动规划, 所以规划时需要把门设置为movable，仿真时再运行门
    def doorPose(point=None, euler=None):
        point = Point() if point is None else point
        euler = Euler() if euler is None else euler
        return point, quat_from_euler(euler)
    
    doorcolor = (0.5569, 0.4667, 0.3451, 1)
    door1 = create_door(length=2, height=2,color=doorcolor)
    set_point(door1, [6, 4, 0])
    set_configuration(door1, [math.radians(-5)])
    dump_body(door1)

    door_joint = get_movable_joints(door1)[0]
    door_link = child_link_from_joint(door_joint)
    draw_pose(doorPose(), parent=door1, parent_link=door_link)

    door2 = create_door(length=2, height=2,color=doorcolor)
    set_point(door2, [2, 6, 0])
    set_quat(door2, [0, 0, 0.7, 0.7])
    # set_euler(door2, [0, 0, 3.14/2])
    get_quat(door2)
    set_configuration(door2, [math.radians(-5)])
    dump_body(door2)

    door_joint = get_movable_joints(door2)[0]
    door_link = child_link_from_joint(door_joint)
    draw_pose(doorPose(), parent=door2, parent_link=door_link)
    
    # 其他物体
    mass = 1
    # mass = 0.01
    # mass = 1e-6
    tablecolor = (0, 1, 0, 1)
    table = create_box(w, w, h, color=tablecolor)
    set_point(table, (11, 3, h/2))
    
    cabbage = create_box(.07, .07, .1, mass=mass, color=(0, 0, 1, 1))
    #cabbage = load_model(BLOCK_URDF, fixed_base=False)
    set_point(cabbage, (11, 3, h + .1/2))
    
    table2 = create_box(w, w, h, color=tablecolor)
    set_point(table2, (17, 9, h/2))
    
    pasta = create_box(.07, .07, .1, mass=mass, color=(0, 0, 1, 1))
    #cabbage = load_model(BLOCK_URDF, fixed_base=False)
    set_point(pasta, (17, 9, h + .1/2))
    
    table3 = create_box(w, w, h, color=tablecolor)
    set_point(table3, (-2.5, 9, h/2))
    
    key = create_box(.07, .07, .1, mass=mass, color=(1, 1, 0, 1))
    #cabbage = load_model(BLOCK_URDF, fixed_base=False)
    set_point(key, (-2.5, 9, h + .1/2))
    
    table4 = create_box(w, w, h, color=(1, 0, 0, 1))
    set_point(table4, (2, 2, h/2))
    
    table5 = create_box(w, w, h, color=(1, 0, 0, 1))
    set_point(table5, (6, 10, h/2))
    
    
    
    
    #######################################################
    
    #######################################################
    # 定义问题文件
    
    saver1 = WorldSaver()
    goal_obj_pose1 = (8, 10, 0)
    goal_obj_pose1 = Pose(cabbage, ((6.2, 10.0, 0.75), (0.0, 0.0, 0.0, 1.0)))
    my_problem = Problem(robot=pr2_1, movable=[cabbage, door2], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, table2, table3, table4], goal_obj_pose=goal_obj_pose1)
    
    # 定义问题文件2
    saver2 = WorldSaver()
    goal_obj_pose2 = Pose(pasta, ((2, 2, 0.75), (0.0, 0.0, 0.0, 1.0)))
    my_problem2 = Problem(robot=pr2_2, movable=[pasta, door2], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, table2, table3, table4], goal_obj_pose=goal_obj_pose2)
    
    # 定义问题文件3
    saver3 = WorldSaver()
    goal_obj_pose3 = (5, 5, 0)
    goal_obj_pose3 = Pose(key, ((5, 5, 0.75), (0.0, 0.0, 0.0, 1.0)))
    my_problem3 = Problem(robot=pr2_3, movable=[key, door2], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, table2, table3, table4], goal_obj_pose=goal_obj_pose3)
    #######################################################
    
    #######################################################
    # z = base_aligned_z(pr2)
    # print(z)
    
    # set_point(pr2, Point(z=z))
    # print(get_aabb(pr2))
    wait_if_gui('Begin to Plan?')
    #######################################################
    
    #######################################################
    # 自定义计划
    restarts = 1
    max_iterations = 1000000
    smooth = 0
    algorithm = 'lazy_prm_star'
    # 'prm''lazy_prm''lazy_prm_star''rrt''rrt_connect''birrt''rrt_star''lattice'
    
    custom_limits = {
        joint_from_name(pr2_1, 'x') : (-5, 24), 
        joint_from_name(pr2_1, 'y') : (-12, 12)   
    } 
    custom_limits2 = {
        joint_from_name(pr2_2, 'x') : (-5, 24), 
        joint_from_name(pr2_2, 'y') : (-12, 12)
    } 
    custom_limits3 = {
        joint_from_name(pr2_3, 'x') : (-5, 24), 
        joint_from_name(pr2_3, 'y') : (-12, 12)
    } 
    commands = plan(my_problem, custom_limits, restarts=restarts, max_iterations=max_iterations, smooth=smooth, algorithm = algorithm)
    my_problem.remove_gripper()
    saver1.restore()
    
    commands2 = plan(my_problem2, custom_limits2, restarts=restarts, max_iterations=max_iterations, smooth=smooth, algorithm = algorithm)
    my_problem2.remove_gripper()
    saver2.restore()
    
    commands3 = plan(my_problem3, custom_limits3, restarts=restarts, max_iterations=max_iterations, smooth=smooth, algorithm = algorithm)
    my_problem3.remove_gripper()
    saver3.restore()
    
    #######################################################
    
    #######################################################
    # 同时执行自定义计划
    state0 = State()
    path1 = []
    path2 = []
    path3 = []
    if commands is None:
        print('Commands1 is None, NO SOLUTION!')
    elif commands2 is None:
        print('Commands2 is None, NO SOLUTION!')
    elif commands3 is None:
        print('Commands3 is None, NO SOLUTION!')
    else:
        # wait_if_gui('{}?'.format(display))
        while True:
            display = input('Execute(Y) or Stop(N): ')
            if(display == 'Execute' or display == 'Y' or display == 'y'):
                state = State()
                for i, command in enumerate(commands):
                    if isinstance(command, Trajectory):
                        for conf in command.path:
                            path1.append(conf)
                    if isinstance(command, Attach):
                        path1.append(command)
                    if isinstance(command, Detach):
                        path1.append(command) 
                
                for i, command in enumerate(commands2):
                    if isinstance(command, Trajectory):
                        for conf in command.path:
                            path2.append(conf)
                    if isinstance(command, Attach):
                        path2.append(command)
                    if isinstance(command, Detach):
                        path2.append(command) 
                
                for i, command in enumerate(commands3):
                    if isinstance(command, Trajectory):
                        for conf in command.path:
                            path3.append(conf)
                    if isinstance(command, Attach):
                        path3.append(command)
                    if isinstance(command, Detach):
                        path3.append(command) 
                
                start1=(22, 2,0.5)
                start2=(22, 4,0.5)
                start3=(21, 3, 0.5)
                for p1, p2, p3 in zip_longest(path1, path2, path3):
                    if p1 is not None:
                        if isinstance(p1, Conf):
                            p1.assign()
                            if(len(p1.values)==3):
                                end = p1.values
                                end = (end[0], end[1], 0.5)
                                add_line(start1, end, color=RED, width=10)
                                start1 = end
                            
                            
                    if isinstance(p1, Attach):
                        state.attachments[p1.body] = create_attachment(p1.robot, p1.link, p1.body)
                        state.grasps[p1.body] = p1.grasp
                        del state.poses[p1.body]
                    
                    if isinstance(p1, Detach):
                        del state.attachments[p1.body]
                        state.poses[p1.body] = Pose(p1.body, get_pose(p1.body))
                        del state.grasps[p1.body]
                        
                    if p2 is not None:
                        if isinstance(p2, Conf):
                            p2.assign()
                            if(len(p2.values)==3):
                                end = p2.values
                                end = (end[0], end[1], 0.5)
                                add_line(start2, end, color=BLUE, width=10)
                                start2 = end
                    
                    if isinstance(p2, Attach):
                        state.attachments[p2.body] = create_attachment(p2.robot, p2.link, p2.body)
                        state.grasps[p2.body] = p2.grasp
                        del state.poses[p2.body]
                        
                    if isinstance(p2, Detach):
                        del state.attachments[p2.body]
                        state.poses[p2.body] = Pose(p2.body, get_pose(p2.body))
                        del state.grasps[p2.body]
                    
                    if p3 is not None:
                        if isinstance(p3, Conf):
                            p3.assign()
                            if(len(p3.values)==3):
                                end = p3.values
                                end = (end[0], end[1], 0.5)
                                add_line(start3, end, color=YELLOW, width=10)
                                start3 = end
                    
                    if isinstance(p3, Attach):
                        state.attachments[p3.body] = create_attachment(p3.robot, p3.link, p3.body)
                        state.grasps[p3.body] = p3.grasp
                        del state.poses[p2.body]
                        
                    if isinstance(p3, Detach):
                        del state.attachments[p3.body]
                        state.poses[p3.body] = Pose(p3.body, get_pose(p3.body))
                        del state.grasps[p3.body]
                        
                    
                    for door, robot in product([door1, door2], [pr2_1, pr2_2, pr2_3]):
                        solve_collision_free(door, robot, draw=False)
                    # solve_collision_free(door1, pr2_1, draw=False)
                    # solve_collision_free(door1, pr2_2, draw=False)
                    state.assign()
                    wait_for_duration(0.001)
                    # wait_if_gui()
            else:
                break
    wait_if_gui('Finish?')
        
        
    
    #######################################################
    disconnect()
    
if __name__ == '__main__':
    main()