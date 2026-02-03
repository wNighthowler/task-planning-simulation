from __future__ import print_function
from itertools import zip_longest
from math import inf

import pybullet as p
import time
import numpy as np
from pybullet_tools.pr2_primitives import Attach, Command, Conf, Detach, GripperCommand, State, Trajectory, apply_commands, get_grasp_gen, get_ik_fn, get_ik_ir_gen, get_motion_gen, Pose
from pybullet_tools.pr2_problems import Problem, create_pr2
from pybullet_tools.pr2_utils import TOP_HOLDING_LEFT_ARM, PR2_URDF, DRAKE_PR2_URDF, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, arm_conf, close_arm, get_arm_joints, get_carry_conf, get_gripper_joints, get_group_conf, get_group_joints, get_other_arm, open_arm, get_disabled_collisions, REST_LEFT_ARM, rightarm_from_leftarm, set_arm_conf


from pybullet_tools.utils import LockRenderer, WorldSaver, base_aligned_z, create_attachment, create_box, enable_gravity, connect, dump_world, get_aabb, get_extend_fn, get_joint_positions, get_max_limit, get_pose, joint_from_name, joints_from_names, plan_joint_motion, set_joint_positions, set_point, set_pose, \
    draw_global_system, draw_pose, set_camera_pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_for_duration, wait_if_gui, disconnect, DRAKE_IIWA_URDF, wait_if_gui, update_state, disable_real_time, HideOutput, \
    load_pybullet, set_quat, Euler, PI, RED, add_line, quat_from_euler, dump_body

SLEEP = 0.01

def test_drake_base_motion(pr2, base_start, base_goal, obstacles=[]):
    # TODO: combine this with test_arm_motion
    """
    Drake's PR2 URDF has explicit base joints
    """
    disabled_collisions = get_disabled_collisions(pr2)
    
    base_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['base']]
    # base_joints 是什么？估计是机械臂的joints的代号，例如1号关节，2号关节
    
    set_joint_positions(pr2, base_joints, base_start)
    # 设置关节的位置
    base_joints = base_joints[:2]
    base_goal = base_goal[:len(base_joints)]
    
    wait_if_gui('Plan Base?')
    with LockRenderer(lock=False):
        # plan_joint_motion可以规划机器人的机械臂与移动
        # 如果end_conf是arm_goal,即机械臂的配置，那么规划机械臂；如果end_conf是base_goal，那么规划移动。
        # base是指机器人底座（猜测）
        base_path = plan_joint_motion(pr2, base_joints, base_goal, obstacles=obstacles,
                                      disabled_collisions=disabled_collisions)
    if base_path is None:
        print('Unable to find a base path')
        return
    print(len(base_path))
    for bq in base_path:
        set_joint_positions(pr2, base_joints, bq)
        if SLEEP is None:
            wait_if_gui('Continue?')
        else:
            wait_for_duration(SLEEP)

    
            
def plan(problem):
    robot = problem.robot
    arms = problem.arms
    arm = arms[0]
    goal_pose = problem.goal_pose
    
    custom_limits = {
        joint_from_name(robot, 'x') : (-6, 6), 
        joint_from_name(robot, 'y') : (-6, 6)
    }  
    grasp_gen = get_grasp_gen(problem)
    ik_fn = get_ik_ir_gen(problem, custom_limits = custom_limits)
    motion_fn = get_motion_gen(problem, custom_limits = custom_limits)
    
    initial_bq = Conf(robot, get_group_joints(robot, 'base'), get_group_conf(robot, 'base'))
    joints = get_arm_joints(robot, arm)
    base_conf = Conf(robot, joints, get_joint_positions(robot, joints))

    
    
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
                
                # set_joint_positions(robot, [0, 1, 2], values=goal_pose)
                end_conf = Conf(robot, joints, goal_pose)
                
                print(type(base_conf), type(bq1), type(bq2), type(end_conf))
                print(base_conf.values, bq1.values, bq2.values, end_conf.values)
    
                motion_result3 = motion_fn(bq2, end_conf)
                if motion_result3 is None:
                    continue
                cmd5, = motion_result3
                

                [m] = cmd1.commands
                [t] = cmd2.commands
                [m2] = cmd3.commands
                [t2] = cmd4.commands
                [m3] = cmd5.commands
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
                return  [m, t, close_gripper, attach, t.reverse(), m2, t2, detach, open_gripper, t2.reverse(), m3]

    return None


def main(display = 'execute'):
    connect(use_gui=True)
    disable_real_time()
    draw_global_system()
    # with HideOutput():
    #     pr2 = load_model(DRAKE_PR2_URDF, fixed_base=True)
    #     floor = p.loadURDF('plane.urdf')
    # table_path = "models/table_collision/table.urdf"
    # table = load_pybullet(table_path, fixed_base=True)
    # set_quat(table, quat_from_euler(Euler(yaw=PI/2)))
    # # table/table.urdf, table_square/table_square.urdf, cube.urdf, block.urdf, door.urdf
    # obstacles = [floor, table]
    # dump_body(pr2)
    
    ####################################################### 
    # 定义机器人
    arm='left'
    grasp_type='top'
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)
    
    # arm_start = SIDE_HOLDING_LEFT_ARM
    # arm_start = TOP_HOLDING_LEFT_ARM
    # arm_start = REST_LEFT_ARM
    # arm_goal = TOP_HOLDING_LEFT_ARM
    # arm_goal = SIDE_HOLDING_LEFT_ARM
    
    # left_joints = joints_from_names(pr2, PR2_GROUPS['left_arm'])
    # right_joints = joints_from_names(pr2, PR2_GROUPS['right_arm'])
    # torso_joints = joints_from_names(pr2, PR2_GROUPS['torso'])
    
    # set_joint_positions(pr2, left_joints, arm_start)
    # set_joint_positions(pr2, right_joints, rightarm_from_leftarm(REST_LEFT_ARM))
    # set_joint_positions(pr2, torso_joints, [0.2])
    # open_arm(pr2, 'left')
    
    # disabled_collisions = get_disabled_collisions(pr2)
    
    base_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['base']]
    # base_joints 是什么？估计是机械臂的joints的代号，例如1号关节，2号关节
    base_start = (3, -1, 0)
    print(base_joints)
    set_joint_positions(pr2, base_joints, base_start)
    # 设置关节的位置
    base_joints = base_joints[:2]
    
    
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
    base_start = (4, -2, 0)
    set_joint_positions(pr2_2, base_joints, base_start)
    # 设置关节的位置
    base_joints = base_joints[:2]
    ####################################################### 
    
    #######################################################
    # 定义问题的其他模型
    w = 0.5
    h = 0.7
    floor = p.loadURDF('plane.urdf')
    
    table = create_box(w, w, h, color=(.75, .75, .75, 1))
    set_point(table, (2, 2, h/2))
    
    table2 = create_box(w/2, w/2, h, color=(.75, .75, .75, 1))
    set_point(table2, (2, 0, h/2))
    
    mass = 1
    #mass = 0.01
    #mass = 1e-6
    cabbage = create_box(.07, .07, .1, mass=mass, color=(0, 1, 0, 1))
    #cabbage = load_model(BLOCK_URDF, fixed_base=False)
    set_point(cabbage, (0, 2, h + .1/2))

    sink = create_box(w, w, h, color=(.25, .25, .75, 1))
    set_point(sink, (0, 2, h/2))

    stove = create_box(w, w, h, color=(.75, .25, .25, 1))
    set_point(stove, (0, -2, h/2))
    
    obstacles = [floor, table, mass, cabbage, sink, stove]
    
    # 定义问题的其他模型2（在上面的基础上）
    
    table3 = create_box(w/2, w/2, h, color=(.75, .75, .75, 1))
    set_point(table3, (-3, 0, h/2))
    
    pasta = create_box(.07, .07, .1, mass=mass, color=(0, 1, 0, 1))
    #cabbage = load_model(BLOCK_URDF, fixed_base=False)
    set_point(pasta, (-3, 0, h + .1/2))
    
    #######################################################
    
    #######################################################
    # 定义问题文件
    saver1 = WorldSaver()
    # print(get_pose(cabbage))
    goal_pose1 = (3, -1, 0)
    goal_obj_pose1 = Pose(pasta, ((2.0, 2.0, 0.75), (0.0, 0.0, 0.0, 1.0)))
    my_problem = Problem(robot=pr2, movable=[pasta], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, table2, table3, sink, stove], goal_pose=goal_pose1, goal_obj_pose=goal_obj_pose1)
    
    # 定义问题文件2
    goal_pose2 = (0, 0, 0)
    goal_obj_pose2 = Pose(cabbage, ((0.0, -2.0, 0.75), (0.0, 0.0, 0.0, 1.0)))
    my_problem2 = Problem(robot=pr2_2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, table2, table3, sink, stove], goal_pose=goal_pose2, goal_obj_pose=goal_obj_pose1)
    #######################################################
    
    #######################################################
    
    
    # arm_start = SIDE_HOLDING_LEFT_ARM
    # arm_start = TOP_HOLDING_LEFT_ARM
    # arm_start = REST_LEFT_ARM
    # arm_goal = TOP_HOLDING_LEFT_ARM
    #######################################################
    
    #######################################################
    # z = base_aligned_z(pr2)
    # print(z)
    
    # set_point(pr2, Point(z=z))
    # print(get_aabb(pr2))
    wait_if_gui()
    #######################################################
    
    #######################################################
    # 画线示意
    # base_start = (-2, -2, 0)
    # base_goal = (2, 2, 0)
    # add_line(base_start, base_goal, color=RED)
    # print(base_start)
    
    # 运动执行
    # test_drake_base_motion(pr2, base_start, base_goal, obstacles=obstacles)
    #######################################################
    
    #######################################################
    # 自定义计划
    commands = plan(my_problem)
    my_problem.remove_gripper()
    saver1.restore()
    
    # commands2 = plan(my_problem2)
    # my_problem2.remove_gripper()
    # saver1.restore()
    
    # print(type(commands))
    # print(type(commands[0]))
    
    #######################################################
    
    #######################################################
    # 执行自定义计划
    # if commands is None:
    #     print('No Solution!')
    # else:
    #     wait_if_gui('Execute?')
        
    #     apply_commands(State(), commands, time_step=0.01)
        
    #     wait_if_gui('Finish?')
    
    # 执行自定义计划细节
    # state = State()
    # if commands is None:
    #     print('No Solution!')
    # else:
    #     wait_if_gui('Execute?')
    #     for i, command in enumerate(commands):
    #         # for j, _ in enumerate(command.apply(state)):
    #         print(type(command))
    #         if isinstance(command, Trajectory):
    #             for conf in command.path:
    #                 conf.assign()
    #                 state.assign()
    #                 # if j == 0:
    #                 #     continue
    #                 wait_for_duration(0.01)
    #         # 其实似乎没用
    #         if isinstance(command, GripperCommand):
    #             joints = get_gripper_joints(command.robot, command.arm)
    #             start_conf = get_joint_positions(command.robot, joints)
    #             end_conf = [command.position] * len(joints)
    #             extend_fn = get_extend_fn(command.robot, joints)
    #             path = [start_conf] + list(extend_fn(start_conf, end_conf))
    #             for positions in path:
    #                 set_joint_positions(command.robot, joints, positions)
    #             state.assign()
    #             wait_for_duration(0.01)
    #         # 必须建立attach约束
    #         if isinstance(command, Attach):
    #             state.attachments[command.body] = create_attachment(command.robot, command.link, command.body)
    #             state.grasps[command.body] = command.grasp
    #             del state.poses[command.body]
    #             state.assign()
    #             wait_for_duration(0.01)
    #         if isinstance(command, Detach):
    #             del state.attachments[command.body]
    #             state.poses[command.body] = Pose(command.body, get_pose(command.body))
    #             del state.grasps[command.body]
    #             state.assign()
    #             wait_for_duration(0.01)
    #     wait_if_gui('Finish?')
    #######################################################
    
    #######################################################
    # 同时执行自定义计划
    state = State()
    path1 = []
    path2 = []
    if commands is None:
        print('Commands1 is None, NO SOLUTION!')
    # elif commands2 is None:
    #     print('Commands2 is None, NO SOLUTION!')
    else:
        wait_if_gui('Execute?')
        for i, command in enumerate(commands):
            if isinstance(command, Trajectory):
                for conf in command.path:
                    path1.append(conf)
            if isinstance(command, Attach):
                path1.append(command)
        
        # for i, command in enumerate(commands2):
        #     if isinstance(command, Trajectory):
        #         for conf in command.path:
        #             path2.append(conf)
        #     if isinstance(command, Attach):
        #         path2.append(command)
        k=0
        start1 = (3, -1, 0)
        for p1, p2 in zip_longest(path1, path2):
            # print(type(p1))
            k=k+1
            if p1 is not None:
                if isinstance(p1, Conf):
                    p1.assign()
                    
                    # joints = get_arm_joints(robot, arm)
                    # base_conf = Conf(robot, joints, get_joint_positions(robot, joints))
                    if(k%10==0):
                        if(len(p1.values)==3):
                            end = p1.values
                            end = (end[0], end[1], 0)
                            add_line(start1, end, color=RED, width=10)
                            start1 = end
                    #     arm='left'
                    #     grasp_type='top'
                    #     other_arm = get_other_arm(arm)
                    #     initial_conf = get_carry_conf(arm, grasp_type)

                    #     pr2 = create_pr2()
                    #     p.setCollisionFilterGroupMask(pr2, -1, 0, 0)
                    #     set_arm_conf(pr2, arm, initial_conf)
                    #     open_arm(pr2, arm)
                    #     set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
                    #     close_arm(pr2, other_arm)
                    
                    #     # set_arm_conf(pr2, p1.ar, p1)
                    #     set_joint_positions(pr2, p1.joints, p1.values)
                    
            if isinstance(p1, Attach):
                state.attachments[p1.body] = create_attachment(p1.robot, p1.link, p1.body)
                state.grasps[p1.body] = p1.grasp
                del state.poses[p1.body]
                
            if p2 is not None:
                if isinstance(p2, Conf):
                    p2.assign()
            
            if isinstance(p2, Attach):
                state.attachments[p2.body] = create_attachment(p2.robot, p2.link, p2.body)
                state.grasps[p2.body] = p2.grasp
                del state.poses[p2.body]
                
            state.assign()
            wait_for_duration(0.01)
        wait_if_gui('Finish?')
        
        
    
    #######################################################
    disconnect()
    
if __name__ == '__main__':
    main()