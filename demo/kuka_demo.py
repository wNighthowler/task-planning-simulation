#!/usr/bin/env python

from __future__ import print_function
from itertools import zip_longest

import pybullet as p
from pybullet_tools.kuka_primitives import BodyPath, BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_free_motion_gen, get_holding_motion_gen, get_stable_gen
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, draw_pose, set_camera_pose, Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_for_duration, wait_if_gui, disconnect, DRAKE_IIWA_URDF, wait_if_gui, update_state, disable_real_time, HideOutput, \
    set_joint_positions, joint_from_name, set_position


def plan(robot, block, fixed, teleport):
    grasp_gen = get_grasp_gen(robot, 'top')
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport)
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), teleport=teleport)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot)
    saved_world = WorldSaver()
    for grasp, in grasp_gen(block):
        saved_world.restore()
        result1 = ik_fn(block, pose0, grasp)
        if result1 is None:
            continue
        conf1, path2 = result1
        pose0.assign()
        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            continue
        path1, = result2
        result3 = holding_motion_fn(conf1, conf0, block, grasp)
        if result3 is None:
            continue
        path3, = result3
        # print(path1.body_paths)
        # print(path2.body_paths)
        # print(path3.body_paths)
        return Command(path1.body_paths +
                          path2.body_paths +
                          path3.body_paths)
    return None

def plan_place(robot, block, fixed, teleport):
    grasp_gen = get_grasp_gen(robot, 'top')  # 生成抓取方式
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport)  # 逆运动学求解
    free_motion_fn = get_free_motion_gen(robot, fixed=fixed, teleport=teleport)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport)
    
    # 抓取前的初始状态
    pose0 = BodyPose(block)
    # print('here')
    # print(pose0.pose)
    conf0 = BodyConf(robot)
    saved_world = WorldSaver()
    
    # 抓取动作
    for grasp, in grasp_gen(block):
        saved_world.restore()
        
        result1 = ik_fn(block, pose0, grasp)
        if result1 is None:
            continue
        conf1, path2 = result1
        
        # 放置目标位置
        target_pose = BodyPose(block, Pose(Point(x=0.3, y=0.3, z=stable_z(block, fixed[0]))))  # 设置目标位置
        result2 = ik_fn(block, target_pose, grasp)
        if result2 is None:
            continue
        conf2, path4 = result2
        
        result4 = free_motion_fn(conf0, conf1)
        if result4 is None:
            continue
        path1, = result4
        
        # 连接路径：抓取 → 放置
        result3 = holding_motion_fn(conf1, conf2, block, grasp)
        if result3 is None:
            continue
        path3, = result3
        
        result5 = free_motion_fn(conf2, conf0)
        if result5 is None:
            continue
        path5, = result5
        
        # return Command(path1.body_paths + path2.body_paths + path3.body_paths + path4.body_paths)
        
        # print(path1.body_paths , path2.body_paths , path3.body_paths)
        
        return Command(path1.body_paths + path2.body_paths + path3.body_paths + path4.reverse().body_paths + path5.body_paths)
    return None


def main(display='execute'): # control | execute | step
    connect(use_gui=True)
    disable_real_time()
    draw_global_system()
    with HideOutput():
        robot = load_model(DRAKE_IIWA_URDF) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        robot2 = load_model(DRAKE_IIWA_URDF)
        # floor = load_model('models/short_floor.urdf')
        floor = p.loadURDF('plane.urdf')
        
    # base_joints = [joint_from_name(robot, name) for name in PR2_GROUPS['base']]
    # conf0 = BodyConf(robot)
    # set_joint_positions(robot, [0, 1, 2], values=(1, 1, 0.5))
    # set_pose(robot2, Pose(Point(x=-0.3, y=0.3, z=stable_z(robot2, floor))))
    set_position(robot, x=-0.1, y=-0.1, z=0)
    
    block1 = load_model(BLOCK_URDF, fixed_base=False)
    block2 = load_model(BLOCK_URDF, fixed_base=False)
    set_pose(block1, Pose(Point(x=-0.1, y=0.4, z=stable_z(block1, floor))))
    set_pose(block2, Pose(Point(x=-0.3, y=0.3, z=stable_z(block2, floor))))

    
    set_default_camera(distance=2)
    dump_world()

    saved_world = WorldSaver()
    # command = plan(robot, block1, fixed=[floor], teleport=False)
    # command = plan2(robot, block1, block2, fixed=[floor], teleport=False)
    # 抓取物体plan
    
    # command = plan_place(robot, block1, fixed=[floor], teleport=False)
    
    command = plan_place(robot, block1, fixed=[floor], teleport=False)
    command2 = plan(robot2, block2, fixed=[floor], teleport=False)

    # if command is None:
    #     print("规划失败，无法抓取或放置")
    # else:
    #     command.refine(num_steps=10).execute(time_step=0.005)

    
    print('here is command', command.body_paths)
    if (command is None) or (display is None):
        print('Unable to find a plan!')
        return
    print('here is command2', command2.body_paths)
    if (command2 is None) or (display is None):
        print('Unable to find a plan!')
        return
    #######################################################################
    # 单体分别执行
    # refine大概是精细化插值之类的功能

    saved_world.restore()
    update_state()
    wait_if_gui('{}?'.format(display))
    # if display == 'control':
    #     enable_gravity()
    #     command.control(real_time=False, dt=0)
    # elif display == 'execute':
    #     command.refine(num_steps=10).execute(time_step=0.001)
    # elif display == 'step':
    #     command.step()
    # else:
    #     raise ValueError(display)
    
    # command2.refine(num_steps=10).execute(time_step=0.001)
    # command2.execute(time_step=0.01)
    
    # 单独执行
    # for i, body_path in enumerate(command.body_paths):
    #     for j in body_path.iterator():
    #         wait_for_duration(0.01)
    #######################################################################
    
    #######################################################################
    # 同时执行
    
    # TEST
    # for body_path1, body_path2 in zip_longest(command.body_paths, command2.body_paths):
        # if body_path1 is not None:
            # for j in body_path1.iterator():
            #     wait_for_duration(0.01)
            # for j in body_path2.iterator():
            #     wait_for_duration(0.01)
        # print(type(body_path1))
        # for body_path in (body_path1, body_path2):
        #     if isinstance(body_path, BodyPath):
                # for i, configuration, in enumerate(body_path.path):
                #     set_joint_positions(body_path.body, body_path.joints, configuration)
                #     for grasp in body_path.attachments:
                #         grasp.assign()
                #     wait_for_duration(0.01)
    # 先存下BodyPath路径
    path1 = []
    path2 = []
    for j, body_path1 in enumerate(command.body_paths):
        if isinstance(body_path1, BodyPath):
            for i, configuration, in enumerate(body_path1.path):
                path1.append([body_path1, configuration])

    for j, body_path2 in enumerate(command2.body_paths):
        if isinstance(body_path2, BodyPath):
            for i, configuration, in enumerate(body_path2.path):
                path2.append([body_path2, configuration])
    # 同时执行两个机械臂的BodyPath路径
    for p1, p2 in zip_longest(path1, path2):
        if p1 is not None:
            set_joint_positions(p1[0].body, p1[0].joints, p1[1])
            for grasp in p1[0].attachments:
                grasp.assign()
        if p2 is not None:
            set_joint_positions(p2[0].body, p2[0].joints, p2[1])
            for grasp in p2[0].attachments:
                grasp.assign()
        wait_for_duration(0.01)
    #######################################################################

    print('Quit?')
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()