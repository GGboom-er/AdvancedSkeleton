#!/usr/bin/env python
# _*_ coding:cp936 _*_

"""
@author: GGboom
@license: MIT
@contact: https://github.com/GGboom-er
@file: fixADVMatrix.py
@date: 2025/3/6 11:47
@desc: 
"""
import re
import maya.cmds as cmds

def select_objects_by_type_and_name_regex( object_type, name_pattern ):
    """
    使用正则表达式匹配指定类型的物体名称。
    select_objects_by_type_and_name_regex("joint", r"^.*Partial.*$")
    :param object_type: str - 要查找的物体类型（例如 "joint"）
    :param name_pattern: str - 正则表达式，用于匹配物体名称
    """
    try:
        objects = cmds.ls(type=object_type)
        if not objects:
            cmds.warning("No objects of type '{}' found.".format(object_type))
            return

        pattern = re.compile(name_pattern)
        matched_objects = [obj for obj in objects if pattern.search(obj)]

        if matched_objects:
            cmds.select(matched_objects, replace=True)
            print("Matched objects: {}".format(", ".join(matched_objects)))
        else:
            cmds.warning("No objects of type '{}' matching pattern '{}' found.".format(object_type, name_pattern))
    except Exception as e:
        cmds.error("An error occurred: {}".format(e))
def update_blend_matrix( bone_name ):
    blend_matrix_node = bone_name.replace("Partial", "PartialBM")  # 直接替换 Partial 为 PartialBM

    if not cmds.objExists(blend_matrix_node) or cmds.nodeType(blend_matrix_node) != "blendMatrix":
        return None

    target_matrix_attr = blend_matrix_node + ".target[0].targetMatrix"
    input_matrix_attr = blend_matrix_node + ".inputMatrix"

    if cmds.objExists(target_matrix_attr) and cmds.objExists(input_matrix_attr):
        target_matrix = cmds.getAttr(target_matrix_attr)
        cmds.setAttr(input_matrix_attr, *target_matrix, type="matrix")
        return blend_matrix_node

    return None
def rebuildPartial(key = 'Partial'):
    cmds.select(clear=True)
    select_objects_by_type_and_name_regex("joint", r"^.*"+key+".*$")
    for jointName in cmds.ls(sl =1):
        for attr in ['tx','ty','tz','rx','ry','rz','sx','sy','sz','jointOrientX','jointOrientY','jointOrientZ']:
            if attr in ['sx','sy','sz']:
                cmds.setAttr(jointName+'.'+attr, 1)
            else:
                cmds.setAttr(jointName+'.'+attr, 0)
        update_blend_matrix(jointName)
    cmds.select(clear=True)

def rebuild_bone_matrices( bone_list ):
    """
    修复 adv 辅助骨骼矩阵信息
    :param bone_list: 需要修复的骨骼列表
    :return: 处理过的骨骼名称列表
    """
    processed_bones = []  # 用于存储已处理的骨骼名称

    for bone in bone_list:
        if not cmds.objExists(bone):
            print(f"骨骼 {bone} 不存在，跳过")
            continue

        # 获取骨骼的所有自定义属性
        attributes = cmds.listAttr(bone, userDefined=True) or []

        # 获取左右标识
        side = "_L" if bone.endswith("_L") else "_R" if bone.endswith("_R") else None
        if not side:
            print(f"骨骼 {bone} 没有 _L 或 _R 后缀，跳过")
            continue

        base_name = bone.replace(side, "")

        # 处理 opmrotateZ 逻辑
        if "opmrotateZ" in attributes and cmds.getAttr(f"{bone}.opmrotateZ") != 0.0:
            parent_bone = cmds.listRelatives(bone, parent=True, type='joint')
            if parent_bone:
                parent_bone = parent_bone[0]
                world_matrix = cmds.getAttr(f"{parent_bone}.worldMatrix[0]")
                inv_world_matrix = cmds.getAttr(f"{bone}.worldInverseMatrix[0]")

                mult_matrix_node = f"{bone}DMMMrotateZ"
                if cmds.objExists(mult_matrix_node) and cmds.nodeType(mult_matrix_node) == 'multMatrix':
                    cmds.setAttr(f"{mult_matrix_node}.matrixIn[1]", *world_matrix, type="matrix")
                    cmds.setAttr(f"{mult_matrix_node}.matrixIn[2]", *inv_world_matrix, type="matrix")
                    print(f"修正 {mult_matrix_node} 的 matrixIn[1] 和 matrixIn[2]")
                    processed_bones.append(bone)
        # 处理 qRotateX/Y/Z 逻辑
        q_rotate_attrs = {"qRotateX", "qRotateY", "qRotateZ"}
        for attr in q_rotate_attrs:
            if attr in attributes and cmds.getAttr(f"{bone}.{attr}") != 0.0:
                twist_node = f"{base_name}QRotateMMTwist{side}"
                if cmds.objExists(twist_node):
                    if cmds.getAttr(f"{twist_node}.matrixIn", size=True) > 0:
                        source_matrix_data = cmds.getAttr(f"{twist_node}.matrixIn[0]")
                        if source_matrix_data:
                            inverse_matrix = cmds.matrixUtil(*source_matrix_data, q=True, inverse=1)
                            cmds.setAttr(f"{twist_node}.matrixIn[2]", *inverse_matrix, type="matrix")
                            print(f"设置 {twist_node}.matrixIn[2] 逆矩阵")
                break  # 一旦处理了一个旋转属性，跳出循环
                processed_bones.append(bone)
        # 添加处理过的骨骼到列表

    return processed_bones  # 返回处理过的骨骼名称列表




def connect_solver_matrix( base_name, reverse_mode=False ):
    """Fk驱动与Matrix驱动切换连接
    [connect_solver_matrix(i, reverse_mode=True) for i in [...]  # 反向操作示例

    :param base_name: 基础关节名称 (e.g. 'Ankle')
    :param reverse_mode: False=FK转Matrix, True=Matrix转FK
    """
    sides = ["_L", "_R"]

    for side in sides:
        solver_node = f"{base_name}{side}_UERBFSolver"
        # 验证解算器节点
        if not cmds.objExists(solver_node) or cmds.nodeType(solver_node) != "UERBFSolverNode":
            print(f"× 节点 {solver_node} 不存在或类型不匹配")
            continue

        if reverse_mode:
            ##########################
            # 反向模式：Matrix转FK #
            ##########################
            # 查找可能的Matrix源节点
            matrix_source = None
            priority_nodes = [
                f"{base_name}QRotateMMTwist{side}",
                f"{base_name}{side}DMMMrotateZ"
            ]

            # 检查存在的Matrix源节点
            for node in priority_nodes:
                if cmds.objExists(node):
                    matrix_source = node
                    break

            if not matrix_source:
                print(f"× 反向模式：{base_name}{side} 未找到Matrix源节点")
                continue

            # 获取当前连接信息
            current_conn = cmds.listConnections(
                f"{solver_node}.inputs[0]",
                source=True,
                destination=False,
                plugs=True
            )

            # 验证当前Matrix连接
            if not current_conn or not current_conn[0].startswith(f"{matrix_source}.matrixSum"):
                print(f"× 反向模式：{solver_node}.inputs[0] 未连接到 {matrix_source}.matrixSum")
                continue

            # 准备FK节点
            fk_node = f"FK{base_name}{side}"
            if not cmds.objExists(fk_node) or cmds.nodeType(fk_node) != "transform":
                print(f"× 反向模式：FK节点 {fk_node} 不存在或类型错误")
                continue

            # 执行反向连接操作
            try:
                # 断开Matrix连接
                cmds.disconnectAttr(current_conn[0], f"{solver_node}.inputs[0]")
                # 连接FK的世界矩阵
                cmds.connectAttr(
                    f"{fk_node}.matrix",
                    f"{solver_node}.inputs[0]",
                    force=True
                )
                print(f"√ 反向连接：{fk_node}.matrix -> {solver_node}.inputs[0]")
            except Exception as e:
                print(f"× 反向连接失败: {str(e)}")

        else:
            ########################
            # 默认模式：FK转Matrix #
            ########################
            # 验证FK节点
            fk_node = f"FK{base_name}{side}"
            if not cmds.objExists(fk_node) or cmds.nodeType(fk_node) != "transform":
                print(f"× 默认模式：FK节点 {fk_node} 不存在或类型错误")
                continue

            # 验证当前FK连接
            current_conn = cmds.listConnections(
                f"{solver_node}.inputs[0]",
                source=True,
                destination=False
            )
            if not current_conn or current_conn[0] != fk_node:
                print(f"× 默认模式：{solver_node}.inputs[0] 未连接 {fk_node}")
                continue

            # 查找Matrix源节点
            matrix_source = None
            priority_nodes = [
                f"{base_name}QRotateMMTwist{side}",
                f"{base_name}{side}DMMMrotateZ"
            ]

            for node in priority_nodes:
                if cmds.objExists(node):
                    matrix_source = node
                    break

            if not matrix_source:
                print(f"× 默认模式：{base_name}{side} 未找到Matrix源节点")
                continue

            # 执行Matrix连接
            try:
                cmds.connectAttr(
                    f"{matrix_source}.matrixSum",
                    f"{solver_node}.inputs[0]",
                    force=True  # 强制断开原有连接
                )
                print(f"√ Matrix连接：{matrix_source}.matrixSum -> {solver_node}.inputs[0]")
            except Exception as e:
                print(f"× Matrix连接失败: {str(e)}")



if __name__ == "__main__":
    #次级辅助骨骼重定向
    rebuildPartial(key='Partial')

    #主要骨骼重定向
    rebuild_bone_matrices(cmds.ls(sl =1))

    #Fk驱动转为Matrix驱动
    [connect_solver_matrix(i, reverse_mode=0) for i in
     ['Ankle', 'Elbow', 'Hip', 'IndexFinger1', 'IndexFinger2', 'IndexFinger3', 'Knee', 'MiddleFinger1', 'MiddleFinger2',
      'MiddleFinger3', 'PinkyFinger1', 'PinkyFinger2', 'PinkyFinger3', 'RingFinger1', 'RingFinger2', 'RingFinger3',
      'Scapula', 'Shoulder', 'ThumbFinger2', 'ThumbFinger3', 'Wrist']]

