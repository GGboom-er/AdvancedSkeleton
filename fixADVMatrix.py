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
    ʹ��������ʽƥ��ָ�����͵��������ơ�
    select_objects_by_type_and_name_regex("joint", r"^.*Partial.*$")
    :param object_type: str - Ҫ���ҵ��������ͣ����� "joint"��
    :param name_pattern: str - ������ʽ������ƥ����������
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
    blend_matrix_node = bone_name.replace("Partial", "PartialBM")  # ֱ���滻 Partial Ϊ PartialBM

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
    �޸� adv ��������������Ϣ
    :param bone_list: ��Ҫ�޸��Ĺ����б�
    :return: ������Ĺ��������б�
    """
    processed_bones = []  # ���ڴ洢�Ѵ���Ĺ�������

    for bone in bone_list:
        if not cmds.objExists(bone):
            print(f"���� {bone} �����ڣ�����")
            continue

        # ��ȡ�����������Զ�������
        attributes = cmds.listAttr(bone, userDefined=True) or []

        # ��ȡ���ұ�ʶ
        side = "_L" if bone.endswith("_L") else "_R" if bone.endswith("_R") else None
        if not side:
            print(f"���� {bone} û�� _L �� _R ��׺������")
            continue

        base_name = bone.replace(side, "")

        # ���� opmrotateZ �߼�
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
                    print(f"���� {mult_matrix_node} �� matrixIn[1] �� matrixIn[2]")
                    processed_bones.append(bone)
        # ���� qRotateX/Y/Z �߼�
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
                            print(f"���� {twist_node}.matrixIn[2] �����")
                break  # һ��������һ����ת���ԣ�����ѭ��
                processed_bones.append(bone)
        # ��Ӵ�����Ĺ������б�

    return processed_bones  # ���ش�����Ĺ��������б�




def connect_solver_matrix( base_name, reverse_mode=False ):
    """Fk������Matrix�����л�����
    [connect_solver_matrix(i, reverse_mode=True) for i in [...]  # �������ʾ��

    :param base_name: �����ؽ����� (e.g. 'Ankle')
    :param reverse_mode: False=FKתMatrix, True=MatrixתFK
    """
    sides = ["_L", "_R"]

    for side in sides:
        solver_node = f"{base_name}{side}_UERBFSolver"
        # ��֤�������ڵ�
        if not cmds.objExists(solver_node) or cmds.nodeType(solver_node) != "UERBFSolverNode":
            print(f"�� �ڵ� {solver_node} �����ڻ����Ͳ�ƥ��")
            continue

        if reverse_mode:
            ##########################
            # ����ģʽ��MatrixתFK #
            ##########################
            # ���ҿ��ܵ�MatrixԴ�ڵ�
            matrix_source = None
            priority_nodes = [
                f"{base_name}QRotateMMTwist{side}",
                f"{base_name}{side}DMMMrotateZ"
            ]

            # �����ڵ�MatrixԴ�ڵ�
            for node in priority_nodes:
                if cmds.objExists(node):
                    matrix_source = node
                    break

            if not matrix_source:
                print(f"�� ����ģʽ��{base_name}{side} δ�ҵ�MatrixԴ�ڵ�")
                continue

            # ��ȡ��ǰ������Ϣ
            current_conn = cmds.listConnections(
                f"{solver_node}.inputs[0]",
                source=True,
                destination=False,
                plugs=True
            )

            # ��֤��ǰMatrix����
            if not current_conn or not current_conn[0].startswith(f"{matrix_source}.matrixSum"):
                print(f"�� ����ģʽ��{solver_node}.inputs[0] δ���ӵ� {matrix_source}.matrixSum")
                continue

            # ׼��FK�ڵ�
            fk_node = f"FK{base_name}{side}"
            if not cmds.objExists(fk_node) or cmds.nodeType(fk_node) != "transform":
                print(f"�� ����ģʽ��FK�ڵ� {fk_node} �����ڻ����ʹ���")
                continue

            # ִ�з������Ӳ���
            try:
                # �Ͽ�Matrix����
                cmds.disconnectAttr(current_conn[0], f"{solver_node}.inputs[0]")
                # ����FK���������
                cmds.connectAttr(
                    f"{fk_node}.matrix",
                    f"{solver_node}.inputs[0]",
                    force=True
                )
                print(f"�� �������ӣ�{fk_node}.matrix -> {solver_node}.inputs[0]")
            except Exception as e:
                print(f"�� ��������ʧ��: {str(e)}")

        else:
            ########################
            # Ĭ��ģʽ��FKתMatrix #
            ########################
            # ��֤FK�ڵ�
            fk_node = f"FK{base_name}{side}"
            if not cmds.objExists(fk_node) or cmds.nodeType(fk_node) != "transform":
                print(f"�� Ĭ��ģʽ��FK�ڵ� {fk_node} �����ڻ����ʹ���")
                continue

            # ��֤��ǰFK����
            current_conn = cmds.listConnections(
                f"{solver_node}.inputs[0]",
                source=True,
                destination=False
            )
            if not current_conn or current_conn[0] != fk_node:
                print(f"�� Ĭ��ģʽ��{solver_node}.inputs[0] δ���� {fk_node}")
                continue

            # ����MatrixԴ�ڵ�
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
                print(f"�� Ĭ��ģʽ��{base_name}{side} δ�ҵ�MatrixԴ�ڵ�")
                continue

            # ִ��Matrix����
            try:
                cmds.connectAttr(
                    f"{matrix_source}.matrixSum",
                    f"{solver_node}.inputs[0]",
                    force=True  # ǿ�ƶϿ�ԭ������
                )
                print(f"�� Matrix���ӣ�{matrix_source}.matrixSum -> {solver_node}.inputs[0]")
            except Exception as e:
                print(f"�� Matrix����ʧ��: {str(e)}")



if __name__ == "__main__":
    #�μ����������ض���
    rebuildPartial(key='Partial')

    #��Ҫ�����ض���
    rebuild_bone_matrices(cmds.ls(sl =1))

    #Fk����תΪMatrix����
    [connect_solver_matrix(i, reverse_mode=0) for i in
     ['Ankle', 'Elbow', 'Hip', 'IndexFinger1', 'IndexFinger2', 'IndexFinger3', 'Knee', 'MiddleFinger1', 'MiddleFinger2',
      'MiddleFinger3', 'PinkyFinger1', 'PinkyFinger2', 'PinkyFinger3', 'RingFinger1', 'RingFinger2', 'RingFinger3',
      'Scapula', 'Shoulder', 'ThumbFinger2', 'ThumbFinger3', 'Wrist']]

