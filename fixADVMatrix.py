#!/usr/bin/env python
# _*_ coding:utf-8 _*_
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
import math
import pymel.core as pm
# ---------------------------
# 🌟 全局变量
# ---------------------------
limbs = ['Hip', 'Knee', 'Shoulder', 'Elbow']
sides = ['_R', '_L']
root_path = '|Group|DeformationSystem|root|Root_M'
alt_root  = '|Group|DeformationSystem|Root_M'
#Fk驱动转为Matrix驱动
def connect_solver_matrix( base_name, reverse_mode=False ):
    """Fk驱动与Matrix驱动切换连接
    [connect_solver_matrix(i, reverse_mode=True) for i in [...]  # 反向操作示例

    :param base_name: 基础关节名称 (e.g. 'Ankle')
    :param reverse_mode: False=FK转Matrix, True=Matrix转FK
    """
    global sides
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
def fix_matrix_connection(child_joint, new_parent):
    for a in ("translate", "rotate", "jointOrient"):
        try:
            cmds.setAttr('{}.{}'.format(child_joint, a), 0, 0, 0)
        except RuntimeError as e:
            return str(e)
    mult = (cmds.listConnections('{}.offsetParentMatrix'.format(child_joint),
                                 s=True, type='multMatrix') or [None])[0]
    if not mult:
        return 'no multMatrix'

    idx = max((int(a.split('[')[1].split(']')[0])
               for a in cmds.listAttr('{}.matrixIn'.format(mult), multi=True) or []),
              default=-1)
    if idx < 0:
        return 'no matrixIn'

    plug = '{}.matrixIn[{}]'.format(mult, idx)
    if (cmds.listConnections(plug, s=True) and
            cmds.attributeQuery('asParent', node=child_joint, exists=True)):
        for old in cmds.listConnections(plug, s=True, p=True) or []:
            cmds.disconnectAttr(old, plug)
        cmds.connectAttr('{}.worldInverseMatrix[0]'.format(new_parent), plug, f=True)
    return 1
def reparent_with_matrix_fix(child, new_parent):
    cur = cmds.listRelatives(child, p=True)
    if cur and cur[0] == new_parent:
        return 'is parent'
    cmds.parent(child, new_parent)
    fix_matrix_connection(child, new_parent)
# 重构骨骼层级
def rebuild_twist_hierarchy():
    global root_path, alt_root, limbs, sides

    if cmds.objExists(root_path):
        cmds.warning('// Twist hierarchy already rebuilt – skip')
        return 0
    if not cmds.objExists(alt_root):
        cmds.warning('// Alt Root_M not found – nothing to rebuild')
        return -1

    if not cmds.objExists('|Group|DeformationSystem|root'):
        cmds.createNode('joint', n='root', p='|Group|DeformationSystem')
    cmds.parent(alt_root, '|Group|DeformationSystem|root')

    def _record(j):
        if not cmds.attributeQuery('asParent', node=j, exists=True):
            cmds.addAttr(j, ln='asParent', dt='string')
        p = cmds.listRelatives(j, p=True)
        if p:
            cmds.setAttr('{}.asParent'.format(j), p[0].split('|')[-1], type='string')

    for limb in limbs:
        for side in sides:
            first = '{}Part1{}'.format(limb, side)
            if not cmds.objExists(first):
                continue
            tgt = cmds.listRelatives(first, p=True)[0]
            for i in range(1, 10):
                j = '{}Part{}{}'.format(limb, i, side)
                if not cmds.objExists(j):
                    continue
                _record(j)
                reparent_with_matrix_fix(j, tgt)
                c = cmds.listRelatives(j, c=True, type='joint') or []
                if c and 'Part' not in c[0]:
                    _record(c[0])
                    reparent_with_matrix_fix(c[0], tgt)

    cmds.warning('// Twist hierarchy rebuild completed')
    return 1
# 还原骨骼层级
def restore_twist_hierarchy():
    global root_path

    if not cmds.objExists(root_path):
        cmds.warning('// Not rebuilt before – nothing to restore')
        return 0

    # 关节短名映射
    short_map = {}
    for j in cmds.ls(type='joint'):
        short_map.setdefault(j.split('|')[-1], []).append(j)

    for j in cmds.ls('DeformationSystem', dag=True, type='joint'):
        if not cmds.attributeQuery('asParent', node=j, exists=True):
            continue
        p_short = cmds.getAttr('{}.asParent'.format(j)) or ''
        tgt = (short_map.get(p_short) or [None])[0]
        if not tgt:
            cmds.warning('⚠ missing parent {} for {}'.format(p_short, j))
            continue
        cur = (cmds.listRelatives(j, p=True) or [None])[0]
        if cur != tgt:
            try:
                cmds.parent(j, tgt)
                fix_matrix_connection(j, tgt)
            except RuntimeError as e:
                cmds.warning('restore {} failed: {}'.format(j, e))
                continue
        try:
            cmds.deleteAttr(j, at='asParent')
        except RuntimeError:
            pass

    if cmds.objExists(root_path):
        cmds.parent(root_path, '|Group|DeformationSystem')
        if cmds.objExists('|Group|DeformationSystem|root'):
            cmds.delete('|Group|DeformationSystem|root')

    cmds.warning('// Twist hierarchy restore completed')
    return 1
#增加骨骼缩放衰减
def decay_scale(joints):
    """
    功能：
    传入一组骨骼列表（targets + driver），自动建立缩放衰减网络，使用 offsetParentMatrix，并把节点放入 decayScaleSet。

    输入：
        joints: 列表，前面为 target joints，最后一个为控制骨骼。
    """

    if len(joints) < 2:
        pm.error("请至少传入2个骨骼：前面为目标骨骼，最后一个为控制骨骼。")

    # 转换 PyNode
    driver = pm.PyNode(joints[-1])
    print(f"Driver: {driver}")

    targets = [pm.PyNode(j) for j in joints[:-1]]
    num_targets = len(targets)

    # 检查或创建 set
    if not pm.objExists("decayScaleSet"):
        decay_set = pm.sets(name="decayScaleSet", empty=True)
    else:
        decay_set = pm.PyNode("decayScaleSet")

    # 构建 decomposeMatrix 节点，使用新缩写
    decomp_name = f"{driver}_DM"
    if pm.objExists(decomp_name):
        pm.delete(decomp_name)
    decomp_node = pm.createNode("decomposeMatrix", name=decomp_name)
    pm.connectAttr(driver.offsetParentMatrix, decomp_node.inputMatrix, force=True)
    pm.sets(decay_set, add=decomp_node)

    for i, tgt in enumerate(targets):
        # 计算 factor
        factor = float(i + 1) / (num_targets + 1)
        const_val = 1 - factor

        # 使用新命名规范
        md_name = f"{tgt}_MD"
        pma_name = f"{tgt}_PMA"

        # 删除已有节点
        for node_name in [md_name, pma_name]:
            if pm.objExists(node_name):
                pm.delete(node_name)

        # multiplyDivide 节点
        md_node = pm.createNode("multiplyDivide", name=md_name)
        pm.connectAttr(decomp_node.outputScale, md_node.input1, force=True)
        md_node.input2X.set(factor)
        md_node.input2Y.set(factor)
        md_node.input2Z.set(factor)
        pm.sets(decay_set, add=md_node)

        # plusMinusAverage 节点
        pma_node = pm.createNode("plusMinusAverage", name=pma_name)
        pm.connectAttr(md_node.output, pma_node.input3D[0], force=True)
        pma_node.input3D[1].set(const_val, const_val, const_val)
        pm.sets(decay_set, add=pma_node)

        # 分量连接
        for axis in ['X', 'Y', 'Z']:
            scale_attr = getattr(tgt, f'scale{axis}')
            output_attr = getattr(pma_node, f'output3D{axis.lower()}')

            if pm.connectionInfo(scale_attr, isDestination=True):
                src = pm.listConnections(scale_attr, s=True, d=False, plugs=True)
                if src:
                    pm.disconnectAttr(src[0], scale_attr)

            pm.connectAttr(output_attr, scale_attr, force=True)

    print("✅ 缩放衰减节点网络已建立完成，节点命名已统一缩写，所有节点已加入 decayScaleSet。")
#修改四肢原始缩放至FK世界矩阵
def fixFKScaleBlend(limbs, sides):
    """
    根据 limb + side 自动处理 ScaleBlend 节点的 scale 替换，断开 FK 连接，
    并重新创建 decomposeMatrix + multiplyDivide 链接。

    节点命名自动缩写（DM, MD）。
    """
    for limb in limbs:
        for side in sides:
            blend_name = f"ScaleBlend{limb}{side}"
            fk_name = f"FK{limb}{side}"

            if not pm.objExists(blend_name):
                print(f"⚠️ 节点 {blend_name} 不存在，跳过。")
                continue

            blend_node = pm.PyNode(blend_name)

            if blend_node.type() != "blendColors":
                print(f"⚠️ 节点 {blend_name} 不是 blendColors 类型，跳过。")
                continue

            if not pm.objExists(fk_name):
                print(f"⚠️ 节点 {fk_name} 不存在，跳过。")
                continue

            fk_node = pm.PyNode(fk_name)

            # 断开原有 FK.scaleX/Y/Z 与 blend color2R/G/B 的连接
            disconnect_pairs = [
                (fk_node.scaleX, blend_node.color2R),
                (fk_node.scaleY, blend_node.color2G),
                (fk_node.scaleZ, blend_node.color2B)
            ]

            for src_attr, tgt_attr in disconnect_pairs:
                if pm.connectionInfo(tgt_attr, isDestination=True):
                    conn_src = pm.listConnections(tgt_attr, s=True, d=False, plugs=True)
                    if conn_src and conn_src[0] == src_attr.name():
                        pm.disconnectAttr(src_attr, tgt_attr)
                        print(f"√ 已断开 {src_attr} -> {tgt_attr}")

            # 创建 decomposeMatrix 节点
            dm_name = f"{fk_name}_DM"
            if pm.objExists(dm_name):
                pm.delete(dm_name)
            dm_node = pm.createNode("decomposeMatrix", name=dm_name)
            pm.connectAttr(fk_node.worldMatrix[0], dm_node.inputMatrix, force=True)

            # 创建 multiplyDivide 节点
            md_name = f"{fk_name}_MD"
            if pm.objExists(md_name):
                pm.delete(md_name)
            md_node = pm.createNode("multiplyDivide", name=md_name)
            md_node.operation.set(2)
            # 连接
            pm.connectAttr(dm_node.outputScale, md_node.input1, force=True)
            pm.connectAttr("MainScaleMultiplyDivide.output", md_node.input2, force=True)
            pm.connectAttr(md_node.output, blend_node.color2, force=True)

            print(f"✅ 已为 {blend_name} 重建 scale 链接，并连接 decomposeMatrix + multiplyDivide 节点。")

    print("🎯 所有 limb+side 已处理完成。")
if __name__ == "__main__":
    import importlib
    import AdvancedSkeleton.fixADVMatrix as fixADVMatrix

    importlib.reload(fixADVMatrix)
    fixADVMatrix.rebuild_twist_hierarchy()

    for i in fixADVMatrix.limbs:
        for side in fixADVMatrix.sides:
            fixADVMatrix.fixFKScaleBlend(limbs, sides)
            joints = cmds.listRelatives(i + side, c=1)
            fixADVMatrix.decay_scale(joints)

    fixADVMatrix.restore_twist_hierarchy()
    #Fk驱动转为Matrix驱动
    [connect_solver_matrix(i, reverse_mode=0) for i in
     ['Ankle', 'Elbow', 'Hip', 'IndexFinger1', 'IndexFinger2', 'IndexFinger3', 'Knee', 'MiddleFinger1', 'MiddleFinger2',
      'MiddleFinger3', 'PinkyFinger1', 'PinkyFinger2', 'PinkyFinger3', 'RingFinger1', 'RingFinger2', 'RingFinger3',
      'Scapula', 'Shoulder', 'ThumbFinger2', 'ThumbFinger3', 'Wrist']]
