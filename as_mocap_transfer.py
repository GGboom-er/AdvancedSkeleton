# -*- coding: utf-8 -*-
import os
import math
from collections import OrderedDict
import maya.cmds as mc
import maya.mel as mel
class ASMoCapTransfer(object):
    DEFAULT_MAPPING_CONFIG = {
        "sideRight"     : "Right",
        "sideLeft"      : "Left",
        "sideMiddle"    : "",
        "sideBeforeName": True,
        "sideUnderScore": False
    }

    TPOSE_REQUIRED_CONTROLS = [
        "FKExtraHip", "FKExtraKnee", "FKExtraAnkle",
        "FKExtraShoulder", "FKExtraElbow", "FKExtraWrist",
        "IKExtraLeg", "PoleExtraLeg"
    ]

    TPOSE_FK_EXTRA_ROTATIONS = {
        "_R": {
            "FKExtraHip"     : [0, -90, -90],
            "FKExtraKnee"    : [0, -90, -90],
            "FKExtraAnkle"   : [90, 0, -90],
            "FKExtraShoulder": [-90, 0, 180],
            "FKExtraElbow"   : [-90, 0, 180],
            "FKExtraWrist"   : [-90, 0, 180],
        },
        "_L": {
            "FKExtraHip"     : [0, -90, 90],
            "FKExtraKnee"    : [0, -90, 90],
            "FKExtraAnkle"   : [-90, 0, 90],
            "FKExtraShoulder": [90, 0, 180],
            "FKExtraElbow"   : [90, 0, 180],
            "FKExtraWrist"   : [90, 0, 180],
        },
    }

    NODE_MAIN = "Main"
    NODE_DEFORMATION_SYSTEM = "DeformationSystem"
    NODE_FIT_SKELETON = "FitSkeleton"
    NODE_FACE_FIT_SKELETON = "FaceFitSkeleton"

    SET_CONTROL = "ControlSet"
    SET_FACE_CONTROL = "FaceControlSet"

    ATTR_HEIGHT = "height"
    ATTR_RUN = "run"
    ATTR_DISABLE_CONSTRAINTS = "disableConstraints"

    DEFAULT_CHAR_HEIGHT = 100.0
    POLE_SHIFT_RATIO = 0.1
    POLE_DISTANCE_RATIO = 0.3

    def __init__( self,
                  namespace="",
                  mocap_namespace="",
                  use_fk_extra=False,
                  maintain_offset=True,
                  baking_mocap=True,
                  mapping_config=None ):

        self.namespace = self._standardize_namespace(namespace)
        self.mocap_namespace = self._standardize_namespace(mocap_namespace)

        self.use_fk_extra = bool(use_fk_extra)
        self.use_ik_feet = True
        self.maintain_offset = bool(maintain_offset)
        self.baking_mocap = bool(baking_mocap)

        self.mapping_config = dict(self.DEFAULT_MAPPING_CONFIG)
        if mapping_config is not None:
            self.mapping_config.update(mapping_config)

        self.mapping = OrderedDict()
        self.as_root_base = None
        self.mocap_root_base = None

        self._constraints = []
        self._dest_objs = set()
        self._all_fk_mode = False
        self.constraint_mgr = None

        self.opm = self._check_opm()

    @staticmethod
    def _standardize_namespace( ns ):
        if not ns:
            return ""
        return ns if ns.endswith(":") else ns + ":"

    def _check_opm( self ):
        fit_skeleton = self.namespace + self.NODE_FIT_SKELETON
        if mc.objExists(fit_skeleton) and mc.attributeQuery("useOffsetParentMatrix", node=fit_skeleton, exists=True):
            return bool(mc.getAttr(fit_skeleton + ".useOffsetParentMatrix"))
        return False

    @staticmethod
    def _get_attr_safe( attr_path, default=0.0 ):
        try:
            obj = attr_path.split(".")[0]
            if mc.objExists(obj):
                return mc.getAttr(attr_path)
        except:
            pass
        return default

    def _get_rotation_from_opm( self, obj ):
        try:
            temp = "tempDecomposeMatrix"
            if mc.objExists(temp):
                mc.delete(temp)
            temp = mc.createNode("decomposeMatrix", name=temp)
            mc.setAttr(temp + ".inputRotateOrder", mc.getAttr(obj + ".rotateOrder"))
            mc.connectAttr(obj + ".offsetParentMatrix", temp + ".inputMatrix", force=True)
            rot = mc.getAttr(temp + ".outputRotate")[0]
            mc.delete(temp)
            return list(rot)
        except:
            return [0.0, 0.0, 0.0]

    @staticmethod
    def _split_side_base( short_name ):
        for suffix, side in [("_R", "R"), ("_L", "L"), ("_M", "M")]:
            if short_name.endswith(suffix):
                return short_name[:-len(suffix)], side

        for prefix, side in [("R_", "R"), ("L_", "L"), ("M_", "M")]:
            if short_name.startswith(prefix):
                return short_name[len(prefix):], side

        for suffix, side in [("_Right", "R"), ("_Left", "L"), ("_Middle", "M")]:
            if short_name.endswith(suffix):
                return short_name[:-len(suffix)], side

        for prefix, side in [("Right_", "R"), ("Left_", "L"), ("Middle_", "M")]:
            if short_name.startswith(prefix):
                return short_name[len(prefix):], side

        return short_name, "M"

    def _get_control_sets( self ):
        if self.namespace:
            ns = self.namespace.rstrip(":")
            sets = [
                ns + ":" + self.SET_CONTROL,
                ns + ":" + self.SET_FACE_CONTROL
            ]
            return [s for s in sets if mc.objExists(s)]

        all_sets = mc.ls(type="objectSet") or []
        return [s for s in all_sets if self.SET_CONTROL in s]

    @staticmethod
    def _exists_any( *names ):
        for name in names:
            if mc.objExists(name):
                return name
        return None

    def _reset_custom_attrs( self, ctrl ):
        try:
            attrs = mc.listAttr(ctrl, ud=True) or []
            for attr in attrs:
                plug = ctrl + "." + attr
                try:
                    attr_type = mc.getAttr(plug, type=True)
                    if attr_type == "string":
                        continue
                    if (not mc.getAttr(plug, settable=True)) or (not mc.getAttr(plug, keyable=True)):
                        continue

                    default = mc.attributeQuery(attr, node=ctrl, listDefault=True)
                    if not default:
                        continue

                    default_val = default[0]
                    if "PoleLeg" in ctrl and attr == "follow":
                        default_val = 10
                    mc.setAttr(plug, default_val)
                except RuntimeError:
                    pass
        except RuntimeError:
            pass

    def _reset_child_controls( self, parent_ctrl ):
        children = mc.listRelatives(parent_ctrl, ad=True, type="transform") or []
        ctrl_set = self.namespace + self.SET_CONTROL
        if not mc.objExists(ctrl_set):
            return

        for child in children:
            try:
                if not mc.sets(child, isMember=ctrl_set):
                    continue

                if not mc.getAttr(child + ".tx", lock=True):
                    mc.setAttr(child + ".t", 0, 0, 0, type="float3")
                if not mc.getAttr(child + ".rx", lock=True):
                    mc.setAttr(child + ".r", 0, 0, 0, type="float3")
            except RuntimeError:
                pass

    def _update_custom_orient( self, ctrl ):
        ctrl_short = ctrl.replace(self.namespace, "")
        custom_orient = self.namespace + "CustomOrient" + ctrl_short
        if not mc.objExists(custom_orient):
            return

        try:
            parent = mc.listRelatives(custom_orient, parent=True)[0]
            temp_node = mc.createNode("transform",
                                      name="asCustomOrientWithRotateOrder",
                                      parent=parent)
            mc.setAttr(temp_node + ".rotateOrder", mc.getAttr(ctrl + ".rotateOrder"))
            const = mc.orientConstraint(custom_orient, temp_node)[0]
            rot = mc.xform(temp_node, q=True, os=True, ro=True)
            mc.delete(const, temp_node)

            mc.xform(ctrl, r=True, os=True, ro=rot)

            sx = mc.getAttr(custom_orient + ".sx")
            if sx < 0:
                rot_ws = mc.xform(ctrl, q=True, ws=True, ro=True)
                mc.setAttr(custom_orient + ".s", 1, 1, 1, type="float3")
                mc.xform(ctrl, ws=True, ro=rot_ws)
                mc.setAttr(custom_orient + ".s", -1, -1, -1, type="float3")

                if mc.attributeQuery("worldorient", node=ctrl, exists=True):
                    if mc.getAttr(ctrl + ".worldorient"):
                        rot_co = mc.xform(custom_orient, q=True, os=True, ro=True)
                        mc.rotate(rot_co[0], rot_co[1], rot_co[2], ctrl, r=True, os=True)
        except RuntimeError:
            pass

    def _get_toes_joint( self, end_joint, side ):
        try:
            end_full = self.namespace + end_joint
            if not mc.objExists(end_full):
                return None

            children = mc.listRelatives(end_full, ad=True, type="joint", fullPath=True) or []
            for child in children:
                try:
                    if not mc.getAttr(child + ".drawLabel"):
                        continue
                    label = mc.getAttr(child + ".otherType")
                    if "Toes" in label and "QToes" not in label and "ToesEnd" not in label:
                        short = child.split("|")[-1].replace(self.namespace, "")
                        if mc.objExists(self.namespace + "FK" + short + side):
                            return short.replace(side, "")
                except RuntimeError:
                    continue
        except RuntimeError:
            pass
        return None

    def _calculate_shift_axis( self, fkx_middle, fkx_end, middle_joint, side ):
        temp_loc_shift1 = None
        temp_loc_shift2 = None

        try:
            temp_loc_shift1 = mc.spaceLocator()[0]
            mc.parent(temp_loc_shift1, fkx_middle, relative=True)
            temp_loc_shift2 = mc.spaceLocator()[0]
            mc.parent(temp_loc_shift2, fkx_end, relative=True)
            mc.parent(temp_loc_shift2, temp_loc_shift1)

            pos = mc.getAttr(temp_loc_shift2 + ".t")[0]
            pos_abs = [abs(p) for p in pos]

            pri_axis = "X"
            if pos_abs[1] > pos_abs[0] and pos_abs[1] > pos_abs[2]:
                pri_axis = "Y"
            elif pos_abs[2] > pos_abs[0] and pos_abs[2] > pos_abs[1]:
                pri_axis = "Z"

            middle_jnt_full = self.namespace + middle_joint + side
            if self.opm:
                jo = self._get_rotation_from_opm(middle_jnt_full)
            else:
                try:
                    jo = list(mc.getAttr(middle_jnt_full + ".jointOrient")[0])
                except:
                    jo = [0.0, 0.0, 0.0]

            jo_abs = [abs(j) for j in jo]
            max_jo = max(jo, key=abs) if any(jo_abs) else 0.0
            jo_fac = 15.0 / abs(max_jo) if abs(max_jo) > 0.001 else 1.0

            mc.setAttr(temp_loc_shift1 + ".r",
                       jo[0] * jo_fac, jo[1] * jo_fac, jo[2] * jo_fac,
                       type="float3")
            mc.parent(temp_loc_shift2, world=True)
            mc.setAttr(temp_loc_shift1 + ".r", 0, 0, 0, type="float3")
            mc.parent(temp_loc_shift2, temp_loc_shift1)
            mc.setAttr(temp_loc_shift2 + ".translate" + pri_axis, 0)

            pos = mc.getAttr(temp_loc_shift2 + ".t")[0]
            pos_abs = [abs(p) for p in pos]

            shift_axis = "Y"
            if pos_abs[0] > pos_abs[1] and pos_abs[0] > pos_abs[2]:
                shift_axis = "X"
            elif pos_abs[2] > pos_abs[0] and pos_abs[2] > pos_abs[1]:
                shift_axis = "Z"

            shift_val = mc.getAttr(temp_loc_shift2 + ".translate" + shift_axis)
            shift_polarity = -1 if shift_val < 0 else 1

            return shift_axis, shift_polarity

        except Exception as e:
            mc.warning("Failed to calculate shift axis: %s" % str(e))
            return "X", 1
        finally:
            if temp_loc_shift1 and mc.objExists(temp_loc_shift1):
                mc.delete(temp_loc_shift1)
            if temp_loc_shift2 and mc.objExists(temp_loc_shift2):
                mc.delete(temp_loc_shift2)

    def align_ik_to_fk( self, limb_type, side ):
        try:
            fkik_ctrl = self.namespace + "FKIK" + limb_type + side

            if not mc.objExists(fkik_ctrl):
                return False
            tempValue = mc.getAttr(fkik_ctrl + ".FKIKBlend")
            try:
                mc.setAttr(fkik_ctrl + '.FKIKBlend', 10)
            except:
                pass
            start_joint = mc.getAttr(fkik_ctrl + ".startJoint")
            middle_joint = mc.getAttr(fkik_ctrl + ".middleJoint")
            end_joint = mc.getAttr(fkik_ctrl + ".endJoint")

            pole_ctrl = self.namespace + "Pole" + limb_type + side
            is_spline = not mc.objExists(pole_ctrl)

            if is_spline:
                TF = self._align_fk_to_ik_spline(limb_type, side, start_joint, middle_joint, end_joint)
            else:
                TF = self._align_fk_to_ik_standard(limb_type, side, start_joint, middle_joint, end_joint)

            try:
                mc.setAttr(fkik_ctrl + '.FKIKBlend', tempValue)
            except:
                pass
            return TF
        except Exception as e:
            mc.warning("Failed to align IK to FK: %s" % str(e))
            return False

    def align_fk_to_ik( self, limb_type, side ):
        try:
            fkik_ctrl = self.namespace + "FKIK" + limb_type + side

            try:
                mc.setAttr(fkik_ctrl + '.FKIKBlend', 0)
            except:
                pass

            if not mc.objExists(fkik_ctrl):
                return False

            start_joint = mc.getAttr(fkik_ctrl + ".startJoint")
            middle_joint = mc.getAttr(fkik_ctrl + ".middleJoint")
            end_joint = mc.getAttr(fkik_ctrl + ".endJoint")

            pole_ctrl = self.namespace + "Pole" + limb_type + side
            is_spline = not mc.objExists(pole_ctrl)

            if is_spline:
                return self._align_ik_to_fk_spline(limb_type, side, start_joint, middle_joint, end_joint)
            else:
                return self._align_ik_to_fk_standard(limb_type, side, start_joint, middle_joint, end_joint)
        except Exception as e:
            mc.warning("Failed to align FK to IK: %s" % str(e))
            return False

    def _align_fk_to_ik_standard( self, limb_type, side, start_joint, middle_joint, end_joint ):
        ik_ctrl = self.namespace + "IK" + limb_type + side
        pole_ctrl = self.namespace + "Pole" + limb_type + side
        pole_extra = self.namespace + "PoleExtra" + limb_type + side

        if not mc.objExists(ik_ctrl) or not mc.objExists(pole_ctrl):
            return False

        self._reset_custom_attrs(ik_ctrl)
        self._reset_child_controls(ik_ctrl)

        if mc.attributeQuery("legAim", node=ik_ctrl, exists=True):
            mc.setAttr(ik_ctrl + ".legAim", 0)

        fk_end = self.namespace + "FK" + end_joint + side
        if mc.objExists(fk_end):
            pos = mc.xform(fk_end, q=True, ws=True, t=True)
            mc.xform(ik_ctrl, ws=True, t=pos)

        align_ik_to = self.namespace + "AlignIKTo" + end_joint + side
        if mc.objExists(align_ik_to):
            roo = mc.xform(ik_ctrl, q=True, roo=True)
            mc.xform(align_ik_to, preserve=True, roo=roo)
            rot = mc.xform(align_ik_to, q=True, ws=True, ro=True)
            mc.xform(ik_ctrl, ws=True, ro=rot)

        self._update_custom_orient(ik_ctrl)

        self._reset_custom_attrs(pole_ctrl)
        self._calculate_pole_position(limb_type, side, start_joint, middle_joint,
                                      end_joint, pole_ctrl, pole_extra)

        if limb_type == "Leg":
            self._align_toes(end_joint, side, ik_ctrl)

        self._sync_scale_and_stretch(ik_ctrl, end_joint, side, fk_end)

        return True

    def _calculate_pole_position( self, limb_type, side,
                                  start_joint, middle_joint, end_joint,
                                  pole_ctrl, pole_extra ):
        fkx_start = self.namespace + "FKX" + start_joint + side
        fkx_end = self.namespace + "FKX" + end_joint + side
        fkx_middle = self.namespace + "FKX" + middle_joint + side

        if not all(mc.objExists(n) for n in [fkx_start, fkx_end, fkx_middle]):
            return

        temp_loc1 = None
        temp_loc2 = None

        try:
            temp_loc1 = mc.spaceLocator()[0]
            const = mc.pointConstraint(fkx_start, fkx_end, temp_loc1)[0]

            const_attrs = mc.listAttr(const, ud=True) or []
            if len(const_attrs) >= 2:
                middle_len = self._get_attr_safe(self.namespace + middle_joint + ".tx")
                end_len = self._get_attr_safe(self.namespace + end_joint + ".tx")
                mc.setAttr(const + "." + const_attrs[0], abs(end_len))
                mc.setAttr(const + "." + const_attrs[1], abs(middle_len))
            mc.delete(const)

            shift_axis, shift_polarity = self._calculate_shift_axis(fkx_middle, fkx_end, middle_joint, side)

            char_size = self.DEFAULT_CHAR_HEIGHT
            main_ctrl = self.namespace + self.NODE_MAIN
            if mc.objExists(main_ctrl) and mc.attributeQuery(self.ATTR_HEIGHT, node=main_ctrl, exists=True):
                try:
                    char_size = mc.getAttr(main_ctrl + "." + self.ATTR_HEIGHT)
                except:
                    pass

            if self.baking_mocap:
                shift_dist = (char_size * self.POLE_SHIFT_RATIO) * shift_polarity
                mc.parent(temp_loc1, fkx_middle)
                mc.setAttr(temp_loc1 + ".t", 0, 0, 0, type="float3")
                current_val = mc.getAttr(temp_loc1 + ".translate" + shift_axis)
                mc.setAttr(temp_loc1 + ".translate" + shift_axis, current_val + shift_dist)
                mc.parent(temp_loc1, world=True)

            mc.aimConstraint(fkx_middle, temp_loc1, aimVector=(1, 0, 0))
            temp_loc2 = mc.spaceLocator()[0]
            mc.parent(temp_loc2, temp_loc1)
            mc.setAttr(temp_loc2 + ".t", 0, 0, 0, type="float3")

            pos_a = mc.xform(temp_loc2, q=True, ws=True, t=True)
            pos_b = mc.xform(self.namespace + middle_joint + side, q=True, ws=True, t=True)
            sampler_dist = math.sqrt(sum((pos_a[i] - pos_b[i]) ** 2 for i in range(3)))

            mc.setAttr(temp_loc2 + ".tx", (char_size * self.POLE_DISTANCE_RATIO) + sampler_dist)
            pole_pos = mc.xform(temp_loc2, q=True, ws=True, t=True)

            pole_target = pole_extra if mc.objExists(pole_extra) else pole_ctrl
            mc.xform(pole_target, ws=True, t=pole_pos)

            if pole_target == pole_extra:
                mc.xform(pole_target, ws=True, ro=[0, 0, 0])
                if mc.objExists(pole_ctrl):
                    try:
                        mc.xform(pole_ctrl, os=True, t=[0, 0, 0], ro=[0, 0, 0])
                    except:
                        pass
        finally:
            if temp_loc1 and mc.objExists(temp_loc1):
                mc.delete(temp_loc1)
            if temp_loc2 and mc.objExists(temp_loc2):
                mc.delete(temp_loc2)

    def _align_toes( self, end_joint, side, ik_ctrl ):
        toes_joint = self._get_toes_joint(end_joint, side)
        if not toes_joint:
            return

        ik_toes = self.namespace + "IKToes" + side
        align_ik_to_toes = self.namespace + "AlignIKToToes" + side

        if mc.objExists(ik_toes) and mc.objExists(align_ik_to_toes):
            if mc.attributeQuery("roll", node=ik_ctrl, exists=True):
                mc.setAttr(ik_ctrl + ".roll", 0)
            rot = mc.xform(align_ik_to_toes, q=True, ws=True, ro=True)
            mc.xform(ik_toes, ws=True, ro=rot)

    def _sync_scale_and_stretch( self, ik_ctrl, end_joint, side, fk_end ):
        end_scale = mc.getAttr(self.namespace + end_joint + side + ".s")[0]
        mc.setAttr(ik_ctrl + ".s", *end_scale, type="float3")

        if mc.attributeQuery("stretchy", node=ik_ctrl, exists=True):
            mc.setAttr(ik_ctrl + ".stretchy", 10)
        if mc.attributeQuery("volume", node=ik_ctrl, exists=True):
            mc.setAttr(ik_ctrl + ".volume", 10)

        if mc.objExists(fk_end):
            pos = mc.xform(fk_end, q=True, ws=True, t=True)
            mc.xform(ik_ctrl, ws=True, t=pos)
            scale = mc.xform(fk_end, q=True, os=True, r=True, s=True)
            mc.xform(ik_ctrl, os=True, a=True, s=scale)

    def _align_fk_to_ik_spline( self, limb_type, side, start_joint, middle_joint, end_joint ):
        try:
            chain_joints = mc.getAttr(
                self.namespace + "FKIK" + limb_type + side + ".listJoints"
            ).split(",")
        except:
            return False

        num_ik_ctrls = 0
        for i in range(1, 99):
            if not mc.objExists(self.namespace + "IK" + limb_type + str(i) + side):
                break
            num_ik_ctrls = i
        if num_ik_ctrls == 0:
            return False

        curve_cmd = "curve -n FK2IKCurve -d 1"
        for jnt in chain_joints:
            pos = mc.xform(self.namespace + jnt, q=True, ws=True, t=True)
            curve_cmd += " -p %f %f %f" % (pos[0], pos[1], pos[2])

        try:
            if mc.objExists("FK2IKCurve"):
                mc.delete("FK2IKCurve")
            mel.eval(curve_cmd)
            mc.rebuildCurve(
                "FK2IKCurve",
                ch=0, rpo=1, rt=0, end=1,
                kr=0, kcp=0, kep=1, kt=0,
                s=0, d=3, tol=0.01
            )
        except:
            return False

        if mc.objExists("tempPointOnCurveInfo"):
            mc.delete("tempPointOnCurveInfo")
        poc_info = mc.createNode("pointOnCurveInfo", name="tempPointOnCurveInfo")
        mc.setAttr(poc_info + ".turnOnPercentage", 1)
        mc.connectAttr("FK2IKCurve.worldSpace[0]", poc_info + ".inputCurve", f=True)

        for i in range(num_ik_ctrls):
            y = 1 if i == 0 else (num_ik_ctrls if i == 1 else i)
            u = (y - 1.0) / (num_ik_ctrls - 1.0)
            mc.setAttr(poc_info + ".parameter", u)
            pos = mc.getAttr(poc_info + ".position")[0]

            ik_hybrid = self.namespace + "IKhybrid" + limb_type + str(y) + side
            if mc.objExists(ik_hybrid):
                mc.xform(ik_hybrid, ws=True, t=pos)
                if i == 1:
                    mc.xform(ik_hybrid, os=True, t=[0, 0, 0], ro=[0, 0, 0])

        for i in range(num_ik_ctrls):
            y = 1 if i == 0 else (num_ik_ctrls if i == 1 else i)
            c = 0 if i == 0 else (len(chain_joints) - 1 if i == 1 else 0)
            u = (y - 1.0) / (num_ik_ctrls - 1.0)
            mc.setAttr(poc_info + ".parameter", u)
            pos = mc.getAttr(poc_info + ".position")[0]

            ik_ctrl = self.namespace + "IK" + limb_type + str(y) + side
            if mc.objExists(ik_ctrl):
                mc.xform(ik_ctrl, ws=True, t=pos)
                mc.xform(ik_ctrl, os=True, ro=[0, 0, 0])

                if i == 1 or (i == 0 and chain_joints[0] == "Root_M"):
                    align_ik_to = self.namespace + "AlignIKTo" + chain_joints[c]
                    if mc.objExists(align_ik_to):
                        roo = mc.xform(ik_ctrl, q=True, roo=True)
                        mc.xform(align_ik_to, preserve=True, roo=roo)
                        rot = mc.xform(align_ik_to, q=True, ws=True, ro=True)
                        mc.xform(ik_ctrl, ws=True, ro=rot)

        mc.delete("FK2IKCurve")
        if mc.objExists("tempPointOnCurveInfo"):
            mc.delete("tempPointOnCurveInfo")

        return True

    def _align_ik_to_fk_standard( self, limb_type, side, start_joint, middle_joint, end_joint ):
        fk_ctrls = [self.namespace + "FK" + j + side for j in [start_joint, middle_joint, end_joint]]
        ikx_joints = [self.namespace + "IKX" + j + side for j in [start_joint, middle_joint, end_joint]]

        if not all(mc.objExists(obj) for obj in fk_ctrls + ikx_joints):
            return False

        for fk_ctrl, ikx_joint in zip(fk_ctrls, ikx_joints):
            mc.xform(fk_ctrl, os=True, t=[0, 0, 0])
            rot = mc.xform(ikx_joint, q=True, ws=True, ro=True)
            mc.xform(fk_ctrl, ws=True, ro=rot)
            self._update_custom_orient(fk_ctrl)

        ik_ctrl = self.namespace + "IK" + limb_type + side
        if mc.objExists(ik_ctrl):
            scale = mc.xform(ik_ctrl, q=True, os=True, r=True, s=True)
            mc.xform(fk_ctrls[2], os=True, a=True, s=scale)

        if limb_type == "Leg":
            fk_toes = self.namespace + "FKToes" + side
            ikx_toes = self.namespace + "IKXToes" + side
            if mc.objExists(fk_toes) and mc.objExists(ikx_toes):
                mc.xform(fk_toes, os=True, t=[0, 0, 0])
                rot = mc.xform(ikx_toes, q=True, ws=True, ro=True)
                mc.xform(fk_toes, ws=True, ro=rot)
                self._update_custom_orient(fk_toes)

        for i, joint in enumerate([start_joint, middle_joint]):
            scale = mc.getAttr(self.namespace + joint + side + ".s")[0]
            mc.setAttr(fk_ctrls[i] + ".s", *scale, type="float3")

        middle_pos = mc.xform(ikx_joints[1], q=True, ws=True, t=True)
        mc.xform(fk_ctrls[1], ws=True, t=middle_pos)

        end_pos = mc.xform(ikx_joints[2], q=True, ws=True, t=True)
        mc.xform(fk_ctrls[2], ws=True, t=end_pos)

        return True

    def _align_ik_to_fk_spline( self, limb_type, side, start_joint, middle_joint, end_joint ):
        try:
            chain_joints = mc.getAttr(
                self.namespace + "FKIK" + limb_type + side + ".listJoints"
            ).split(",")
        except:
            return False

        for jnt in chain_joints:
            ikx_jnt = self.namespace + "IKX" + jnt
            fk_ctrl = self.namespace + "FK" + jnt

            if not mc.objExists(ikx_jnt) or not mc.objExists(fk_ctrl):
                continue

            try:
                in_set = mc.sets(fk_ctrl, isMember=self.namespace + self.SET_CONTROL)
                if not in_set:
                    continue
            except:
                continue

            pos = mc.xform(ikx_jnt, q=True, ws=True, t=True)
            rot = mc.xform(ikx_jnt, q=True, ws=True, ro=True)

            has_w_attrs = any(
                mc.attributeQuery("w" + str(y), node=fk_ctrl, exists=True)
                for y in range(99)
            )

            if not has_w_attrs:
                mc.xform(fk_ctrl, ws=True, t=pos, ro=rot)
            else:
                mc.xform(fk_ctrl, ws=True, t=pos)

        return True

    def reset_to_build_pose( self, ctrl_button_pressed=False, going_to_tpose=False ):
        control_sets = self._get_control_sets()
        if not control_sets:
            return False

        build_pose_name = "buildPose"
        for cs in control_sets:
            if self.SET_FACE_CONTROL in cs:
                build_pose_name = "faceBuildPose"
                break

        build_pose_node = self.namespace + build_pose_name
        if not mc.objExists(build_pose_node):
            return False

        set_attr_cmd = ""
        if mc.attributeQuery("udAttr", node=build_pose_node, exists=True):
            set_attr_cmd = mc.getAttr(build_pose_node + ".udAttr") or ""
        if mc.attributeQuery("udExtraAttr", node=build_pose_node, exists=True):
            set_attr_cmd += (mc.getAttr(build_pose_node + ".udExtraAttr") or "")

        if not set_attr_cmd:
            return False

        for cmd in [c.strip() for c in set_attr_cmd.split(";") if c.strip()]:
            mel_cmd = cmd
            if self.namespace:
                mel_cmd = self._inject_namespace_to_cmd(cmd)

            if ctrl_button_pressed and "Main." in mel_cmd:
                continue

            try:
                mel.eval(mel_cmd)
            except RuntimeError as e:
                mc.warning("Failed to execute build pose command: %s" % str(e))

        if going_to_tpose:
            self._execute_tpose_logic()

        self._reset_face_gui()
        self._execute_fit_skeleton_run(control_sets)

        return True

    def set_to_tpose( self, ctrl_button_pressed=False ):
        return self.reset_to_build_pose(ctrl_button_pressed=ctrl_button_pressed,
                                        going_to_tpose=True)

    def _execute_tpose_logic( self ):
        for side in ["_R", "_L"]:
            for ctrl in self.TPOSE_REQUIRED_CONTROLS:
                if not mc.objExists(self.namespace + ctrl + side):
                    return

        for side in ["_R", "_L"]:
            side_rot = self.TPOSE_FK_EXTRA_ROTATIONS.get(side, {})
            for ctrl, rot in side_rot.items():
                node = self.namespace + ctrl + side
                if not mc.objExists(node):
                    continue
                try:
                    mc.xform(node, ws=True, rotation=rot)
                except RuntimeError:
                    pass

            self.align_ik_to_fk("Leg", side)
            self.align_ik_to_fk("Arm", side)

            ik = self.namespace + "IKLeg" + side
            ikx = self.namespace + "IKExtraLeg" + side
            if mc.objExists(ik) and mc.objExists(ikx):
                try:
                    pos = mc.xform(ik, q=True, os=True, t=True)
                    rot = mc.xform(ik, q=True, os=True, ro=True)
                    mc.xform(ikx, os=True, t=pos, ro=rot)
                    mc.xform(ik, os=True, t=[0, 0, 0], ro=[0, 0, 0])
                except RuntimeError:
                    pass

    def _inject_namespace_to_cmd( self, cmd ):
        if not self.namespace:
            return cmd

        toks = cmd.split()
        if not toks:
            return cmd

        if toks[0].startswith("xform"):
            target_index = len(toks) - 1
        else:
            target_index = 1

        if target_index >= len(toks):
            return cmd

        new_tokens = []
        for i, t in enumerate(toks):
            if i == target_index:
                if ":" in t or t.startswith(self.namespace):
                    new_tokens.append(t)
                    continue

                if t.startswith('"') and t.endswith('"') and len(t) > 2:
                    inner = t[1:-1]
                    if ":" in inner or inner.startswith(self.namespace):
                        new_tokens.append(t)
                    else:
                        new_tokens.append('"%s%s"' % (self.namespace, inner))
                else:
                    new_tokens.append(self.namespace + t)
            else:
                new_tokens.append(t)

        return " ".join(new_tokens)

    def _reset_face_gui( self ):
        grp = self.namespace + "GRP_faceGUI"
        if not mc.objExists(grp):
            return

        nodes = mc.listRelatives(grp, ad=True, type="transform") or []
        for n in nodes:
            if "CTRL_faceGUI" in n or not n.startswith(self.namespace + "CTRL_"):
                continue

            for a in (mc.listAttr(n, k=True) or []):
                plug = n + "." + a
                try:
                    if not mc.getAttr(plug, settable=True):
                        continue
                except:
                    continue

                dv = 1 if (a.startswith("scale") or a == "visibility") else 0
                try:
                    mc.setAttr(plug, dv)
                except:
                    pass

    def _execute_fit_skeleton_run( self, control_sets ):
        fit = None
        for cs in control_sets:
            if (self.SET_CONTROL in cs) and ("Face" not in cs):
                fit = self.namespace + self.NODE_FIT_SKELETON
            elif self.SET_FACE_CONTROL in cs:
                fit = self.namespace + self.NODE_FACE_FIT_SKELETON

        if not fit or not mc.objExists(fit):
            return
        if not mc.attributeQuery(self.ATTR_RUN, node=fit, exists=True):
            return

        run = mc.getAttr(fit + "." + self.ATTR_RUN) or ""
        if not run:
            return

        for cmd in [c.strip() for c in run.split(";") if c.strip()]:
            mel_cmd = cmd
            if self.namespace:
                toks = cmd.split()
                if len(toks) >= 3:
                    obj_token = toks[1]
                    obj_attr = obj_token.strip('"')

                    if (":" in obj_attr) or obj_attr.startswith(self.namespace):
                        new_obj_attr = obj_attr
                    else:
                        new_obj_attr = self.namespace + obj_attr

                    if obj_token.startswith('"') and obj_token.endswith('"'):
                        toks[1] = '"%s"' % new_obj_attr
                    else:
                        toks[1] = new_obj_attr

                    mel_cmd = toks[0] + " " + toks[1] + " " + toks[2]

            try:
                mel.eval(mel_cmd)
            except RuntimeError as e:
                mc.warning("Failed to execute fit skeleton run command: %s" % str(e))

    def set_all_fk( self ):
        ctrl_set = self.namespace + self.SET_CONTROL
        if mc.objExists(ctrl_set):
            for m in (mc.sets(ctrl_set, q=True) or []):
                if mc.attributeQuery("FKIKBlend", node=m, exists=True):
                    try:
                        mc.setAttr(m + ".FKIKBlend", 0)
                    except RuntimeError:
                        pass

        hip = self.namespace + "HipSwinger_M"
        if mc.objExists(hip) and mc.attributeQuery("stabilize", node=hip, exists=True):
            try:
                mc.setAttr(hip + ".stabilize", 0)
            except RuntimeError:
                pass

        self._all_fk_mode = True
        self.use_ik_feet = False
        return True

    def set_all_ik( self ):
        ctrl_set = self.namespace + self.SET_CONTROL
        if mc.objExists(ctrl_set):
            for m in (mc.sets(ctrl_set, q=True) or []):
                if mc.attributeQuery("FKIKBlend", node=m, exists=True):
                    try:
                        mc.setAttr(m + ".FKIKBlend", 10)
                    except RuntimeError:
                        pass

        self._all_fk_mode = False
        self.use_ik_feet = True
        return True

    def load_mapping_file( self, mapping_file ):
        if not os.path.exists(mapping_file):
            raise IOError("Mapping file not found: %s" % mapping_file)

        with open(mapping_file, "r") as f:
            lines = [ln.strip() for ln in f.readlines()]

        section = 1
        first_mapping = True

        for ln in lines:
            if not ln or ln.startswith("//") or ln.startswith("#"):
                if not ln and section == 1:
                    section = 2
                continue

            if "=" not in ln:
                continue

            k, v = [x.strip() for x in ln.split("=", 1)]

            if section == 1:
                if k in self.mapping_config:
                    if k in ["sideBeforeName", "sideUnderScore"]:
                        self.mapping_config[k] = (v == "1")
                    else:
                        self.mapping_config[k] = v
            else:
                if first_mapping:
                    self.as_root_base = k
                    self.mocap_root_base = v
                    first_mapping = False

                self.mapping[k] = v

        if not self.as_root_base or not self.mocap_root_base:
            raise RuntimeError("Failed to load root bone mapping from first line of mapping file")

        return True

    def _mocap_name_from( self, mocap_base, side_char ):
        under = "_" if self.mapping_config["sideUnderScore"] else ""

        side_right = self.mapping_config["sideRight"]
        side_left = self.mapping_config["sideLeft"]
        side_middle = self.mapping_config["sideMiddle"]
        before = self.mapping_config["sideBeforeName"]

        if side_char == "R":
            side = side_right
        elif side_char == "L":
            side = side_left
        else:
            side = side_middle

        if side_char == "M" and not side:
            under = ""

        if not side:
            return mocap_base

        if before:
            return side + under + mocap_base
        else:
            return mocap_base + under + side

    def _ctrl_name_from( self, base, side_char ):
        side_suffix = "_" + side_char
        pref = "FKExtra" if self.use_fk_extra else "FK"
        return self.namespace + pref + base + side_suffix

    def _determine_sides_for_base( self, as_base ):
        sides = []

        for side in ["M", "R", "L"]:
            ctrl_name = self._ctrl_name_from(as_base, side)
            if mc.objExists(ctrl_name):
                sides.append(side)

        if not sides:
            sides = ["M"]

        return sides

    def _construct_mocap_node( self, mocap_base, side_char ):
        mocap_short = self._mocap_name_from(mocap_base, side_char)

        candidates = []
        if self.mocap_namespace:
            candidates.append(self.mocap_namespace + mocap_short)
        candidates.append(mocap_short)

        return self._exists_any(*candidates)

    def _construct_as_controller( self, as_base, side_char, effective_ik_feet ):
        dest = None

        if effective_ik_feet and as_base in {"Ankle", "Foot", "Toes", "Knee"}:
            if as_base in {"Ankle", "Foot"}:
                name = "IKExtraLeg" if self.use_fk_extra else "IKLeg"
                dest = self.namespace + name + "_" + side_char
            elif as_base == "Toes":
                name = "IKExtraToes" if self.use_fk_extra else "IKToes"
                dest = self.namespace + name + "_" + side_char
            elif as_base == "Knee":
                name = "PoleExtraLeg" if self.use_fk_extra else "PoleLeg"
                dest = self.namespace + name + "_" + side_char

        if not dest:
            dest = self._ctrl_name_from(as_base, side_char)

        if as_base == self.as_root_base and side_char == "M":
            if dest.endswith("FK" + self.as_root_base + "_M") and mc.objExists(self.namespace + "RootX_M"):
                dest = self.namespace + "RootX_M"
            if dest.endswith("FKExtra" + self.as_root_base + "_M") and mc.objExists(self.namespace + "RootExtraX_M"):
                dest = self.namespace + "RootExtraX_M"

        return dest

    def derive_probe_mocap_node( self ):
        if not self.as_root_base or not self.mocap_root_base:
            raise RuntimeError("Root bone mapping not loaded")

        sides_to_try = self._determine_sides_for_base(self.as_root_base)

        for side_char in sides_to_try:
            mocap_node = self._construct_mocap_node(self.mocap_root_base, side_char)
            if mocap_node:
                return mocap_node

        raise RuntimeError("Cannot resolve probe MoCap joint from root mapping: %s" % self.mocap_root_base)

    def get_anim_range_from_node( self, node,
                                  start_buffer=0,
                                  end_buffer=0,
                                  default_to_playback=True ):
        attrs = ("tx", "ty", "tz", "rx", "ry", "rz", "sx", "sy", "sz")
        times = set()

        for a in attrs:
            try:
                ks = mc.keyframe(node, at=a, q=True, timeChange=True)
                if ks:
                    times.update(round(k, 3) for k in ks)
            except:
                pass

        if not times:
            if default_to_playback:
                return (
                    mc.playbackOptions(q=True, min=True),
                    mc.playbackOptions(q=True, max=True)
                )
            raise RuntimeError("Node '%s' has no keyframes" % node)

        start = min(times)
        end = max(times)
        return (start - start_buffer, end + end_buffer)

    def get_mocap_animation_range( self, start_buffer=0, end_buffer=0 ):
        probe = self.derive_probe_mocap_node()
        return self.get_anim_range_from_node(
            probe,
            start_buffer=start_buffer,
            end_buffer=end_buffer,
            default_to_playback=True
        )

    def _init_constraint_manager( self ):
        self.constraint_mgr = "MoCapConstraints"
        if mc.objExists(self.constraint_mgr):
            try:
                mc.delete(self.constraint_mgr)
            except:
                pass

        self.constraint_mgr = mc.createNode("transform", name="MoCapConstraints")
        if not mc.attributeQuery(self.ATTR_DISABLE_CONSTRAINTS, n=self.constraint_mgr, exists=True):
            mc.addAttr(self.constraint_mgr,
                       ln=self.ATTR_DISABLE_CONSTRAINTS,
                       at="bool",
                       k=True)

    def validate_mapping( self ):
        missing_mappings = []

        for as_base, mocap_base in self.mapping.items():
            sides = self._determine_sides_for_base(as_base)

            for side_char in sides:
                as_ctrl = self._construct_as_controller(as_base, side_char, self.use_ik_feet)
                mocap_node = self._construct_mocap_node(mocap_base, side_char)

                if as_ctrl and mc.objExists(as_ctrl):
                    if not mocap_node:
                        mocap_short = self._mocap_name_from(mocap_base, side_char)
                        missing_mappings.append((as_ctrl, mocap_short))

        is_valid = (len(missing_mappings) == 0)

        return is_valid, missing_mappings

    def setup_mocap_tpose_at_frame( self, tpose_frame ):
        if not self.mocap_root_base:
            mc.warning("Root bone not defined, cannot setup MoCap T-Pose")
            return False

        sides = self._determine_sides_for_base(self.as_root_base)
        root = None

        for side_char in sides:
            root = self._construct_mocap_node(self.mocap_root_base, side_char)
            if root:
                break

        if not root:
            mc.warning("Cannot find MoCap root bone: %s" % self.mocap_root_base)
            return False

        mc.currentTime(tpose_frame)

        jnts = mc.listRelatives(root, ad=True, type="joint") or []
        jnts.insert(0, root)

        for j in jnts:
            for a in ("rx", "ry", "rz"):
                try:
                    mc.setAttr("%s.%s" % (j, a), 0)
                    mc.setKeyframe(j, at=a, t=tpose_frame)
                except RuntimeError:
                    pass

        for a in ("tx", "tz"):
            try:
                mc.setAttr("%s.%s" % (root, a), 0)
                mc.setKeyframe(root, at=a, t=tpose_frame)
            except RuntimeError:
                pass

        return True

    def create_constraints( self, tpose_frame ):
        mc.currentTime(tpose_frame)

        self.cleanup_constraints()
        self._init_constraint_manager()

        self._constraints = []
        self._dest_objs = set()

        effective_ik_feet = self.use_ik_feet

        for as_base, mocap_base in self.mapping.items():
            sides = self._determine_sides_for_base(as_base)

            for side_char in sides:
                dest = self._construct_as_controller(as_base, side_char, effective_ik_feet)
                if not dest or not mc.objExists(dest):
                    continue

                target = self._construct_mocap_node(mocap_base, side_char)
                if not target:
                    continue

                try:
                    if as_base == self.as_root_base and side_char == "M":
                        oc = mc.orientConstraint(target, dest, mo=self.maintain_offset)[0]
                        pc = mc.pointConstraint(target, dest, mo=self.maintain_offset)[0]

                        for c in (oc, pc):
                            mc.connectAttr(self.constraint_mgr + "." + self.ATTR_DISABLE_CONSTRAINTS,
                                           c + ".nodeState", f=True)
                            mc.parent(c, self.constraint_mgr)

                        self._constraints.extend([oc, pc])
                    else:
                        if effective_ik_feet and as_base in {"Ankle", "Foot", "Toes"}:
                            c = mc.parentConstraint(target, dest, mo=self.maintain_offset)[0]
                        elif effective_ik_feet and as_base == "Knee":
                            c = mc.pointConstraint(target, dest, mo=self.maintain_offset)[0]
                        else:
                            c = mc.orientConstraint(target, dest, mo=self.maintain_offset)[0]

                        mc.connectAttr(self.constraint_mgr + "." + self.ATTR_DISABLE_CONSTRAINTS,
                                       c + ".nodeState", f=True)
                        mc.parent(c, self.constraint_mgr)

                        self._constraints.append(c)

                    self._dest_objs.add(dest)

                except Exception as e:
                    mc.warning("Failed to create constraint for %s -> %s: %s" % (target, dest, str(e)))

        if not self._dest_objs:
            mc.warning("No constraints were created successfully!")
            return False

        return True

    def bake_animation( self, start_frame, end_frame ):
        if not self._dest_objs:
            raise RuntimeError("No constrained controllers to bake")

        objs = sorted(self._dest_objs)

        mc.bakeResults(
            objs,
            time=(start_frame, end_frame),
            sampleBy=1,
            oversamplingRate=1,
            disableImplicitControl=True,
            preserveOutsideKeys=False,
            sparseAnimCurveBake=False,
            removeBakedAttributeFromLayer=False,
            bakeOnOverrideLayer=False,
            minimizeRotation=True,
            controlPoints=False,
            shape=False
        )

        try:
            mc.delete(objs, staticChannels=True)
        except:
            pass

        return True

    def cleanup_constraints( self ):
        if self.constraint_mgr and mc.objExists(self.constraint_mgr):
            try:
                mc.delete(self.constraint_mgr)
            except:
                pass

        self.constraint_mgr = None
        self._constraints = []
        self._dest_objs = set()

        return True
def quick_transfer( namespace="",
                    mocap_namespace="mixamorig:",
                    mapping_file=r"Y:\GGbommer\scripts\AdvancedSkeleton\AdvancedSkeletonFiles\moCapMatchers\Mixamo.txt",
                    reset_pose=True,
                    set_all_fk=True,
                    use_fk_extra=False,
                    maintain_offset=False,
                    start_buffer=0,
                    end_buffer=0,
                    cleanup=True,
                    mocap_tpose=False ):
    tr = ASMoCapTransfer(
        namespace=namespace,
        mocap_namespace=mocap_namespace,
        use_fk_extra=use_fk_extra,
        maintain_offset=maintain_offset
    )

    tr.load_mapping_file(mapping_file)

    is_valid, missing_mappings = tr.validate_mapping()

    if not is_valid:
        error_msg = "Mapping validation failed!\n"
        if missing_mappings:
            error_msg += "\nMissing MoCap nodes for AS controllers:\n"
            for as_ctrl, mocap_bone in missing_mappings[:20]:
                error_msg += "  AS Controller: %s -> MoCap Bone: %s (NOT FOUND)\n" % (as_ctrl, mocap_bone)
            if len(missing_mappings) > 20:
                error_msg += "  ... and %d more\n" % (len(missing_mappings) - 20)

        mc.warning(error_msg)
        raise RuntimeError(error_msg)

    if reset_pose:
        tr.set_to_tpose()

    if set_all_fk:
        tr.set_all_fk()
    else:
        tr.set_all_ik()

    start_frame, end_frame = tr.get_mocap_animation_range(
        start_buffer=start_buffer,
        end_buffer=end_buffer
    )

    tpose_frame = start_frame - 1
    if mocap_tpose:
        tr.setup_mocap_tpose_at_frame(tpose_frame)

    tr.create_constraints(tpose_frame)
    tr.bake_animation(start_frame, end_frame)

    if cleanup:
        tr.cleanup_constraints()

    return start_frame, end_frame


if __name__ == "__main__":
    quick_transfer(
        namespace="",
        mocap_namespace="",
        mapping_file=r"Y:\GGbommer\scripts\AdvancedSkeleton\AdvancedSkeletonFiles\moCapMatchers\HXQ.txt",
        reset_pose=True,
        set_all_fk=True,
        use_fk_extra=False,
        maintain_offset=1,
        start_buffer=0,
        end_buffer=0,
        cleanup=1,
        mocap_tpose=False
    )