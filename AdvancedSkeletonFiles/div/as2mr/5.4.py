# AdvancedSkeleton To ModularRig
#	Copyright (C) Animation Studios
# email: support@animationstudios.com.au
# exported using AdvancedSkeleton version:x.xx
import unreal
import os
import re

engineVersion = unreal.SystemLibrary.get_engine_version()
asExportVersion = 'x.xx'
asExportTemplate = '4x'
asVariables = True
print ('AdvancedSkeleton To ModularControlRig (Unreal:'+engineVersion+') (AsExport:'+str(asExportVersion)+') (Template:'+asExportTemplate+')')
utilityBase = unreal.GlobalEditorUtilityBase.get_default_object()
selectedAssets = utilityBase.get_selected_assets()
if len(selectedAssets)<1:
	raise Exception('Nothing selected, you must select a ControlRig')
selectedAsset = selectedAssets[0]
if selectedAsset.get_class().get_name() != 'ControlRigBlueprint':
	raise Exception('Selected object is not a ControlRigBlueprint, you must select a ControlRigBlueprint')
blueprint = selectedAsset
RigGraphDisplaySettings = blueprint.get_editor_property('rig_graph_display_settings')
RigGraphDisplaySettings.set_editor_property('node_run_limit',256)
library = blueprint.get_local_function_library()
library_controller = blueprint.get_controller(library)
hierarchy = blueprint.hierarchy
hierarchy_controller = hierarchy.get_controller()
ModularRigController = blueprint.get_modular_rig_controller()
PreviousEndPlug = 'RigUnit_BeginExecution.ExecuteContext'

def asObjExists (obj):
	RigElementKeys = hierarchy.get_all_keys()
	LocObject = None
	for key in RigElementKeys:
		if key.name == obj:
			return True
	return False

def asGetKeyFromName (name):
	all_keys = hierarchy.get_all_keys(traverse = True)
	for key in all_keys:
		if key.name == name:
			return key
	return ''

def asConnect (target_key_name, connector_key_name):
	connector_key = asGetKeyFromName (connector_key_name)
	target_key = asGetKeyFromName (target_key_name)
	if (connector_key==""):
			print ("No connector_key_name:\""+connector_key_name+"\", skipping")
			return
	if (target_key_name==""):
			print ("No target_key_name:\""+target_key_name+"\", skipping")
			return
	ModularRigController.connect_connector_to_element (connector_key, target_key)

def asAddFace ():
	global PreviousEndPlug
	pathName = selectedAsset.get_path_name()
	dirName = os.path.dirname(pathName)
	asFace = unreal.ControlRigBlueprintFactory.create_new_control_rig_asset(dirName+'/asFace')
	asFace.set_auto_vm_recompile(False)

	hierarchy2 = asFace.hierarchy
	hierarchy2_controller = hierarchy2.get_controller()
	rigVMModel = asFace.get_controller_by_name('RigVMModel')

	rigVMModel.add_unit_node_from_struct_path('/Script/ControlRig.RigUnit_BeginExecution', 'Execute', unreal.Vector2D(0, 0), 'RigUnit_BeginExecution')

	# Create FacePanel
	hierarchy2_controller.add_null('ctrlBoxOffset', '', unreal.Transform(location=[0,0,9],rotation=[0,0,0],scale=[1,1,1]))
	control_settings_ctrl_box = unreal.RigControlSettings()
	control_settings_ctrl_box.animation_type = unreal.RigControlAnimationType.ANIMATION_CONTROL
	control_settings_ctrl_box.control_type = unreal.RigControlType.VECTOR2D
	control_settings_ctrl_box.display_name = 'None'
	control_settings_ctrl_box.draw_limits = True
	control_settings_ctrl_box.shape_color = unreal.LinearColor(1, 1, 0, 1)
	control_settings_ctrl_box.shape_name = 'Square_Thick'
	control_settings_ctrl_box.shape_visible = True
	control_settings_ctrl_box.is_transient_control = False
	control_settings_ctrl_box.limit_enabled = [unreal.RigControlLimitEnabled(False, False), unreal.RigControlLimitEnabled(False, False)]
	control_settings_ctrl_box.minimum_value = unreal.RigHierarchy.make_control_value_from_vector2d(unreal.Vector2D(0, 0))
	control_settings_ctrl_box.maximum_value = unreal.RigHierarchy.make_control_value_from_vector2d(unreal.Vector2D(0, 0))
	control_settings_ctrl_box.primary_axis = unreal.RigControlAxis.Y
	hierarchy2_controller.add_control('ctrlBox', unreal.RigElementKey(type=unreal.RigElementType.NULL, name='ctrlBoxOffset'), control_settings_ctrl_box, unreal.RigHierarchy.make_control_value_from_vector2d(unreal.Vector2D(0, 0)))
	hierarchy2.set_control_shape_transform(unreal.RigElementKey(type=unreal.RigElementType.CONTROL, name='ctrlBox'), unreal.Transform(location=[0,0,0],rotation=[0,0,90],scale=[0.6,2,1]), True)

	# Place FacePanel
	rigVMModel.add_unit_node_from_struct_path('/Script/ControlRig.RigUnit_Item', 'Execute', unreal.Vector2D(-848, 384), 'Item')
	rigVMModel.set_pin_default_value('Item.Item', '(Type=Connector,Name="Root")')
	rigVMModel.set_pin_expansion('Item.Item', True)
	rigVMModel.add_unit_node_from_struct_path('/Script/ControlRig.RigUnit_GetTransform', 'Execute', unreal.Vector2D(-496, 368), 'GetTransform')
	rigVMModel.set_pin_default_value('GetTransform.Item', '(Type=Bone,Name="None")')
	rigVMModel.set_pin_expansion('GetTransform.Item', True)
	rigVMModel.set_pin_default_value('GetTransform.Space', 'GlobalSpace')
	rigVMModel.set_pin_default_value('GetTransform.bInitial', 'False')
	rigVMModel.add_template_node('Set Transform::Execute(in Item,in Space,in bInitial,in Value,in Weight,in bPropagateToChildren)', unreal.Vector2D(256, -32), 'Set Transform')
	rigVMModel.resolve_wild_card_pin('Set Transform.Value', 'FTransform', '/Script/CoreUObject.Transform')
	rigVMModel.set_pin_default_value('Set Transform.Item', '(Type=Null,Name="ctrlBoxOffset")')
	rigVMModel.set_pin_expansion('Set Transform.Item', True)
	rigVMModel.set_pin_default_value('Set Transform.Space', 'GlobalSpace')
	rigVMModel.set_pin_default_value('Set Transform.bInitial', 'False')
	rigVMModel.set_pin_default_value('Set Transform.Value', '(Rotation=(X=0,Y=0,Z=0,W=1),Translation=(X=0,Y=0,Z=0),Scale3D=(X=1,Y=1,Z=1))')
	rigVMModel.set_pin_expansion('Set Transform.Value', True)
	rigVMModel.set_pin_expansion('Set Transform.Value.Rotation', False)
	rigVMModel.set_pin_expansion('Set Transform.Value.Translation', False)
	rigVMModel.set_pin_expansion('Set Transform.Value.Scale3D', False)
	rigVMModel.set_pin_default_value('Set Transform.Weight', '1')
	rigVMModel.set_pin_default_value('Set Transform.bPropagateToChildren', 'True')
	rigVMModel.add_unit_node_from_struct_path('/Script/RigVM.RigVMFunction_MathTransformMake', 'Execute', unreal.Vector2D(-480, 32), 'RigVMFunction_MathTransformMake')
	rigVMModel.set_pin_default_value('RigVMFunction_MathTransformMake.Translation', '(X=0,Y=0,Z=-30)')
	rigVMModel.set_pin_expansion('RigVMFunction_MathTransformMake.Translation', True)
	rigVMModel.set_pin_default_value('RigVMFunction_MathTransformMake.Rotation', '(X=-0,Y=0.707107,Z=0,W=0.707107)')
	rigVMModel.set_pin_expansion('RigVMFunction_MathTransformMake.Rotation', False)
	rigVMModel.set_pin_default_value('RigVMFunction_MathTransformMake.Scale', '(X=2.000000,Y=2.000000,Z=2.000000)')
	rigVMModel.set_pin_expansion('RigVMFunction_MathTransformMake.Scale', True)
	rigVMModel.add_template_node('Multiply::Execute(in A,in B,out Result)', unreal.Vector2D(-176, 128), 'Multiply')
	rigVMModel.resolve_wild_card_pin('Multiply.A', 'FTransform', '/Script/CoreUObject.Transform')
	rigVMModel.set_pin_default_value('Multiply.A', '(Rotation=(X=0,Y=0,Z=0,W=1),Translation=(X=0,Y=0,Z=0),Scale3D=(X=1,Y=1,Z=1))')
	rigVMModel.set_pin_expansion('Multiply.A', False)
	rigVMModel.set_pin_expansion('Multiply.A.Rotation', False)
	rigVMModel.set_pin_expansion('Multiply.A.Translation', False)
	rigVMModel.set_pin_expansion('Multiply.A.Scale3D', False)
	rigVMModel.set_pin_default_value('Multiply.B', '(Rotation=(X=0,Y=0,Z=0,W=1),Translation=(X=0,Y=0,Z=0),Scale3D=(X=1,Y=1,Z=1))')
	rigVMModel.set_pin_expansion('Multiply.B', False)
	rigVMModel.set_pin_expansion('Multiply.B.Rotation', False)
	rigVMModel.set_pin_expansion('Multiply.B.Translation', False)
	rigVMModel.set_pin_expansion('Multiply.B.Scale3D', False)
	rigVMModel.add_link('Item.Item', 'GetTransform.Item')
	rigVMModel.add_link(PreviousEndPlug, 'Set Transform.ExecuteContext')
	PreviousEndPlug='Set Transform.ExecuteContext'
	rigVMModel.add_link('RigVMFunction_MathTransformMake.Result', 'Multiply.A')
	rigVMModel.add_link('GetTransform.Transform', 'Multiply.B')
	rigVMModel.add_link('Multiply.Result', 'Set Transform.Value')

	ctrls=["ctrlBrow_R","ctrlBrow_L","ctrlEye_R","ctrlEye_L","ctrlCheek_R","ctrlCheek_L","ctrlNose_R","ctrlNose_L","ctrlLips_M","ctrlMouth_M","ctrlMouthCorner_R","ctrlMouthCorner_L","ctrlPhonemes_M","ctrlEmotions_M"]
	ctrlXs=[-1.5,1.5, -1.5,1.5, -1.5,1.5, -1.5,1.5, 0,0, -1.5,1.5, -1.5,1.5, -1.5,1.5]
	ctrlYs=[8.0,8.0, 5.5,5.5, 3,3, 1.5,1.5, 0.0, -1.5, -4.0,-4.0, -6.5,-6.5]
	bsXps=["brow_innerRaiser_R","brow_innerRaiser_L","eye_left_R","eye_left_L","cheek_out_R","cheek_out_L","nose_wide_R","nose_wide_L","lip_left_M","mouth_wide_M","mouth_wide_R","mouth_wide_L","",""]
	bsXns=["brow_innerlower_R","brow_innerlower_L","eye_right_R","eye_right_L","cheek_in_R","cheek_in_L","nose_narrow_R","nose_narrow_L","lip_right_M","mouth_narrow_M","mouth_narrow_R","mouth_narrow_L","",""]
	bsYps=["brow_raiser_R","brow_raiser_L","eye_up_R","eye_up_L","cheek_raiser_R","cheek_raiser_L","nose_raiser_R","nose_raiser_L","lip_up_M","mouth_close","mouth_raiser_R","mouth_raiser_L","",""]
	bsYns=["brow_lower_R","brow_lower_L","eye_down_R","eye_down_L","","","","","lip_down_M","mouth_open_M","mouth_lower_R","mouth_lower_L","",""]


#	bss=["brow_innerRaiser_R","brow_innerlower_R","brow_raiser_R","brow_lower_R","brow_squeeze_R","brow_outerUpDown_R","brow_innerRaiser_L","brow_innerlower_L","brow_raiser_L","brow_lower_L","brow_squeeze_L","brow_outerUpDown_L","eye_left_R","eye_right_R","eye_up_R","eye_down_R","blink_R","squint_R","eye_left_L","eye_right_L","eye_up_L","eye_down_L","blink_L","squint_L","cheek_out_R","cheek_in_R","cheek_raiser_R","cheek_out_L","cheek_in_L","cheek_raiser_L","nose_wide_R","nose_narrow_R","nose_raiser_R","nose_wide_L","nose_narrow_L","nose_raiser_L","lip_left_M","lip_right_M","lip_up_M","lip_down_M","lip_upperPressPos_M","lip_upperPressNeg_M","lip_lowerPressPos_M","lip_lowerPressNeg_M","lip_upperSqueezePos_M","lip_upperSqueezeNeg_M","lip_lowerSqueezePos_M","lip_lowerSqueezeNeg_M","lip_upperRollPos_M","lip_upperRollNeg_M","lip_lowerRollPos_M","lip_lowerRollNeg_M","lip_upperPuckerPos_M","lip_upperPuckerNeg_M","lip_lowerPuckerPos_M","lip_lowerPuckerNeg_M",
#"mouth_wide_M","mouth_narrow_M","mouth_close_M","mouth_open_M","mouth_jawForwardIn_M","mouth_jawForwardOut_M","mouth_jawSideIn_M","mouth_jawSideOut_M","mouth_wide_R","mouth_narrow_R","mouth_raiser_R","mouth_lower_R","mouth_wide_L","mouth_narrow_L","mouth_raiser_L","mouth_lower_L","aaa_M","eh_M","ahh_M","ohh_M","uuu_M","iee_M","rrr_M","www_M","sss_M","fff_M","tth_M","mbp_M","ssh_M","schwa_M","gk_M","lntd_M","happy_M","angry_M","sad_M","surprise_M","fear_M","disgust_M","contempt_M"]


	yPos=600
	#PreviousEndPlug = 'RigUnit_BeginExecution.ExecuteContext'
	for i in range(len(ctrls)):
		ctrl = ctrls[i]
		maxX = float (1)
		minX = float (-1)
		maxY = float (1)
		minY = float (-1)
		limitEnabled = True
		if re.search("ctrlCheek_", ctrl) or re.search("ctrlNose_", ctrl):
			minY = 0
		if ctrl=="ctrlMouth_M":
			maxY = 0
		if ctrl=="ctrlPhonemes_M" or ctrl=="ctrlEmotions_M":
			limitEnabled = False
			minX = 0
			minY = 0
		x = ctrl.split("_")
		if len(x)>1:
			baseName = x[0]
			side = '_'+x[1]
		attrExtras=[]
		bsExtras=[]
		if re.search("ctrlBrow_", ctrl):
			attrExtras=["squeeze"+side,"outerUpDown"+side] #Unreal seems to not work well with Left & Righ ctrl using same named ctrl
			bsExtras=["brow_squeeze"+side,"brow_outerUpDown"+side]
		if re.search("ctrlEye_", ctrl):
			attrExtras=["blink"+side,"squint"+side]
			bsExtras=["blink"+side,"squint"+side]
		if ctrl=="ctrlLips_M":
			attrExtras=["upperPress"+side,"lowerPress"+side,"upperSqueeze"+side,"lowerSqueeze"+side,"upperRoll"+side,"lowerRoll"+side,"upperPucker"+side,"lowerPucker"+side]
			bsExtras=["lip_upperPressPos_M","lip_lowerPressPos_M","lip_upperSqueezePos_M","lip_lowerSqueezePos_M","lip_upperRollPos_M","lip_lowerRollPos_M","lip_upperPuckerPos_M","lip_lowerPuckerPos_M"]
		if ctrl=="ctrlMouth_M":
			attrExtras=["jawForward"+side,"jawSide"+side]
			bsExtras=["mouth_jawForwardIn_M","mouth_jawSideIn_M"]
		if ctrl=="ctrlEmotions_M":
			attrExtras=["happy"+side,"angry"+side,"sad"+side,"surprise"+side,"fear"+side,"disgust"+side,"contempt"+side]
			bsExtras=["happy_M","angry_M","sad_M","surprise_M","fear_M","disgust_M","contempt_M"]



		xForm = unreal.Transform(location=[ctrlXs[i],0,ctrlYs[i]],rotation=[0,0,0],scale=[1,1,1])
		settings = unreal.RigControlSettings()
		settings.animation_type = unreal.RigControlAnimationType.ANIMATION_CONTROL
		settings.control_type = unreal.RigControlType.VECTOR2D
		settings.display_name = 'None'
		settings.draw_limits = limitEnabled
		settings.shape_color = unreal.LinearColor(1, 1, 0, 1)
		settings.shape_name = 'Default'
		settings.shape_visible = True
		settings.is_transient_control = False
		settings.limit_enabled = [unreal.RigControlLimitEnabled(limitEnabled, limitEnabled), unreal.RigControlLimitEnabled(limitEnabled, limitEnabled)]
		settings.minimum_value = unreal.RigHierarchy.make_control_value_from_vector2d(unreal.Vector2D(minX, minY))
		settings.maximum_value = unreal.RigHierarchy.make_control_value_from_vector2d(unreal.Vector2D(maxX, maxY))
		settings.primary_axis = unreal.RigControlAxis.Y
		hierarchy2_controller.add_control(ctrl, unreal.RigElementKey(type=unreal.RigElementType.CONTROL, name='ctrlBox'), settings, unreal.RigHierarchy.make_control_value_from_vector2d(unreal.Vector2D(0, 0)))
		hierarchy2.set_control_shape_transform(unreal.RigElementKey(type=unreal.RigElementType.CONTROL, name=ctrl), unreal.Transform(location=[0,0,0],rotation=[0,0,0],scale=[0.050000,0.050000,0.050000]), True)
		hierarchy2.set_control_offset_transform(unreal.RigElementKey(type=unreal.RigElementType.CONTROL, name=ctrl), xForm, True, True)
		hierarchy2.set_control_value(unreal.RigElementKey(type=unreal.RigElementType.CONTROL, name=ctrl), unreal.RigHierarchy.make_control_value_from_vector2d(unreal.Vector2D(0, 0)), unreal.RigControlValueType.CURRENT)

		#addAttr
		for y in range(len(attrExtras)):
			settings = unreal.RigControlSettings()
			settings.animation_type = unreal.RigControlAnimationType.ANIMATION_CHANNEL
			settings.control_type = unreal.RigControlType.FLOAT
			settings.display_name = attrExtras[y]
			settings.draw_limits = True
			settings.shape_color = unreal.LinearColor(1.000000, 0.000000, 0.000000, 1.000000)
			settings.shape_name = 'Default'
			settings.shape_visible = True
			settings.is_transient_control = False
			settings.limit_enabled = [unreal.RigControlLimitEnabled(False, False)]
			settings.minimum_value = unreal.RigHierarchy.make_control_value_from_float(0.000000)
			settings.maximum_value = unreal.RigHierarchy.make_control_value_from_float(1.000000)
			settings.primary_axis = unreal.RigControlAxis.X
			hierarchy2_controller.add_control(attrExtras[y], unreal.RigElementKey(type=unreal.RigElementType.CONTROL, name=ctrl), settings, unreal.RigHierarchy.make_control_value_from_float(0.000000))

		#attr loop
		for x in range(4+len(bsExtras)):
			if x==0:
				bs = bsXps[i]
				mult = '1'
				axis = 'X'
			if x==1:
				bs = bsXns[i]
				mult = '-1'
				axis = 'X'
			if x==2:
				bs = bsYps[i]
				mult = '1'
				axis = 'Y'
			if x==3:
				bs = bsYns[i]
				mult = '-1'
				axis = 'Y'
			if x>3:
				bs = bsExtras[x-4]
				mult = '1'
				axis = attrExtras[x-4]

			if bs=='':
				continue
			hierarchy2_controller.add_curve(bs, 0)
			scv = rigVMModel.add_unit_node_from_struct_path('/Script/ControlRig.RigUnit_SetCurveValue', 'Execute', unreal.Vector2D(200, yPos), 'SetCurveValue').get_name()
			rigVMModel.set_pin_default_value(scv+'.Curve', bs)
		#	rigVMModel.add_link(c+'.Result', scv+'.Value')
			rigVMModel.add_link(PreviousEndPlug, scv+'.ExecuteContext')
			PreviousEndPlug = scv+'.ExecuteContext'

			if x>3:
				gac = rigVMModel.add_template_node('GetAnimationChannel::Execute(out Value,in Control,in Channel,in bInitial)', unreal.Vector2D(-550, yPos), 'RigUnit_GetFloatAnimationChannel').get_name()
				rigVMModel.set_pin_default_value(gac+'.Control', ctrl)
				rigVMModel.set_pin_default_value(gac+'.Channel', axis)
				rigVMModel.resolve_wild_card_pin(gac+'.Value', 'float', 'None')
				rigVMModel.add_link(gac+'.Value', scv+'.Value')
			else:
				gcv = rigVMModel.add_unit_node_from_struct_path('/Script/ControlRig.RigUnit_GetControlVector2D', 'Execute', unreal.Vector2D(-550, yPos), 'GetControlVector2D').get_name()
				rigVMModel.set_pin_default_value(gcv+'.Control', ctrl)
				m = rigVMModel.add_template_node('Multiply::Execute(in A,in B,out Result)', unreal.Vector2D(-200, yPos), 'Multiply').get_name()
				c = rigVMModel.add_template_node('Clamp::Execute(in Value,in Minimum,in Maximum,out Result)', unreal.Vector2D(0, yPos), 'Clamp').get_name()
				rigVMModel.add_link(gcv+'.Vector.'+axis, m+'.A')
				rigVMModel.add_link(m+'.Result', c+'.Value')
				rigVMModel.set_pin_default_value(c+'.Maximum', '100')
				rigVMModel.add_link(c+'.Result', scv+'.Value')
				rigVMModel.set_pin_default_value(m+'.B', mult)

			#eyeRot
			if re.search("ctrlEye_", ctrl) and x==2:
				eyeMultX = rigVMModel.add_template_node('Multiply::Execute(in A,in B,out Result)', unreal.Vector2D(512, yPos    ), 'MultiplyX').get_name()
				eyeMultY = rigVMModel.add_template_node('Multiply::Execute(in A,in B,out Result)', unreal.Vector2D(512, yPos+100), 'MultiplyY').get_name()
				rigVMModel.resolve_wild_card_pin(eyeMultX+'.A', 'double', 'None')
				rigVMModel.resolve_wild_card_pin(eyeMultY+'.A', 'double', 'None')
				rigVMModel.set_pin_default_value(eyeMultX+'.A', '1.000000')
				rigVMModel.set_pin_default_value(eyeMultY+'.A', '1.000000')
				rigVMModel.set_pin_default_value(eyeMultX+'.B', '30.000000')
				rigVMModel.set_pin_default_value(eyeMultY+'.B', '30.000000')
				qfu = rigVMModel.add_unit_node_from_struct_path('/Script/RigVM.RigVMFunction_MathQuaternionFromEuler', 'Execute', unreal.Vector2D(705, yPos), 'RigVMFunction_MathQuaternionFromEuler').get_name()
				rigVMModel.set_pin_default_value(qfu+'.Euler', '(X=0.000000,Y=0.000000,Z=0.000000)')
				rigVMModel.set_pin_expansion(qfu+'.Euler', True)
				rigVMModel.set_pin_default_value(qfu+'.RotationOrder', 'ZYX')
				rigVMModel.add_link(gcv+'.Vector.X', eyeMultX+'.A')
				rigVMModel.add_link(gcv+'.Vector.Y', eyeMultY+'.A')
				rigVMModel.add_link(eyeMultX+'.Result', qfu+'.Euler.Y')
				rigVMModel.add_link(eyeMultY+'.Result', qfu+'.Euler.X')

				srt = rigVMModel.add_template_node('Set Relative Transform::Execute(in Child,in Parent,in bParentInitial,in Value,in Weight,in bPropagateToChildren)', unreal.Vector2D(1056, yPos), 'RigUnit_SetRelativeTransformForItem').get_name()
				rigVMModel.set_pin_default_value(srt+'.Child', '(Type=Bone,Name="unrealEyeJoint'+side+'")')
				rigVMModel.set_pin_default_value(srt+'.Parent', '(Type=Bone,Name="unrealEyeJoint'+side+'")')
				rigVMModel.resolve_wild_card_pin(srt+'.Value', 'FTransform', '/Script/CoreUObject.Transform')
				rigVMModel.add_link(qfu+'.Result', srt+'.Value.Rotation')

				rigVMModel.add_link(PreviousEndPlug, srt+'.ExecuteContext')
				PreviousEndPlug = srt+'.ExecuteContext'



			yPos = yPos+200



#	asFace.recompile_modular_rig()
	all_keys2 = hierarchy2.get_all_keys(traverse = True)
	asFace.convert_hierarchy_elements_to_spawner_nodes(hierarchy2,all_keys2)
	asFace.turn_into_control_rig_module()
	unreal.BlueprintEditorLibrary.compile_blueprint(asFace)

	#blueprint.recompile_modular_rig()
	unreal.EditorAssetLibrary.save_asset(dirName+'/asFace', only_if_is_dirty=False)
	module_class = unreal.load_class(None, (dirName+'/asFace.asFace_C'))
	ModularRigController.add_module (module_name='asFace', class_= module_class , parent_module_path='Root:Spine:Neck')

	unreal.BlueprintEditorLibrary.compile_blueprint(blueprint)




def main ():
	RigElementKeys = hierarchy.get_all_keys()

	target_key = asGetKeyFromName ('spine_01_socket')
	if not (target_key):

		#remove old sockets
		for key in RigElementKeys:
			x = re.search("_socket", str(key.name))
			if x:
				hierarchy_controller.remove_element(key)

		hierarchy_controller.add_socket('spine_01_socket', unreal.RigElementKey(type=unreal.RigElementType.BONE, name='spine_01'), unreal.Transform(location=[0,0,0],rotation=[-0,0,-0],scale=[1,1,1]), False, unreal.LinearColor(1, 1, 1, 1), '')
		hierarchy_controller.add_socket('neck_socket', unreal.RigElementKey(type=unreal.RigElementType.BONE, name='neck'), unreal.Transform(location=[0,0,0],rotation=[-0,0,-0],scale=[1,1,1]), False, unreal.LinearColor(1, 1, 1, 1), '')
		for side in ['_r','_l']:
			hierarchy_controller.add_socket('shoulder'+side+'_socket', unreal.RigElementKey(type=unreal.RigElementType.BONE, name='upperArm'+side), unreal.Transform(location=[0,0,0],rotation=[-0,0,-0],scale=[1,1,1]), False, unreal.LinearColor(1, 1, 1, 1), '')
			hierarchy_controller.add_socket('hand'+side+'_socket', unreal.RigElementKey(type=unreal.RigElementType.BONE, name='hand'+side), unreal.Transform(location=[0,0,0],rotation=[-0,0,-0],scale=[1,1,1]), False, unreal.LinearColor(1, 1, 1, 1), '')
			hierarchy_controller.add_socket('foot'+side+'_socket', unreal.RigElementKey(type=unreal.RigElementType.BONE, name='foot'+side), unreal.Transform(location=[0,0,0],rotation=[-0,0,-0],scale=[1,1,1]), False, unreal.LinearColor(1, 1, 1, 1), '')


		module_class = unreal.load_class(None, '/ControlRigModules/Modules/Spine.Spine_C')
		ModularRigController.add_module (module_name='Spine', class_= module_class , parent_module_path='Root')
#		ModularRigController.set_config_value_in_module('Root','Scale Root Controls Override','2')
#		ModularRigController.set_config_value_in_module('Root:Spine','Module Settings','Control Size=3')
#blueprint.add_member_variable('test6','ModuleSettings')
#vars = blueprint.get_member_variables ()
#x = getattr(vars[0], 'default_value')
#ModularRigController.bind_module_variable('Root:Spine','Module Settings','test3')

		module_class = unreal.load_class(None, '/ControlRigModules/Modules/Neck.Neck_C')
		ModularRigController.add_module (module_name='Neck', class_= module_class , parent_module_path='Root:Spine')

		for side in ['_r','_l']:
			module_class = unreal.load_class(None, '/ControlRigModules/Modules/Shoulder.Shoulder_C')
			ModularRigController.add_module (module_name='Shoulder'+side, class_= module_class , parent_module_path='Root:Spine')
			module_class = unreal.load_class(None, '/ControlRigModules/Modules/Arm.Arm_C')
			ModularRigController.add_module (module_name='Arm'+side, class_= module_class , parent_module_path='Root:Spine:Shoulder'+side)
			module_class = unreal.load_class(None, '/ControlRigModules/Modules/Leg.Leg_C')
			ModularRigController.add_module (module_name='Leg'+side, class_= module_class , parent_module_path='Root')
			module_class = unreal.load_class(None, '/ControlRigModules/Modules/Foot.Foot_C')
			ModularRigController.add_module (module_name='Foot'+side, class_= module_class , parent_module_path='Root:Leg'+side)

			for finger in ['index','middle','ring','pinky','thumb']:
				module_class = unreal.load_class(None, '/ControlRigModules/Modules/Finger.Finger_C')
				ModularRigController.add_module (module_name='Finger'+finger+side, class_= module_class , parent_module_path='Root:Spine:Shoulder'+side+':Arm'+side)

		if includeFaceRig:
			asAddFace ()

		print ('First run Complete, Now open the ControlRig, then run the python-script again.')
		return


	asConnect ('root' , 'Root:Root')
	asConnect ('spine_01_socket' , 'Root:Spine:Spine Primary')
	asConnect ('spine_05' , 'Root:Spine:Spine End Bone')
	asConnect ('neck_socket' , 'Root:Spine:Neck:Neck Primary')
	asConnect ('neck_01' , 'Root:Spine:Neck:Neck Start Bone')
	asConnect ('head' , 'Root:Spine:Neck:Head Bone')

	for side in ['_r','_l']:
		asConnect ('shoulder'+side+'_socket' , 'Root:Spine:Shoulder'+side+':Shoulder Primary')
		asConnect ('spine_05' , 'Root:Spine:Shoulder'+side+':Chest Bone')
		asConnect ('hand'+side+'_socket' , 'Root:Spine:Shoulder'+side+':Arm'+side+':Arm Primary')
		asConnect ('spine_01_socket' , 'Root:Leg'+side+':Leg Primary')
		asConnect ('thigh'+side , 'Root:Leg'+side+':Thigh Bone')
		asConnect ('foot'+side , 'Root:Leg'+side+':Foot Bone')
		#Knee Bone Connector added in Unreal 5.5
		asConnect ('calf'+side , 'Root:Leg'+side+':Knee Bone')
		asConnect ('foot'+side+'_socket' , 'Root:Leg'+side+':Foot'+side+':Foot Primary')
		asConnect ('ball'+side , 'Root:Leg'+side+':Foot'+side+':Ball Bone')
		asConnect ('foot'+side , 'Root:Leg'+side+':Foot'+side+':Foot Bone')

		for finger in ['index','middle','ring','pinky','thumb']:
			asConnect ('hand'+side+'_socket' , 'Root:Spine:Shoulder'+side+':Arm'+side+':Finger'+finger+side+':Finger Primary')
			if finger == 'thumb':
				asConnect (finger+'_01'+side , 'Root:Spine:Shoulder'+side+':Arm'+side+':Finger'+finger+side+':Start Bone')
			else:
				asConnect (finger+'_metacarpal'+side , 'Root:Spine:Shoulder'+side+':Arm'+side+':Finger'+finger+side+':Start Bone')
			asConnect (finger+'_03'+side , 'Root:Spine:Shoulder'+side+':Arm'+side+':Finger'+finger+side+':End Bone')

	if includeFaceRig:
		asConnect ('head' , 'Root:Spine:Neck:asFace:Root')


	print ('Second run complete.')

if __name__ == "__main__":
    main()