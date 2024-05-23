play_motion2:
  ros__parameters:
@[if end_effector == "pal-hey5"]@
    controllers: [arm_controller, head_controller, torso_controller, hand_controller]
@[end if]@
@[if end_effector in ["pal-gripper", "robotiq-2f-85", "robotiq-2f-140"]]@
    controllers: [arm_controller, head_controller, torso_controller, gripper_controller]
@[end if]@
@[if end_effector == "custom"]@
    controllers: [arm_controller, head_controller, torso_controller]
@[end if]@
@[if not has_arm]@
    controllers: [head_controller, torso_controller]
@[end if]
    motions:
@[if has_arm]@
      home:
        joints: [torso_lift_joint, arm_1_joint,
        arm_2_joint, arm_3_joint, arm_4_joint, arm_5_joint,
        arm_6_joint, arm_7_joint]
@[if end_effector in ["robotiq-2f-85", "robotiq-2f-140"]]@
        positions: [0.25, 0.20, 0.35, -0.20, 1.94, -1.57, 1.37, -1.58,
                    0.18, @[if end_effector == "robotiq-2f-140"]0.50@[else]0.20@[end if], -1.34, @[if end_effector == "robotiq-2f-140"]-0.48@[else]-0.20@[end if], 1.94, @[if end_effector == "robotiq-2f-140"]-1.49@[else]-1.57@[end if], 1.37, -1.58,
                    0.15, @[if end_effector == "robotiq-2f-140"]0.50@[else]0.20@[end if], -1.34, @[if end_effector == "robotiq-2f-140"]-0.48@[else]-0.20@[end if], 1.94, @[if end_effector == "robotiq-2f-140"]-1.49@[else]-1.57@[end if], 1.37, -1.58,
                    0.15, @[if end_effector == "robotiq-2f-140"]0.50@[else]0.20@[end if], -1.34, @[if end_effector == "robotiq-2f-140"]-0.48@[else]-0.20@[end if], 1.94, @[if end_effector == "robotiq-2f-140"]-1.49@[else]-1.57@[end if], 1.37, 0.0]
        times_from_start: [0.5, 4.0, 7.0, 9.0]
@[else]@
        positions: [0.25, 0.20, 0.35, -0.20, 1.94, -1.57, 1.37, 0.0,
                    0.18, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0,
                    0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]
        times_from_start: [0.5, 4.0, 7.0]
@[end if]@
        meta:
          name: Home
          usage: demo
          description: 'Go home'
@[if end_effector == "pal-gripper"]@

      close:
        joints: [gripper_left_finger_joint, gripper_right_finger_joint]
        positions: [0.0, 0.0]
        times_from_start: [0.5]
        meta:
          name: Close Gripper
          usage: demo
          description: 'Close Gripper'

      close_half:
        joints: [gripper_left_finger_joint, gripper_right_finger_joint]
        positions: [0.024, 0.024]
        times_from_start: [0.5]
        meta:
          name: Close Gripper Half
          usage: demo
          description: 'Close Gripper Halfway'

      open:
        joints: [gripper_left_finger_joint, gripper_right_finger_joint]
        positions: [0.044, 0.044]
        times_from_start: [0.5]
        meta:
          name: Open Gripper
          usage: demo
          description: 'Open Gripper'

      point:
        joints: [gripper_left_finger_joint, gripper_right_finger_joint]
        positions: [0.0, 0.0]
        times_from_start: [0.5]
        meta:
          name: Point Gripper Pose
          usage: demo
          description: 'Close Gripper to point to something'
@[end if]@
@[if end_effector == "pal-hey5"]@

      open:
        joints: [hand_thumb_joint, hand_index_joint, hand_mrl_joint]
        positions: [-1.0, -1.0, -1.0,
                    0.0, 0.0, 0.0]
        times_from_start: [0.1, 2.5]
        meta:
          name: Open Hand
          usage: demo
          description: 'Opens hand'

      close:
        joints: [hand_thumb_joint, hand_index_joint, hand_mrl_joint]
        positions: [2.37, 0.0, 0.0,
                    6.2, 6.8, 9.2]
        times_from_start: [0.1, 2.5]
        meta:
          name: Close Hand
          usage: demo
          description: 'Closes hand'

      close_half:
        joints: [hand_thumb_joint, hand_index_joint, hand_mrl_joint]
        positions: [3.2, 3.8, 4.6]
        times_from_start: [2.5]
        meta:
          name: Close Hand Half
          usage: demo
          description: 'Closes hand halfway'

      point:
        joints: [hand_thumb_joint, hand_index_joint, hand_mrl_joint]
        positions: [2.37, -1.0, 0.0,
                    6.2, 0.0, 9.2]
        times_from_start: [0.1, 1.5]
        meta:
          name: Pointing Hand
          usage: demo
          description: 'Pointing Hand'

      gun_hand:
        joints: [hand_thumb_joint, hand_index_joint, hand_mrl_joint]
        positions: [-1.0, -1.0, 0.0,
                    0.0, 0.0, 9.2]
        times_from_start: [0.1, 1.5]
        meta:
          name: Gun Hand
          usage: demo
          description: 'gun_hand'

      thumb_up_hand:
        joints: [hand_thumb_joint, hand_index_joint, hand_mrl_joint]
        positions: [-1.0, 0.0, 0.0,
                    0.0, 6.8, 9.2]
        times_from_start: [0.1, 1.5]
        meta:
          name: Thumb Up Hand
          usage: demo
          description: 'thumb_up_hand'

      pinch_hand:
        joints: [hand_thumb_joint, hand_index_joint, hand_mrl_joint]
        positions: [0.0, -1.0, -1.0,
                    3.37, 4.0, 0.0]
        times_from_start: [0.1, 1.5]
        meta:
          name: Pinch Hand
          usage: demo
          description: 'pinch_hand'
@[end if]@
@[if end_effector in ["robotiq-2f-85", "robotiq-2f-140"]]@

      close:
        joints: [gripper_finger_joint]
        positions: [@[if end_effector == "robotiq-2f-85"]0.8@[else]0.7@[end if]]
        times_from_start: [0.5]
        meta:
          name: Close Gripper
          usage: demo
          description: 'Closes gripper'

      close_half:
        joints: [gripper_finger_joint]
        positions: [@[if end_effector == "robotiq-2f-85"]0.4@[else]0.35@[end if]]
        times_from_start: [0.5]
        meta:
          name: Close Gripper Half
          usage: demo
          description: 'Closes gripper halfway'

      open:
        joints: [gripper_finger_joint]
        positions: [0.0]
        times_from_start: [0.5]
        meta:
          name: Open Gripper
          usage: demo
          description: 'Open gripper'

      point:
        joints: [gripper_finger_joint]
        positions: [@[if end_effector == "robotiq-2f-85"]0.8@[else]0.7@[end if]]
        times_from_start: [0.5]
        meta:
          name: Point
          usage: demo
          description: 'Closes gripper to point to something'
@[end if]@
@[if end_effector == "pal-hey5"]@

      offer:
        joints: ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'hand_thumb_joint', 'hand_index_joint', 'hand_mrl_joint']
        positions: [0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.62, -1.577, -0.001, -0.0, -0.001]
        times_from_start: [0.0]
        meta:
          name: Offer Hand
          usage: demo
          description: 'Offer hand'

      shake_hands:
        joints: ['hand_thumb_joint','hand_index_joint','hand_mrl_joint','torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        positions: [0.0, 0.0, 0.0, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    6.2, 6.8, 9.2, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    6.2, 6.8, 9.2, 0.296, 1.61, -0.93, -3.14, 1.40, -1.577, -0.2, -1.577,
                    6.2, 6.8, 9.2, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    6.2, 6.8, 9.2, 0.296, 1.61, -0.93, -3.14, 1.40, -1.577, -0.2, -1.577,
                    6.2, 6.8, 9.2, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    0.0, 0.0, 0.0, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577]
        times_from_start: [0.0, 5.0, 6.0, 7.0, 8.0, 9.0, 11.0]
        meta:
          name: Shake hands
          usage: demo
          description: 'shake_hands'
@[if ft_sensor == "schunk-ft"]@

      pick_from_floor:
        joints: ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'hand_thumb_joint', 'hand_index_joint', 'hand_mrl_joint']
        positions: [0.226, 0.308, -0.695, -0.968, 1.582,  1.965, 0.273, -1.101, 0.0, -0.001, -0.0,
                    0.12, 0.809, -1.197, -1.119, 0.322, 1.96, -0.849, 0.041, 0.0, -0.001, -0.0,
                    0.12, 0.809, -1.197, -1.119, 0.345, 1.96, -0.849, 0.041, 6.2, 6.77, 8.8,
                    0.158, 0.21, -1.153, -1.538, 2.26, 1.965, 0.394, -0.082, 6.2, 6.77, 8.8]
        times_from_start: [0.0, 4.0, 6.0, 9.0]
        meta:
          name: Pick from floor
          usage: demo
          description: 'Pick a shirt-like object from floor in front of the robot'
@[else]@

      pick_from_floor:
        joints: ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'hand_thumb_joint', 'hand_index_joint', 'hand_mrl_joint']
        positions: [0.226, 0.308, -0.695, -0.968, 1.582,  1.965, 0.273, -1.101, 0.0, -0.001, -0.0,
                    0.12, 0.809, -1.197, -1.119, 0.322, 1.96, -0.849, 0.041, 0.0, -0.001, -0.0,
                    0.12, 0.809, -1.197, -1.119, 0.345, 1.96, -0.849, 0.041, 6.2, 6.77, 8.8,
                    0.158, 0.21, -1.153, -1.538, 2.26, 1.965, 0.394, -0.082, 6.2, 6.77, 8.8]
        times_from_start: [0.0, 4.0, 6.0, 9.0]
        meta:
          name: Pick from floor
          usage: demo
          description: 'Pick a shirt-like object from floor in front of the robot'
@[end if]@
@[end if]@
@[if end_effector == "pal-gripper"]@

      offer:
        joints: ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'gripper_right_finger_joint', 'gripper_left_finger_joint']
        positions: [0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.62, -1.577, 0.044, 0.044]
        times_from_start: [0.0]
        meta:
          name: Offer Gripper
          usage: demo
          description: 'Offer Gripper'

      shake_hands:
        joints: ['gripper_left_finger_joint', 'gripper_right_finger_joint', 'torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        positions: [0.044, 0.044, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    0.035, 0.035, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    0.035, 0.035, 0.296, 1.61, -0.93, -3.14, 1.40, -1.577, -0.2, -1.577,
                    0.035, 0.035, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    0.035, 0.035, 0.296, 1.61, -0.93, -3.14, 1.40, -1.577, -0.2, -1.577,
                    0.035, 0.035, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    0.044, 0.044, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577]
        times_from_start: [0.0, 5.0, 6.0, 7.0, 8.0, 9.0, 11.0]
        meta:
          name: Shake Hands
          usage: demo
          description: 'shake_hands'

      pick_from_floor:
        joints: ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'gripper_left_finger_joint', 'gripper_right_finger_joint']
        positions: [0.226, 0.308, -0.695, -0.968, 1.582,  1.965, 0.273, -1.101, 0.044, 0.044,
                    0.12, 0.809, -1.197, -1.119, 0.322, 1.96, -0.849, 0.041, 0.044, 0.044,
                    0.12, 0.809, -1.197, -1.119, 0.345, 1.96, -0.849, 0.041, 0.0, 0.0,
                    0.27, 0.21, -1.153, -1.538, 2.26, 1.965, 0.394, -0.082, 0.0, 0.0]
        times_from_start: [0.0, 4.0, 6.0, 9.0]
        meta:
          name: Pick from floor
          usage: demo
          description: 'Pick a shirt-like object from floor in front of the robot'
@[end if]@

@[if end_effector in ["robotiq-2f-85", "robotiq-2f-140"]]@

      offer:
        joints: ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'gripper_finger_joint']
        positions: [0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.62, -1.577, 0.0]
        times_from_start: [0.0]
        meta:
          name: Offer Gripper
          usage: demo
          description: 'Offer Gripper'

      shake_hands:
        joints: ['gripper_finger_joint', 'torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        positions: [0.0, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    0.3, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    0.3, 0.296, 1.61, -0.93, -3.14, 1.40, -1.577, -0.2, -1.577,
                    0.3, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    0.3, 0.296, 1.61, -0.93, -3.14, 1.40, -1.577, -0.2, -1.577,
                    0.3, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577,
                    0.3, 0.296, 1.61, -0.93, -3.14, 1.83, -1.577, -0.53, -1.577]
        times_from_start: [0.0, 5.0, 6.0, 7.0, 8.0, 9.0, 11.0]
        meta:
          name: Shake Hands
          usage: demo
          description: 'shake_hands'

      pick_from_floor:
        joints: ['torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'gripper_finger_joint']
        positions: [0.226, 0.308, -0.695, -0.968, 1.582,  1.965, 0.273, -1.101, 0.0,
                    0.12, 0.809, -1.197, -1.119, 0.322, 1.96, -0.849, 0.041, 0.0,
                    0.12, 0.809, -1.197, -1.119, 0.345, 1.96, -0.849, 0.041, @[if end_effector == "robotiq-2f-85"]0.75@[else]0.65@[end if],
                    0.27, 0.21, -1.153, -1.538, 2.26, 1.965, 0.394, -0.082, @[if end_effector == "robotiq-2f-85"]0.75@[else]0.65@[end if]]
        times_from_start: [0.0, 4.0, 6.0, 9.0]
        meta:
          name: Pick from floor
          usage: demo
          description: 'Pick a shirt-like object from floor in front of the robot'
@[end if]@
@[else]@

      home:
        joints: [torso_lift_joint]
        positions: [0.15]
        times_from_start: [3.0]
        meta:
          name: Home
          usage: demo
          description: 'Go home'
@[end if]@


