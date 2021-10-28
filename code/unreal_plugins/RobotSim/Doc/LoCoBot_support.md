# LoCoBot Support

RobotSim support LoCoBot importing in UrdfBot mode. The LoCoBot structure and meshes are based
on [cmu_locobot_description](https://github.com/facebookresearch/pyrobot/blob/main/robots/LoCoBot/locobot_description/urdf/cmu_locobot_description.urdf)

In order to import to UE4, there are several modifications in URDF and mesh files. 

## Mesh Modification
Before importing to UE4, the pivot point Location for `main_body.dae` is moved by xyz = 0 0 0.0512. This is because currently RobotSim does not visual/collision offset for root link. 

## URDF Modification
### base_link
Remove  `base_footprint` link, use `base_link` as root_link. 
   
Move main_body.dae pivot point by xyz = 0 0 0.0512, and reset visual->origin and collision->origin.
```
<origin rpy="0 0 0" xyz="0 0 0"/>
```            

Use consistent description for visual and collision. (Using different description will cause weird scaling)

### wheels
change joint axis for `wheel_left_joint` and `wheel_right_joint`
```commandline
<joint>
    <!--<axis xyz="0 0 1"/>-->
    <axis xyz="0 1 0"/>
</joint>
```
add joint.limit  for `wheel_left_joint` and `wheel_right_joint` (currently not used)
```
<joint>
    <limit effort="8.85" velocity="17.4"/>
</joint>
```
change inertial.mass from 0.01 to 1 (large difference between parent and child link cause joint instability in UE4)

### caster_wheels
Add visual for `caster_front_link` and `caster_back_link`. Valid link requires both visual and collision description. 
friction and static friction set to 0

### fingers
UE4 uses Left-handed cooridinate. As such, the mesh files for r_finger and l_finger are swapped.

In order to have consistent response to control signal with fetch, the finger joint axis are modified as follow:

```commandline
    <joint name="joint_6" type="prismatic">
        <!--<axis xyz="0 1 0"/>-->
        <axis xyz="0 -1 0"/>
        <!--<limit effort="1" lower="-1" upper="0" velocity="1"/>-->
        <limit effort="1" lower="0" upper="0.04" velocity="1"/>
    </joint>
    
    <joint name="joint_7" type="prismatic">
        <!--<axis xyz="0 1 0"/>-->
        <<axis xyz="0 1 0"/>
        <!--limit effort="1" lower="0" upper="1" velocity="1"/>-->
        <limit effort="1" lower="0" upper="0.04" velocity="1"/>
    </joint>
```

Note that the range sizes are changed from 1 to 0.04 to match the actual gripper link size. 