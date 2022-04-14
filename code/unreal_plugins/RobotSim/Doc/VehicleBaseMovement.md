# Vehicle Base Control 

### Low Level Control 
Te achieve vehicle base control, call Motor::SetDriveTargetVelocity() for l_wheel_joint and r_wheel_joint. 

Since in low level API, UE4 only provides `SetAngularVelocityTarget()` for joint velocity control. Other method such as torque control might require development with PhysX API. 
In PhysX, target velocity control follows following formula (see [PhysX Doc](https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Joints.html#drives)):

        force = damping * (target velocity - current velocity)

This function does not guarantee the joint to reach target velocity, as it might be affected by the environment such as friction or gravity. 
Although it does not provide physical parameter control such as torque control, the underlying simulation mechanism is entirely physical.

Note that the unit is in revolution/second (see [link](https://answers.unrealengine.com/questions/531176/what-are-the-units-for-angular-velocity-in-physics.html))

### High Level Control 
For high level API, there are several function bind to keyboard actions in `UrdfBotPawn.h` (see `AUrdfBotPawn::setupInputBindings`). 
Developed based on low level API, these function determines joint target velocity depending on current target and actual velocity of the joint.
Limited by functionality of low level API, we approximate impulse like behavior as following pseudocode shows:

    void AUrdfBotPawn::onBaseMove(float keyboardSignal)
    {
        if(keyboardSignal)
        {
            if(driveEnabled)
            {
                previousTargetVelocity = findPreviousTargetVelocity();
                nextTargetVelocity = previousTargetVelocity + keyboardSignal;
                SetDriveTargetVelocity(nextTargetVelocity);
            }
            else
            {
                // if actualVelocity > keyboardSignal, using 0 + keyboardSignal might result in braking.
                actualVelocity = findCurrentVelocity();
                nextTargetVelocity = actualVelocity + keyboardSignal;
                enableVelocityDrive(true);
                SetDriveTargetVelocity(nextTargetVelocity);
            }
        }
        else
        {
            enableVelocityDrive(false);
            SetDriveTargetVelocity(0);
        }
    }

In this format, press `WS` provide acceleration on the base, while `AD` provide velocity difference in two wheel to rotate the base. It is also possible to directly determine driveTarget directly.

### Friction with the Ground

In RobotSim, the Fetch robot has six contact points with the ground: l_wheel_link, r_wheel_link, and four support wheels from the base_link.

In Fetch_ROS, they add four caster links that wrap up the support wheels such that the base moves smoother (see [link](https://github.com/fetchrobotics/fetch_gazebo/pull/49)). 
Typically, the friction is 0.1 for caster links and very high for wheels (see [link](https://github.com/fetchrobotics/fetch_gazebo/pull/59)).

In RobotSim, the robot might stop very quickly if no internal driver. This is primarily due to the friction between base_link and the ground.
It is possible to adjust the friction.

In Examples/fetch_description/robots/fetch.xml, you might change the materials for caster links to modify their friction.

    plastic - friction = 0.1  
    metal - friction = 0  
    other (default) - friction = 0.7

The robot might slide along the ground for some distance if caster friction set to `0` or `0.1` .  
The robot might stop very quickly if friction is `0.7`.   
You might adjust the friction value or other material property by modifying corresponding unreal_material.uasset file。 
