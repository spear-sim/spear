import argparse
import mujoco
import mujoco.viewer
import numpy as np
import os
import time
import matplotlib.pyplot as plt

# XML=r"""
# <mujoco>
#   <asset>
#     <mesh file="Big_Carpet.stl"/>
#   </asset>
#   <worldbody>
#     <body>
#       <freejoint/>
#       <geom type="mesh" name="Big_Carpet" mesh="Big_Carpet"/>
#     </body>
#   </worldbody>
# </mujoco>
# """

# ASSETS=dict()
# with open('C:/github/spear/examples/mujoco/apartment_0000/output/Unknown/Big_Carpet/Big_Carpet.stl', 'rb') as f:
#   ASSETS['Big_Carpet.stl'] = f.read()

# model = mujoco.MjModel.from_xml_string(XML, ASSETS)
# data = mujoco.MjData(model)

# print(data.geom('Big_Carpet'))
# print(model.geom('Big_Carpet'))


# m = mujoco.MjModel.from_xml_path("C:/github/spear/examples/mujoco/apartment_0000/output/Unknown/Big_Carpet/Big_Carpet.xml")
# d = mujoco.MjData(m)

# print(m.body('Big_Carpet'))


#################################################################################################################################

# xml_path = os.path.realpath(os.path.join(os.path.dirname(__file__), "apartment_0000_vhacd", "scene.xml"))

# # create mujoco objects
# mujoco_model = mujoco.MjModel.from_xml_path(xml_path)
# mujoco_data = mujoco.MjData(mujoco_model)

# # perform this step once to load all information
# mujoco.mj_forward(mujoco_model, mujoco_data)

###################################
####### mujoco viewer block #######

# filename = 'apartment_0000_fine_coacd/scene.xml'
# m = mujoco.MjModel.from_xml_path(filename)
# d = mujoco.MjData(m)

# act_mids = [np.mean(m.actuator(i).ctrlrange) for i in range(m.nu)]
# act_mags = [0.5*(m.actuator(i).ctrlrange[1]-m.actuator(i).ctrlrange[0]) for i in range(m.nu)]
# period = 1000

# with mujoco.viewer.launch_passive(m, d) as viewer:
#     # Close the viewer automatically after 30 wall-seconds.
#     start = time.time()
#     t = 0
#     while viewer.is_running() and time.time() - start < 240:
#         step_start = time.time()

#         # mj_step can be replaced with code that also evaluates
#         # a policy and applies a control signal before stepping the physics.
#         l = np.sign(np.sin(2*np.pi*t/period))

#         for i in range(m.nu):
#             d.ctrl[i] = act_mids[i] + l*act_mags[i]
#         mujoco.mj_step(m, d)

#         # Example modification of a viewer option: toggle contact points every two seconds.
#         with viewer.lock():
#             viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

#         # Pick up changes to the physics state, apply perturbations, update options from GUI.
#         viewer.sync()

#         # Rudimentary time keeping, will drift relative to wall clock.
#         time_until_next_step = m.opt.timestep - (time.time() - step_start)
#         if time_until_next_step > 0:
#             time.sleep(time_until_next_step)

#         t += 1

# for _ in range(10):
    # mujoco_data.actuator("Cabinet/PhysicsConstraint_door_01_x_revolute_actuator").ctrl[0] = 1 # cabinet->door->revolute_joint->actuator
    # mujoco.mj_step(mujoco_model, mujoco_data)


###################################
####### mujoco viewer camera #######

if __name__ == "__main__":

    osp = os.path
    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf", required=True, help="path to scene MJCF file")
    args = parser.parse_args()


    mujoco_model = mujoco.MjModel.from_xml_path(osp.expanduser(args.mjcf))
    mujoco_data = mujoco.MjData(mujoco_model)

    # perform this step once to load all information
    mujoco.mj_forward(mujoco_model, mujoco_data)

    body_ids = [ x for x in range(mujoco_model.nbody) ]
    body_names = [ mujoco_model.body(body_id).name for body_id in range(mujoco_model.nbody) ]

    # filter out only actors and components that are chairs
    mj_chair_body_ids = [ x for x, y in zip(body_ids, body_names) if "Meshes/05_chair/Kitchen" in y ]
    mj_chair_body_names = [ y for x, y in zip(body_ids, body_names) if "Meshes/05_chair/Kitchen" in y ]
    
    chair_xpos_means = {i:mujoco_data.body(i).xpos for i in mj_chair_body_ids}

    # for x in range(mujoco_model.njnt):
    #     print(mujoco_model.body(mujoco_model.joint(x).bodyid[0]).name)

    print("mujoco_model.nu = ", mujoco_model.nu)
    print("mujoco_model.njnt = ", mujoco_model.njnt)

    # get mujoco viewer object
    viewer = mujoco.viewer.launch_passive(mujoco_model, mujoco_data)

    period = 1000
    start = time.time()
    t = 0
    muj_update_steps = 20

    viewer.cam.lookat = [0, 300.0, 100]
    viewer.cam.distance = 500.0
    viewer.cam.azimuth = 180
    viewer.cam.elevation = 0

    qpos = []
    xpos = []
    while viewer.is_running() and time.time() - start < 10:

        for _ in range(muj_update_steps):

            # for i in range(mujoco_model.njnt):
                # mujoco_data.joint(i).qfrc_applied = np.array([10.0, 0, 0, 0, 0, 0], dtype=np.float64)

            for i in mj_chair_body_ids:
                mujoco_data.body(i).xpos = chair_xpos_means[i] + [0.0, 10.0, 0.0]

            mujoco.mj_step(mujoco_model, mujoco_data)
            
            # updated qpos
            qpos.append(mujoco_data.joint(0).qpos[0])
            xpos.append(mujoco_data.body(mj_chair_body_ids[0]).xpos[1])

            # increament time
            t += 1

        viewer.sync()

    viewer.close()

    plt.plot(qpos)
    plt.plot(xpos)
    plt.show()