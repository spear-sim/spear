## SPEAR v0.5.0

This release includes the following new user-facing features and improvements.

- We have developed an experimental new Python API that can be used to call any public Unreal function, and can be used to get or set any public Unreal property, on any Unreal object, at any time. Our new API exposes more than 10,000 public Unreal functions and more than 50,000 public Unreal properties. As far as we are aware, there is no comparable functionality in any existing video game simulator. We have started using the new API in our `mujoco_interop` example application (see below), and we expect it to support all of our internal use cases going forward.

- We have developed an export pipeline to support interoperability with the MuJoCo physics engine. To demonstrate this pipeline, we have developed a new example application, `mujoco_interop`, shown in the image below. In this example application, an interactive MuJoCo simulation (left) is used to set the poses of objects in the photorealistic SPEAR rendering (right).

<img width="1738" alt="image" src="https://github.com/isl-org/spear/assets/2341965/259c11ac-fabb-49d8-9360-6c593cbdbad5">

- We are releasing 23 new photorealistic Kujiale scenes. The following images of `kujiale_0010`, `kujiale_0020`, `kujiale_0030` were captured in SPEAR running at over 60 fps.

![image](https://github.com/isl-org/spear/assets/2341965/c06b5699-ccfd-423a-b50c-5b2d1ed9bfc3)

![image](https://github.com/isl-org/spear/assets/2341965/701d9b94-61c3-4c6a-ae50-5135c28a0144)

![image](https://github.com/isl-org/spear/assets/2341965/9653d787-6f94-4ffe-985d-2ccd2aad47eb)

- We have made many usability and readability improvements to our code.

## SPEAR v0.4.0

This release includes the following new user-facing features and improvements.

- We are releasing nine new photorealistic Kujiale scenes. The following image of `kujiale_0004` was captured in SPEAR running at 4K resolution at 50 fps.

![image](https://github.com/isl-org/spear/assets/2341965/a58d19b9-5494-42d5-bc66-4e17d2a6ad8c)

- We are re-releasing our `warehouse_0000` scene (pictured below), which is now fully compatible with Unreal Engine 5.

![image](https://github.com/isl-org/spear/assets/2341965/075daa76-eb75-46fa-8789-0e88686cb1c9)

- We have re-enabled our Fetch robot agent, which is now fully compatible with Unreal Engine 5 (pictured above). We have also implemented an improved tutorial, `open_loop_control_fetch`, demonstrating how to interact with our Fetch agent in a mobile manipulation application.

- We have made several usability and readability improvements to our code.

## SPEAR v0.3.0

This release includes the following new user-facing features and improvements.

- We are releasing a new photorealistic scene, `apartment_0000` under a Creative Commons license. We include all of the Unreal content for this scene directly in our GitHub repository, so users can easily modify and experiment with the content directly in the Unreal Editor. The following image of `apartment_0000` was captured in SPEAR running at 4K resolution at 60 fps.

![image](https://github.com/isl-org/spear/assets/2341965/f08aa881-5873-4d84-8113-b9b37d126939)

- We have upgraded from Unreal Engine 4 to Unreal Engine 5. This update significantly improves the overall visual quality that users can expect from SPEAR, and makes rendering noticeably more consistent across platforms. The following image shows our `kujiale_0000` scene, and demonstrates the visual quality users can expect in Unreal Engine 5. This image was captured in SPEAR running at 4K resolution at 40 fps.

![image](https://github.com/isl-org/spear/assets/2341965/8c6413b4-b72d-4f8e-9efc-fa264b7ec825)

- Under the hood, upgrading to Unreal Engine 5 enables SPEAR to take advantage of Unreal's latest rendering features, including a state-of-the-art fully dynamic global illumination lighting system known as Lumen, and a state-of-the-art virtual geometry streaming system known as Nanite. Lumen enables all lights and objects to be moved fully dynamically with minimal precomputation required, and Nanite enables very high-resolution geometry to be imported and rendered such that the overall rendering cost scales with pixels rather than triangles. We expect these features to be especially useful as we scale the number of scenes that are available in SPEAR in the coming months.

- We have improved the visual appearance of our OpenBot model.

![image](https://github.com/isl-org/spear/assets/2341965/a1fbf106-ae32-49d7-a5b1-cc98a58d59bc)

- We have made several usability and readability improvements to our code, especially to our examples.

## SPEAR v0.2.0

This release includes the following new user-facing features and improvements.

- A large new photorealistic scene, `warehouse_0000`, which can be used to train industrial robots for warehouse applications.

![image](https://user-images.githubusercontent.com/2341965/229026112-5500bd96-9066-4784-811c-d6ac1ed543b1.jpg)

- A new agent, Fetch, that is capable of mobile manipulation tasks.
- A new example application, `open_loop_control_fetch`, which demonstrates our Fetch agent performing a simple mobile manipulation task.

![image](https://user-images.githubusercontent.com/2341965/229028307-649ee7b3-d82a-4e80-8f74-5a1c9ea4809d.jpg)

- A new scene that is intended for debugging, `starter_content_0000`, which loads very quickly and is suitable for simple manipulation and navigation tasks.

![image](https://user-images.githubusercontent.com/2341965/229026660-e78a8459-5263-4683-be30-50fb45aae2cb.jpg)

- Improved behavior when running the `SpearSim` executable with no additional arguments. The default behavior is now an interactive navigation mode, where a user can navigate around the `kujiale_0000` scene.
- A new command-line tool, `run_executable`, which launches `SpearSim` in interactive navigation mode and loads a user-specified scene.
- An improved communication mechanism between the `SpearSim` executable and surrounding Python code, which more than doubles the overall frame rate when communicating large image observations.
- An improved navigation policy that was trained using our `imitation_learning_openbot` example application to collect data.
- Our macOS `SpearSim` executable is now correctly code-signed.

## SPEAR v0.1.0

- Initial release.
