import mmap

import numpy as np


class CameraSensor:
    def __init__(self, instance, unreal_actor, render_pass_names=["final_color", "depth", "normal", "segmentation"], width=512, height=512, fov=90):
        assert unreal_actor
        self._instance = instance
        self._render_pass_names = render_pass_names
        self._width = width
        self._height = height
        self._fov = fov

        self._camera_sensor_component = instance.unreal_service.get_component_by_type_v2("/Script/CoreUObject.Class'/Script/SpComponents.CameraSensorComponent'", unreal_actor)
        assert self._camera_sensor_component
        self._camera_sensor_component_class = instance.unreal_service.get_class(self._camera_sensor_component)

        # setup CameraSensorComponent
        camera_component = instance.unreal_service.get_component_by_type("UCameraComponent", unreal_actor)
        assert camera_component
        setup_args = {
            "camera_component": instance.unreal_service.to_ptr(camera_component),
            "render_pass_names": self._render_pass_names,
            "width": self._width,
            "height": self._height,
            "fov": self._fov,
        }
        unreal_camera_sensor_setup_func = instance.unreal_service.find_function_by_name(uclass=self._camera_sensor_component_class, name="setup")
        instance.unreal_service.call_function(self._camera_sensor_component, unreal_camera_sensor_setup_func, setup_args)

    def get_images(self, render_pass_name="final_color"):
        assert render_pass_name in self._render_pass_names
        # do rendering
        self._instance.rpc_client.call("cpp_func_service.call_func", self._camera_sensor_component, "camera_sensor.render",
                                       {'items_':
                                            {'render_pass_name': {'data_': render_pass_name, 'data_type_': 0}}}
                                       )
        # get rendered image from shared memory
        shared_memory_desc = self._instance.rpc_client.call("cpp_func_service.get_shared_memory_views", self._camera_sensor_component)
        shared_memory_object = mmap.mmap(-1, shared_memory_desc[render_pass_name]['num_bytes_'], shared_memory_desc[render_pass_name]['id_'])
        if render_pass_name == 'final_color' or render_pass_name == 'segmentation':
            shared_memory_array = np.ndarray(shape=(-1,), dtype=np.uint8, buffer=shared_memory_object)
            img = shared_memory_array.reshape([self._width, self._height, -1])
            img[:, [0, 1, 2]] = img[:, [2, 1, 0]]
            return img[:, :, :3]
        if render_pass_name == "depth":
            shared_memory_array = np.ndarray(shape=(-1,), dtype=np.float32, buffer=shared_memory_object)
            img = shared_memory_array.reshape([self._width, self._height, -1])
            return img[:, :, 0]
        elif render_pass_name == 'normal':
            shared_memory_array = np.ndarray(shape=(-1,), dtype=np.float32, buffer=shared_memory_object)
            img = shared_memory_array.reshape([self._width, self._height, -1])
            img = img * 0.5 + 0.5
            return img[:, :, :3]
        else:
            assert False
