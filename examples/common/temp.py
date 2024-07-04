import json

import numpy as np


class data():
    def test(self):
        self._height = 240
        self._width = 320
        self.data = []
        for i in range(0, self._height * self._width):
            self.data.append({'b': 111, 'g': 115, 'r': 123, 'a': 255})

        image = np.array([[[color['r'], color['g'], color['b']] for color in self.data]]).reshape([self._height, self._width, 3]).astype(dtype=np.uint8)
        print(image)


if __name__ == '__main__':
    ReturnValue = "[ 0x000001971890f400, 0x000001971890f700, 0x0000019739a30100, 0x0000019738f2fd00 ]"
    print(ReturnValue)

    array = ReturnValue[1:-1].strip().split(",")
    print(array)
    # data().test()

    val = np.array([1, 1, 1, 1])
    a = np.array([1, 1, 0, 0])
    b = np.array([1, 0, 1, 0])
    maska = a == 0
    maskb = b == 0
    maskc = np.logical_and(maska, maskb)
    print(maskc)
    # val[maska] = 0
    # val[maskb] = 0
    val[np.logical_and(maska, maskb)] = 0
    print(val)
    motor_torque_max = 0.5
    motor_torque = np.array([-1, 1, -1, 1])
    print("test", motor_torque)
    motor_torque = np.clip(motor_torque, -motor_torque_max, motor_torque_max)

    img = np.zeros((320, 240, 3))

    print(img.shape)
    img = np.transpose(img,(1,0,2))
    # print(img)
    print(img.shape)