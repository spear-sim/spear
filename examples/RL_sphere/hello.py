import numpy as np

if __name__ == '__main__':
    # [-1.79769313e+308 -1.79769313e+308 -1.79769313e+308] [1.79769313e+308 1.79769313e+308 1.79769313e+308] (3,) [ True  True  True]
    bounded = np.array([True, True, True])
    low = np.array([-1.79769313e+308 ,- 1.79769313e+308 ,- 1.79769313e+308])
    high = np.array([1.79769313e+308 , 1.79769313e+308 , 1.79769313e+308])
    print("sample start.", low, high, bounded[bounded].shape, bounded)

    val = np.random.uniform(
        low=low[bounded], high=high[bounded], size=bounded[bounded].shape
    )
