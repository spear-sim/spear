# Python Client for communicating with server running on UnrealEngine

## Required

- Python 3.6 and above
- `numpy`
- `gym`
- `yacs==0.1.8`
- `psutil`

## Setup

- We recommend you have a python virtual environment managers like `virtualenv` / `conda`/ `mini-conda`.
- You will need to install msgpack-rpc-python package that is provided by us.
  - Run `cd thirdparty/msgpack-rpc-python` and `pip install -e .`
- Recommended way to install is via `pip install -e .` in current directory. `pip` will take care of dependency installs.
- You could also install python packages mentioned in `requirements.txt`. You can do `pip install -r requirements.txt`, for example, or you could install them separately using pip.

## Something wrong?

- If you encounter error as in (<https://github.com/msgpack-rpc/msgpack-rpc-python/issues/19>), (<https://github.com/microsoft/AirSim/issues/2633>), re-create your virtual env from scratch with minimal packages. Also, install `msgpack-rpc-python` package towards the end.
