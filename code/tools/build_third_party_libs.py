import os 
import shutil
import subprocess
import sys

SCRIPT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))
MIN_CMAKE_VERSION = "3.5.1"

# check cmake version requirement
min_cmake_version = MIN_CMAKE_VERSION.split(".")
print(f"cmake min version required is {MIN_CMAKE_VERSION}, checking if this requirement is met...")
cmake_version = (subprocess.run(["cmake", "--version"], stdout=subprocess.PIPE).stdout).decode("utf-8").split('\n')[0].split(' ')[-1]
print(f"cmake version on this sytem is {cmake_version}")
cmake_version = cmake_version.split(".")
assert len(min_cmake_version) == len(cmake_version)
for i in range(len(min_cmake_version)):
    if int(cmake_version[i]) > int(min_cmake_version[i]):
        break
    if i == len(min_cmake_version) - 1:
        assert False
print("cmake version looks good...")

print("building rbdl...")
rbdl_build_dir = os.path.join(SCRIPT_DIR_PATH, "..", "third_party", "rbdl", "build")
if os.path.isdir(rbdl_build_dir):
    shutil.rmtree(rbdl_build_dir, ignore_errors=True)

os.makedirs(rbdl_build_dir)
os.chdir(rbdl_build_dir)

if sys.platform == "linux":
    os.environ["CC"] = "clang"
    os.environ["CXX"] = "clang++"
    cmake_cmd = subprocess.run(["cmake", "-DCMAKE_BUILD_TYPE=Release" , "-DRBDL_BUILD_STATIC=ON", "-DRBDL_BUILD_ADDON_URDFREADER=ON",  "-DCMAKE_CXX_COMPILER='clang++'", "-DCMAKE_CXX_FLAGS='-fPIC -stdlib=libc++'", ".."], stdout=sys.stdout)
else:
    cmake_cmd = subprocess.run(["cmake", "-DCMAKE_BUILD_TYPE=Release" , "-DRBDL_BUILD_STATIC=ON", "-DRBDL_BUILD_ADDON_URDFREADER=ON", ".."], stdout=sys.stdout)
assert cmake_cmd.returncode == 0
rbld_build_cmd = subprocess.run(["cmake",  "--build", ".", "--"], stdout=sys.stdout)
assert rbld_build_cmd.returncode == 0
print("rbdl built successfully...")

print("building rpclib...")
rpblib_build_dir = os.path.join(SCRIPT_DIR_PATH, "..", "third_party", "rpclib", "build")
if os.path.isdir(rpblib_build_dir):
    shutil.rmtree(rpblib_build_dir, ignore_errors=True)

os.makedirs(rpblib_build_dir)
os.chdir(rpblib_build_dir)

if sys.platform == "linux":
    os.environ["CC"] = "clang"
    os.environ["CXX"] = "clang++"
    cmake_cmd = subprocess.run(["cmake", "-DCMAKE_BUILD_TYPE=Release" , "-DCMAKE_CXX_FLAGS='-stdlib=libc++'", "-DCMAKE_POSITION_INDEPENDENT_CODE=ON", ".."], stdout=sys.stdout)
else:
    cmake_cmd = subprocess.run(["cmake", "-DCMAKE_BUILD_TYPE=Release"], stdout=sys.stdout)
assert cmake_cmd.returncode == 0
rpclib_build_cmd = subprocess.run(["cmake",  "--build", ".", "--"], stdout=sys.stdout)
assert rpclib_build_cmd.returncode == 0
print("rpclib built successfully...")

print("building yaml-cpp...")
yamlcpp_build_dir = os.path.join(SCRIPT_DIR_PATH, "..", "third_party", "yaml-cpp", "build")
if os.path.isdir(yamlcpp_build_dir):
    shutil.rmtree(yamlcpp_build_dir, ignore_errors=True)

os.makedirs(yamlcpp_build_dir)
os.chdir(yamlcpp_build_dir)

if sys.platform == "linux":
    os.environ["CC"] = "clang"
    os.environ["CXX"] = "clang++"
    cmake_cmd = subprocess.run(["cmake", "-DCMAKE_BUILD_TYPE=Release" , "-DCMAKE_CXX_FLAGS='-stdlib=libc++'", "-DCMAKE_POSITION_INDEPENDENT_CODE=ON", ".."], stdout=sys.stdout)
else:
    cmake_cmd = subprocess.run(["cmake", "-DCMAKE_BUILD_TYPE=Release"], stdout=sys.stdout)
assert cmake_cmd.returncode == 0
yamlcpp_build_cmd = subprocess.run(["cmake",  "--build", ".", "--"], stdout=sys.stdout)
assert yamlcpp_build_cmd.returncode == 0
print("yaml-cpp built successfully...")
