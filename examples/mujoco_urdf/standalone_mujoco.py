import argparse
import subprocess

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("--mujoco", default=r"F:\intel\interiorsim\third_party\mujoco\bin\simulate.exe")
    parser.add_argument("--mjcf_file", default=r"F:\intel\interiorsim\pipeline\fetch_in_apartment_0000.mjcf")
    args = parser.parse_args()

    cmd = [args.mujoco, args.mjcf_file]

    subprocess.run(cmd)
