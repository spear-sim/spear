#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import P4
import pathlib
import posixpath


parser = argparse.ArgumentParser()
parser.add_argument("--p4_depot_name", required=True)
parser.add_argument("--p4_stream_name", required=True)
parser.add_argument("--source_path", required=True)
parser.add_argument("--destination_path", required=True)
parser.add_argument("--p4_port")
parser.add_argument("--p4_user")
parser.add_argument("--p4_password")
parser.add_argument("--p4_revision")
args = parser.parse_args()


if __name__ == '__main__':

    p4 = P4.P4()

    if args.p4_port is not None:
        p4.port = args.p4_port
    if args.p4_user is not None:
        p4.user = args.p4_user
    if args.p4_password is not None:
        p4.password = args.p4_password

    revision_str = ""
    if args.p4_revision is not None:
        revision_str = f"@{args.p4_revision}"

    source_depot_name = args.p4_depot_name
    source_stream_name = args.p4_stream_name
    source_base_path = args.source_path
    source_base_depot_stream_path = posixpath.join("//", source_depot_name, source_stream_name, f"{source_base_path}{revision_str}")
    destination_base_path = args.destination_path

    p4.connect()
    p4.run_trust("-y")
    p4.disconnect()

    p4.connect()
    p4.run_login()
    p4.disconnect()

    # We use print(...) here because we want to be able to run this script in situations where the spear
    # module hasn't been installed.

    cmd = ["info"]
    print(f'[SPEAR | download_content_from_perforce.py] Executing: {" ".join(cmd)}')
    p4.connect()
    info_results = p4.run(*cmd)
    p4.disconnect()

    for k, v in info_results[0].items():
        print(f"{k}: {v}")

    cmd = ["files", source_base_depot_stream_path]
    print(f'[SPEAR | download_content_from_perforce.py] Executing: {" ".join(cmd)}')
    p4.connect()
    files_results = p4.run(*cmd)
    p4.disconnect()

    print(f"[SPEAR | download_content_from_perforce.py] Found {len(files_results)} files...")

    for files_result in files_results:

        source_full_path = files_result["depotFile"]
        source_full_path_tokens = pathlib.PurePosixPath(source_full_path).parts
        source_path = posixpath.join(*source_full_path_tokens[3:])
        source_file = source_full_path_tokens[-1]
        
        assert source_full_path_tokens[0] == "//"
        assert source_full_path_tokens[1] == source_depot_name
        assert source_full_path_tokens[2] == source_stream_name

        source_base_path_tokens = pathlib.PurePosixPath(source_base_path).parts
        source_base_dir = os.path.join(*source_base_path_tokens[:-1])
        assert source_path.startswith(source_base_dir)

        source_rel_path = posixpath.relpath(source_path, source_base_dir)
        source_rel_dir = os.path.dirname(source_rel_path)
        source_rel_dir_tokens = pathlib.PurePosixPath(source_rel_dir).parts
        destination_path = os.path.join(destination_base_path, *source_rel_dir_tokens, source_file)
        destination_dir = os.path.dirname(destination_path)

        os.makedirs(destination_dir, exist_ok=True)

        print(f"[SPEAR | download_content_from_perforce.py] Downloading {source_full_path} -> {destination_path}")
        
        cmd = ["print", "-o", destination_path, source_full_path]
        print(f'[SPEAR | download_content_from_perforce.py] Executing: {" ".join(cmd)}')
        p4.connect()
        print_results = p4.run(*cmd)
        p4.disconnect()

    print("[SPEAR | download_content_from_perforce.py] Done.")
