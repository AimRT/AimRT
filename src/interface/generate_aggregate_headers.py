# Copyright (c) 2024 The AimRT Authors.
# AimRT is licensed under Mulan PSL v2.

import os


def generate_aggregate_header(interface_dir):
    """Generate aggregate header for interface directory."""
    header_files = []
    aggregate_header = f"{interface_dir}.h"

    # Iterate over the directory, collect all .h header files
    for root, _, files in os.walk(interface_dir):
        for file in files:
            if file.endswith(".h") and file != aggregate_header:
                # Get relative path and convert to #include format
                relative_path = os.path.relpath(os.path.join(root, file), interface_dir)
                include_path = f'#include "{interface_dir}/{relative_path}"'
                header_files.append(include_path)

    # Sort header files
    header_files.sort()

    # Generate content
    content = (
        [
            "// Copyright (c) 2024 The AimRT Authors.",
            "// AimRT is licensed under Mulan PSL v2.",
            "",
            "#pragma once",
            "",
        ]
        + header_files
        + [
            "",
        ]
    )

    # Write to file
    with open(os.path.join(interface_dir, aggregate_header), "w") as f:
        f.write("\n".join(content))

    print(f"Generated {aggregate_header}")


if __name__ == "__main__":
    interface_dirs = [
        "aimrt_core_plugin_interface",
        "aimrt_module_c_interface",
        "aimrt_module_cpp_interface",
        "aimrt_module_protobuf_interface",
        "aimrt_module_ros2_interface",
        "aimrt_pkg_c_interface",
        "aimrt_type_support_pkg_c_interface",
    ]
    for interface_dir in interface_dirs:
        if os.path.exists(interface_dir):
            generate_aggregate_header(interface_dir)
        else:
            print(f"Warning: Directory {interface_dir} does not exist, skipping...")
