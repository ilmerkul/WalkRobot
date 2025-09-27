import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

pkg_path = get_package_share_directory("control")


def modify_namespace_and_save_yaml_files(
    input_yaml: str, yaml_count: int, base_namespace: str
) -> None:
    path = Path(os.path.join(pkg_path, "config", input_yaml))
    if path.is_file():
        with open(path, "r") as f:
            data = yaml.safe_load(f)
    else:
        data = yaml.safe_load(input_yaml)

    for i in range(yaml_count):
        new_data = {}
        new_namespace = f"{base_namespace}_{i}"

        for key in data.keys():
            new_key = f"/{new_namespace}/control/{key}"
            new_data[new_key] = data[key]

        output_path = path.parent / f"control_{i}.yaml"
        with open(output_path, "w") as f:
            yaml.dump(new_data, f, default_flow_style=False, sort_keys=False)

        print(f"Saved config for {new_namespace} to {output_path}")
