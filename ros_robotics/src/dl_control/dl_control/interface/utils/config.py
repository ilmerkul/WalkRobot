import os
from typing import Dict, List

import yaml


def load_config(file_path: str) -> Dict[str, str]:
    if not os.path.exists(file_path):
        raise ValueError(f"config {file_path} not exist")

    with open(file_path, "r") as f:
        config = yaml.safe_load(f)

    return config
