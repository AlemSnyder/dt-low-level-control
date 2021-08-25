from collections import defaultdict
from typing import List, Any, Dict

from utils import TagStore
import yaml

CONFIG_PATH = './doc/apriltagsDB.yaml'
TRAFFIC_TYPE = 'TrafficSign'
TYPES = defaultdict(list)
TRAFFIC_SIGN_TYPES = defaultdict(list)


def get_duckietown_types() -> Dict[Any, List[TagStore]]:
    with open(CONFIG_PATH, 'r') as file:
        global TYPES, TRAFFIC_SIGN_TYPES
        content = yaml.safe_load(file)
        for tag in content:
            if not tag["tag_type"]:
                continue
            TYPES[tag["tag_type"]].append(TagStore(int(tag["tag_id"]), tag['traffic_sign_type']))
            if tag["tag_type"] == TRAFFIC_TYPE and tag['traffic_sign_type']:
                TRAFFIC_SIGN_TYPES[tag["traffic_sign_type"]].append(tag["tag_id"])
    return TYPES
