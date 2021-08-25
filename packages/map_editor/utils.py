from collections import Counter
from os import listdir
from os.path import join
import logging
from typing import Optional

import yaml
from dataclasses import dataclass

logger = logging.getLogger('root')
CONFIG_PATH = './doc/apriltagsDB.yaml'


def get_id_by_type(type_of_obj: str) -> Optional[int]:
    with open(CONFIG_PATH) as file:
        content = yaml.safe_load(file)
        for tag in content:
            if tag['traffic_sign_type'] == type_of_obj:
                return int(tag['tag_id'])


@dataclass
class TagStore:
    id: int
    type: str


def count_elements(data): return Counter(data)


def get_list_dir(dir_path):
    try:
        entries = listdir(dir_path)
        return entries
    except FileNotFoundError as e:
        logger.warning(e)
        return []


def get_list_dir_with_path(dir_path): return [(filename, join(dir_path, filename)) for filename in
                                              get_list_dir(dir_path)]


def get_available_translations(lang_dir_path): return {filename[len('lang_'):-len('.qm')]: path for filename, path in
                                                       get_list_dir_with_path(lang_dir_path)}


if __name__ == '__main__':
    print(get_id_by_type('stop'))