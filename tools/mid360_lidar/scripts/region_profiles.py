"""区域路由；只接受明确的1至13，不隐式退回其他区域。"""
import re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def region_number(region):
    value=str(region)
    if not re.fullmatch(r'(?:space_)?(?:[1-9]|1[0-3])',value):
        raise ValueError('区域必须是space_1至space_13或对应编号')
    return int(value.removeprefix('space_'))
def virtual_config_for_region(region):
    return ROOT/f'config/virtual_initialization_space_{region_number(region)}.json'
def dynamic_profile():
    return ROOT/'config/region_initialization_all_spaces.yaml'
