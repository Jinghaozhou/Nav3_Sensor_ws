"""Resolve experimental-tool paths in Nav3 and legacy workspace layouts."""
import os
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
def localization_workspace():
    if os.environ.get("NAV3_LOCALIZATION_WS"):
        return Path(os.environ["NAV3_LOCALIZATION_WS"]).expanduser().resolve()
    if ROOT.parent.name == "tools":
        return ROOT.parents[2] / "Nav3_Localization_ws"
    return ROOT.parent / "kiss_localization_ws1.1"
def map_file():
    if os.environ.get("NAV3_MAP_PCD"):
        return Path(os.environ["NAV3_MAP_PCD"]).expanduser().resolve()
    ws = localization_workspace()
    shared = ws.parent.parent / "data/maps/building_split_space.pcd" if ws.parent.name == "modules" else ws.parent / "Nav3_ws/data/maps/building_split_space.pcd"
    return shared if shared.is_file() else ws / "src/localization/maps/building_split_space.pcd"
def labels_file():
    return localization_workspace() / "src/localization/maps/region_labels.csv"
