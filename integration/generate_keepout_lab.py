"""Build PRIVATE Nav2 Jazzy KeepoutFilter lab assets from the official OOMWOO map.

Only writes the caller-provided temporary output directory in isolated
Podman. Never mutates OOMWOO's official map or navigation parameters.
Actual robot motion / mask enforcement must be tested separately.
Requires PyYAML and Pillow supplied by OOMWOO's jazzy-dev image.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml
from PIL import Image

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.zones import ZonePlan, ZoneShape


def generate(out: Path, *, test_room: Path, nav_params: Path):
    out.mkdir(parents=True,exist_ok=True)
    original=yaml.safe_load(test_room.read_text())
    image=Image.open(test_room.parent/original["image"]).convert("L")
    width,height=image.size
    res=float(original["resolution"])
    ox,oy,yaw=(float(v) for v in original["origin"])
    assert len(original["origin"])==3
    assert width==height==140 and abs(res-.05)<1e-8
    # Decode the SAME official map used by Nav2, including unknown pixels.
    # Image rows are stored top-to-bottom, ROS OccupancyGrid row zero is
    # bottom-to-top. This ties the zone plan fingerprint to actual occupancy.
    pixels=list(image.getdata())
    occupancy=tuple(
        100 if (255-pixels[row*width+col])/255>=original["occupied_thresh"]
        else 0 if (255-pixels[row*width+col])/255<=original["free_thresh"]
        else -1
        for row in reversed(range(height)) for col in range(width))
    raster=Raster(width,height,res,ox,oy,yaw,occupancy)
    grid=Grid(width,height,occupancy,resolution_m=res,clearance_cells=0)
    block=ZoneShape("Lab keepout rectangle","polygon",(
        (.67,-.55),(1.14,-.55),(1.14,.55),(.67,.55)))
    plan=ZonePlan.create(raster,grid,(block,))
    mask=plan.mask(raster,grid)
    count=mask.data.count(100)
    assert count>120 and count<1000
    # Occupancy grid data is row-major bottom-to-top. PGM storage is
    # top-to-bottom: reverse the row order; white=free, black=keepout.
    raw=bytes(0 if mask.data[row*width+col]==100 else 255
              for row in reversed(range(height))
              for col in range(width))
    picture=Image.frombytes("L",(width,height),raw)
    pgm=out/"keepout_test_room.pgm"
    picture.save(pgm)
    mask_yaml=out/"keepout_test_room.yaml"
    mask_yaml.write_text(yaml.safe_dump({
      "image":pgm.name,
      "mode":"trinary",
      "resolution":res,
      "origin":[ox,oy,yaw],
      "negate":0,
      "occupied_thresh":.65,
      "free_thresh":.25,
    },sort_keys=False),encoding="utf-8")

    original_params=yaml.safe_load(nav_params.read_text())
    # The official OOMWOO nav2_params.yaml contains "map.yaml".
    # Unlike its native sim_bringup launch, generic nav2_bringup does
    # NOT override this correctly, leaving map_server unconfigured.
    original_params["map_server"]["ros__parameters"]["yaml_filename"]=str(test_room)
    amcl=original_params["amcl"]["ros__parameters"]
    amcl["set_initial_pose"]=True
    amcl["initial_pose"]={"x":0.0,"y":0.0,"z":0.0,"yaw":0.0}
    for layer in ("global_costmap","local_costmap"):
        params=original_params[layer][layer]["ros__parameters"]
        assert "keepout_filter" not in params.get("filters",[])
        params["filters"]=["keepout_filter"]
        params["keepout_filter"]={
            "plugin":"nav2_costmap_2d::KeepoutFilter",
            "enabled":True,
            "filter_info_topic":"/valdi/keepout_costmap_filter_info",
            "transform_tolerance":.3,
            "override_lethal_cost":False,
        }
        # Separate filter inflation, using a value greater than actual
        # robot radius. Nav2 docs caution filters are not inflated by
        # their normal layered inflation plugins.
        params["filters"].append("keepout_inflation")
        params["keepout_inflation"]={
            "plugin":"nav2_costmap_2d::InflationLayer",
            "enabled":True,
            "inflation_radius":.235,
            "cost_scaling_factor":5.0,
        }
    params_path=out/"nav2_keepout_params.yaml"
    params_path.write_text(yaml.safe_dump(original_params,sort_keys=False),
                           encoding="utf-8")
    x0,y0=raster.centre(raster.cell_at(.85,0.0))
    assert mask.data[raster.cell_at(x0,y0)]==100
    meta={"count":count,"resolution_m":res,
          "origin":[ox,oy,yaw],"shape":"rectangular experimental no-go area",
          "rectangle_world":[[.67,-.55],[1.14,.55]],
          "source_map_yaml":str(test_room),
          "source_map_fingerprint":grid.fingerprint(),
          "params_source":str(nav_params),
          "filters_global":original_params["global_costmap"]["global_costmap"]["ros__parameters"]["filters"],
          "filters_local":original_params["local_costmap"]["local_costmap"]["ros__parameters"]["filters"],
          "mask_topic":"/valdi/keepout_filter_mask",
          "filter_info_topic":"/valdi/keepout_costmap_filter_info",
          "pgm":str(pgm),"mask_yaml":str(mask_yaml),
          "nav_params":str(params_path)}
    (out/"manifest.json").write_text(json.dumps(meta,sort_keys=True,indent=2)+"\n",
                                      encoding="utf-8")
    print("NAV2_KEEP_OUT_GENERATED "+json.dumps(meta,sort_keys=True))


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--test-room",type=Path,default=Path(
       "/ros_ws/install/oomwoo_sim_support/share/oomwoo_sim_support/maps/test_room.yaml"))
    p.add_argument("--nav-params",type=Path,default=Path(
       "/ros_ws/install/oomwoo_sim_support/share/oomwoo_sim_support/config/nav2_params.yaml"))
    args=p.parse_args()
    generate(args.out,test_room=args.test_room,nav_params=args.nav_params)
