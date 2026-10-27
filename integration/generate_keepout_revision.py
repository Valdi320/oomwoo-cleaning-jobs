"""Prepare a SECOND test-only Nav2 keepout mask for hot-change verification.

Reads current private lab mask YAML; writes a separate new PGM + YAML
into that SAME ignored .local-sim/keepout-lab directory. Does not modify
OOMWOO upstream maps, ROS2 robot config or existing first mask.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import json,math,yaml
from PIL import Image
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.zones import ZoneShape,ZonePlan

def generate(root:Path):
    original=yaml.safe_load((root/"keepout_test_room.yaml").read_text())
    resolution=float(original["resolution"])
    x,y,yaw=map(float,original["origin"])
    img=Image.open(root/original["image"]).convert("L")
    w,h=img.size
    empty=(0,)*(w*h)
    raster=Raster(w,h,resolution,x,y,yaw,empty)
    grid=Grid(w,h,empty,resolution_m=resolution)
    shape=ZoneShape("Alternate lab forbidden rectangle","polygon",
                  ((-1.02,-.55),(-.57,-.55),(-.57,.55),(-1.02,.55)))
    plan=ZonePlan.create(raster,grid,(shape,))
    mask=plan.mask(raster,grid)
    assert mask.data.count(100)>=150
    binary=bytes(0 if mask.data[row*w+col]==100 else 255
              for row in reversed(range(h)) for col in range(w))
    path=root/"keepout_test_room_v2.pgm"
    Image.frombytes("L",(w,h),binary).save(path)
    doc={**original,"image":path.name}
    yml=root/"keepout_test_room_v2.yaml"
    yml.write_text(yaml.safe_dump(doc,sort_keys=False),encoding="utf-8")
    res={"new_mask_cells":mask.data.count(100),
        "new_keepout_world":[[-1.02,-.55],[-.57,.55]],
        "source_yml":str(root/"keepout_test_room.yaml"),
        "new_yml":str(yml),
        "mask_digest_source_map":grid.fingerprint()}
    print("OOMWOO_ZONE_V2 "+json.dumps(res,sort_keys=True),flush=True)

if __name__=="__main__":
    cli=argparse.ArgumentParser()
    cli.add_argument("--root",type=Path,required=True)
    generate(cli.parse_args().root)
