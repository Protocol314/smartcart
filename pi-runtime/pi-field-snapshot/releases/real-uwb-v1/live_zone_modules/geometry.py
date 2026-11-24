"""Cart footprint relative to nominal front-track rotation midpoint."""
import json
import math
from pathlib import Path

def load_geometry(path=None):
    path=Path(path) if path else Path(__file__).with_name('cart_geometry.json')
    data=json.loads(path.read_text(encoding='utf-8'))
    if data.get('reference')!='front_track_midpoint':raise ValueError('unsupported_geometry_reference')
    keys=('length_m','width_m','front_extent_m','rear_extent_m','wheelbase_m',
          'lidar_x_m','lidar_y_m','uwb_x_m','uwb_y_m','margin_m','noise_m')
    if not all(type(data.get(k)) in (int,float) and math.isfinite(data[k]) for k in keys):raise ValueError('invalid_geometry')
    if not (.1<=data['length_m']<=3 and .1<=data['width_m']<=3 and
            0<=data['front_extent_m']<=3 and 0<=data['rear_extent_m']<=3 and
            abs(data['front_extent_m']+data['rear_extent_m']-data['length_m'])<1e-8 and
            0<data['wheelbase_m']<=data['rear_extent_m'] and
            .1<=data['margin_m']<=1 and 0<=data['noise_m']<=.2):raise ValueError('invalid_footprint')
    data['body_radius_m']=math.hypot(max(data['front_extent_m'],data['rear_extent_m']),data['width_m']/2)
    data['zone_inflation_m']=data['body_radius_m']+data['margin_m']
    return data
