#!/usr/bin/env python3
import os

urdf_path = os.path.expanduser('~/gofa_ws/src/gofa_viz/urdf/carrier.urdf')
ascii_stl = 'package://gofa_viz/resource/carrier_meshes/carrier_full_ascii.STL'

with open(urdf_path, 'r') as file:
    urdf_data = file.read()

# Replace any existing carrier_full STL line
import re
urdf_data = re.sub(r'<mesh filename=".*carrier_full.*\.STL"/>',
                   f'<mesh filename="{ascii_stl}"/>', urdf_data)

with open(urdf_path, 'w') as file:
    file.write(urdf_data)

print("URDF updated to use carrier_full_ascii.STL")
