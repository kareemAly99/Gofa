#!/usr/bin/env python3
import os

# Paths
home = os.getenv('HOME')
urdf_file = os.path.join(home, 'gofa_ws', 'src', 'gofa_viz', 'urdf', 'carrier.urdf')
binary_stl_path = 'package://gofa_viz/resource/carrier_meshes/carrier_full_binary.STL'

# Read URDF
with open(urdf_file, 'r') as f:
    urdf = f.read()

# Replace the mesh path
import re
urdf_updated = re.sub(r'<mesh filename=".*carrier_full.*\.STL"/>',
                      f'<mesh filename="{binary_stl_path}"/>',
                      urdf)

# Save updated URDF
with open(urdf_file, 'w') as f:
    f.write(urdf_updated)

print(f"URDF updated to use {binary_stl_path}")
