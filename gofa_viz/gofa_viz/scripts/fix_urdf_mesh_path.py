#!/usr/bin/env python3
import os

urdf_file = os.path.expanduser('~/gofa_ws/src/gofa_viz/urdf/carrier.urdf')

# Read URDF
with open(urdf_file, 'r') as f:
    data = f.read()

# Replace any previous mesh path to the correct one
data = data.replace('package://gofa_viz/resource/carrier_full_binary.STL',
                    'package://gofa_viz/resource/carrier_meshes/carrier_full_binary.STL')

# Write back
with open(urdf_file, 'w') as f:
    f.write(data)

print(f"✅ URDF mesh paths fixed: {urdf_file}")
