#!/usr/bin/env python3
import os
import re

# Paths
home = os.getenv('HOME')
urdf_file = os.path.join(home, 'gofa_ws', 'src', 'gofa_viz', 'urdf', 'carrier.urdf')

# Read URDF
with open(urdf_file, 'r') as f:
    urdf_content = f.read()

# Replace STL paths with package:// URI
updated_content = re.sub(
    r'file://[^"]*carrier_full_binary\.STL',
    'package://gofa_viz/resource/carrier_meshes/carrier_full_binary.STL',
    urdf_content
)

# Write back
with open(urdf_file, 'w') as f:
    f.write(updated_content)

print(f"✅ URDF updated: {urdf_file}")
