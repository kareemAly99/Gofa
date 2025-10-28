#!/usr/bin/env python3
from stl import mesh
import os

# Paths
home = os.getenv('HOME')
ascii_file = os.path.join(home, 'gofa_ws', 'src', 'gofa_viz', 'resource', 'carrier_meshes', 'carrier_full_ascii.STL')
binary_file = os.path.join(home, 'gofa_ws', 'src', 'gofa_viz', 'resource', 'carrier_meshes', 'carrier_full_binary.STL')
urdf_file = os.path.join(home, 'gofa_ws', 'src', 'gofa_viz', 'urdf', 'carrier.urdf')

# Load ASCII STL
ascii_mesh = mesh.Mesh.from_file(ascii_file)

# Save as binary STL (filename ending in .STL is enough)
ascii_mesh.save(binary_file)

print(f"Binary STL created: {binary_file}")

# Update URDF automatically
with open(urdf_file, 'r') as f:
    urdf_data = f.read()
urdf_data = urdf_data.replace('carrier_full_ascii.STL', 'carrier_full_binary.STL')
with open(urdf_file, 'w') as f:
    f.write(urdf_data)

print(f"URDF updated to use binary STL")
