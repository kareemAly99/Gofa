#!/usr/bin/env python3
from stl import mesh
import os

# Paths
home = os.getenv('HOME')
ascii_file = os.path.join(home, 'gofa_ws', 'src', 'gofa_viz', 'resource', 'carrier_meshes', 'carrier_full_ascii.STL')
binary_file = os.path.join(home, 'gofa_ws', 'src', 'gofa_viz', 'resource', 'carrier_meshes', 'carrier_full_binary.STL')

# Load ASCII STL
ascii_mesh = mesh.Mesh.from_file(ascii_file)

# Save as binary STL
ascii_mesh.save(binary_file, mode='binary')

print(f"Conversion done: {binary_file} created")
