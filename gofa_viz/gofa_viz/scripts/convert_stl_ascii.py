#!/usr/bin/env python3
from stl import mesh
import stl

# Load the binary STL
binary_mesh = mesh.Mesh.from_file('/home/kareem/gofa_ws/src/gofa_viz/resource/carrier_meshes/carrier_full.STL')

# Save as ASCII STL
binary_mesh.save('/home/kareem/gofa_ws/src/gofa_viz/resource/carrier_meshes/carrier_full_ascii.STL', mode=stl.Mode.ASCII)

print("Conversion done: carrier_full_ascii.STL created")

