#!/usr/bin/env python3
import os, re

home = os.getenv('HOME')
urdf_file = os.path.join(home, 'gofa_ws', 'src', 'gofa_viz', 'urdf', 'carrier.urdf')
pkg_file  = os.path.join(home, 'gofa_ws', 'src', 'gofa_viz', 'package.xml')

# --- Fix URDF STL paths ---
if os.path.exists(urdf_file):
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    # replace any STL path with package:// reference
    urdf_content_new = re.sub(
        r'file://[^"]*resource/([^"]*\.STL)',
        r'package://gofa_viz/resource/\1',
        urdf_content,
        flags=re.IGNORECASE
    )
    if urdf_content != urdf_content_new:
        with open(urdf_file, 'w') as f:
            f.write(urdf_content_new)
        print(f"✅ URDF paths fixed in: {urdf_file}")
    else:
        print(f"ℹ️ URDF already clean: {urdf_file}")
else:
    print("⚠️ URDF file not found:", urdf_file)

# --- Check package.xml ---
if os.path.exists(pkg_file):
    with open(pkg_file, 'r') as f:
        pkg_content = f.read()
    missing = []
    if '<build_depend>ament_cmake</build_depend>' not in pkg_content:
        missing.append('  <build_depend>ament_cmake</build_depend>')
    if '<exec_depend>ament_cmake</exec_depend>' not in pkg_content:
        missing.append('  <exec_depend>ament_cmake</exec_depend>')
    if missing:
        pkg_content_new = pkg_content.replace('</package>', '\n'.join(missing) + '\n</package>')
        with open(pkg_file, 'w') as f:
            f.write(pkg_content_new)
        print(f"✅ package.xml updated with missing dependencies: {pkg_file}")
    else:
        print(f"ℹ️ package.xml already has required dependencies: {pkg_file}")
else:
    print("⚠️ package.xml not found:", pkg_file)
