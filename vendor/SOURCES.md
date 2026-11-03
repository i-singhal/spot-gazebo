# Spot robot description source

- Repository: https://github.com/rai-opensource/spot_description
- Revision: `156d1802bfb117f219dbfce7597d283d5fabc968`
- Standard model: `spot_description/urdf/spot.urdf.xacro`
- Expanded with: `arm:=false feet:=true add_ros2_control_tag:=false`
- Checkout: `vendor/spot_description` (upstream files retained unchanged)
- Licenses: [upstream MIT and BSD notices](spot_description/LICENSE)

The world uses the standard model and its `meshes/base/visual` and
`meshes/base/collision` assets, not `spot_simple` and not a hand-built box model.
OBJ material files are kept in their original relative locations.

The parent repository records this dependency as a Git submodule at the revision
above. After cloning the parent repository, restore it with:

```bash
git submodule update --init --recursive
python3 tools/generate_world.py
```

Generated `models/spot/spot.urdf` is the unmodified expansion with ROS package
mesh URIs. `spot_sim.urdf` resolves mesh URIs to local files and moves the mass
from the coincident fixed `base_link` onto `body` for Gazebo's URDF importer;
the rigid body's mass and inertia are preserved. Fixed links are lumped by
Gazebo's standard importer. Joint names, origins, axes and limits are retained.

Gazebo-only additions: joint controllers, odometry and state publishers,
VelocityControl, disabled gravity and disabled leg contact response. These are
simulation controller settings; the imported robot geometry is upstream Spot.
