"""Expand the upstream Spot URDF and convert it to a Gazebo Harmonic world.

The source checkout is retained unchanged. Simulation-only changes are made in
models/spot/spot_sim.urdf and the generated SDF, never in the upstream Xacros.
"""
import copy
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from spot_gazebo.model import JOINT_NAMES, SIT_HEIGHT, SIT_HIP, SIT_KNEE

PACKAGE = ROOT/'vendor/spot_description/spot_description'
MODEL_DIR = ROOT/'models/spot'


def sub(parent, tag, value=None, **attrs):
    node = E.SubElement(parent, tag, attrs)
    if value is not None:
        node.text = str(value)
    return node


def plugin(parent, filename, name):
    return sub(parent, 'plugin', filename=filename, name=name)


def write(root, path):
    E.indent(root)
    E.ElementTree(root).write(path, encoding='unicode', xml_declaration=True)


def expand_urdf():
    if not PACKAGE.is_dir():
        raise FileNotFoundError('Missing vendor/spot_description checkout; see vendor/SOURCES.md')
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    # Provide a local ament index so $(find spot_description) resolves without
    # installing or modifying the system ROS workspace.
    prefix = ROOT/'build/description_prefix'
    index = prefix/'share/ament_index/resource_index/packages'
    index.mkdir(parents=True, exist_ok=True)
    (index/'spot_description').touch()
    package_link = prefix/'share/spot_description'
    package_target = os.path.relpath(PACKAGE, package_link.parent)
    if package_link.is_symlink() and os.readlink(package_link) != package_target:
        package_link.unlink()
    if not package_link.exists():
        package_link.symlink_to(package_target, target_is_directory=True)
    env = os.environ.copy()
    env['AMENT_PREFIX_PATH'] = str(prefix)+os.pathsep+env.get('AMENT_PREFIX_PATH', '')
    # Keep the generated model deterministic regardless of user payload settings.
    env = {key: value for key, value in env.items() if not key.startswith('SPOT_')}
    source = (PACKAGE/'urdf/spot.urdf.xacro').relative_to(ROOT)
    output = subprocess.check_output(['xacro', str(source),
                                      'arm:=false', 'feet:=true', 'add_ros2_control_tag:=false'],
                                     env=env, cwd=ROOT)
    output = b'\n'.join(line.rstrip() for line in output.splitlines()) + b'\n'
    (MODEL_DIR/'spot.urdf').write_bytes(output)
    urdf = E.fromstring(output)
    # The upstream body is a visual root with mass on a coincident fixed child.
    # Move that inertia onto body so URDF-to-SDF retains the entire tree. The
    # rigid assembly's mass, CoM and inertia are unchanged.
    body = urdf.find("link[@name='body']")
    base = urdf.find("link[@name='base_link']")
    inertia = base.find('inertial')
    if body.find('inertial') is None and inertia is not None:
        body.append(copy.deepcopy(inertia))
        base.remove(inertia)
    for mesh in urdf.findall('.//mesh'):
        filename = mesh.attrib['filename']
        prefix_uri = 'package://spot_description/'
        if not filename.startswith(prefix_uri):
            raise ValueError(f'Unexpected upstream mesh URI: {filename}')
        asset = PACKAGE/filename[len(prefix_uri):]
        if not asset.is_file():
            raise FileNotFoundError(asset)
        mesh.set('filename', asset.as_uri())
    write(urdf, MODEL_DIR/'spot_sim.urdf')
    return urdf


def main():
    urdf = expand_urdf()
    converted = subprocess.check_output(['gz', 'sdf', '-p', str(MODEL_DIR/'spot_sim.urdf')])
    model = E.fromstring(converted).find('model')
    model.set('name', 'spot')
    sub(model, 'pose', f'0 0 {SIT_HEIGHT} 0 0 0')
    sub(model, 'self_collide', 'false')
    for link in model.findall('link'):
        sub(link, 'gravity', 'false')
        for collision in link.findall('collision'):
            surface = collision.find('surface')
            if surface is None:
                surface = sub(collision, 'surface')
            contact = surface.find('contact')
            if contact is None:
                contact = sub(surface, 'contact')
            sub(contact, 'collide_bitmask', '1' if link.attrib['name'] == 'body' else '0')
    available = {j.attrib['name'] for j in model.findall('joint')}
    if not set(JOINT_NAMES.values()) <= available:
        raise ValueError(f'URDF conversion lost joints: {set(JOINT_NAMES.values())-available}')
    for alias, name in JOINT_NAMES.items():
        ctl = plugin(model, 'gz-sim-joint-position-controller-system', 'gz::sim::systems::JointPositionController')
        sub(ctl, 'joint_name', name)
        sub(ctl, 'topic', f'/model/spot/joint/{name}/cmd_pos')
        sub(ctl, 'use_velocity_commands', 'true')
        sub(ctl, 'p_gain', 15)
        angle = SIT_HIP if alias.endswith('_hy') else SIT_KNEE if alias.endswith('_kn') else 0
        sub(ctl, 'initial_position', angle)
    ctl = plugin(model, 'gz-sim-velocity-control-system', 'gz::sim::systems::VelocityControl')
    sub(ctl, 'topic', '/model/spot/cmd_vel')
    odom = plugin(model, 'gz-sim-odometry-publisher-system', 'gz::sim::systems::OdometryPublisher')
    sub(odom, 'odom_publish_frequency', 50)
    sub(odom, 'dimensions', 3)
    sub(odom, 'odom_topic', '/model/spot/odometry')
    sub(odom, 'odom_frame', 'odom')
    sub(odom, 'robot_base_frame', 'body')
    js = plugin(model, 'gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher')
    sub(js, 'topic', '/model/spot/joint_state')
    model_sdf = E.Element('sdf', version='1.9')
    model_sdf.append(copy.deepcopy(model))
    write(model_sdf, MODEL_DIR/'model.sdf')
    sdf = E.Element('sdf', version='1.9')
    world = sub(sdf, 'world', name='spot_world')
    physics = sub(world, 'physics', name='physics', type='ignored')
    sub(physics, 'max_step_size', .002)
    sub(physics, 'real_time_factor', 1)
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')]:
        plugin(world, f'gz-sim-{filename}-system', f'gz::sim::systems::{name}')
    light = sub(world, 'light', name='sun', type='directional')
    sub(light, 'pose', '0 0 10 0 0 0')
    sub(light, 'diffuse', '0.9 0.9 0.9 1')
    sub(light, 'direction', '-0.5 0.2 -1')
    sub(light, 'cast_shadows', 'true')
    ground = sub(world, 'model', name='ground')
    sub(ground, 'static', 'true')
    gl = sub(ground, 'link', name='ground_link')
    for typ in ('collision', 'visual'):
        geom = sub(gl, typ, name=typ)
        plane = sub(sub(geom, 'geometry'), 'plane')
        sub(plane, 'normal', '0 0 1')
        sub(plane, 'size', '50 50')
        if typ == 'visual':
            mat = sub(geom, 'material')
            sub(mat, 'ambient', '0.35 0.39 0.43 1')
            sub(mat, 'diffuse', '0.35 0.39 0.43 1')
    world.append(model)
    (ROOT/'worlds').mkdir(parents=True, exist_ok=True)
    write(sdf, ROOT/'worlds/spot.sdf')
    print(f'Generated Spot URDF and Gazebo world: {len(JOINT_NAMES)} actuated joints')


if __name__ == '__main__':
    main()
