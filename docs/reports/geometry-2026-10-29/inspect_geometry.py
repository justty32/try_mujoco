"""Read-only MuJoCo geometry sampling; optional small software-rendered previews."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import zlib

import mujoco

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
JOINTS = ('yaw1', 'pitch1', 'yaw2', 'pitch2', 'yaw3', 'pitch3')


def samples():
    yield 'zero', [0.0] * 6, True
    for j, name in enumerate(JOINTS):
        limit = 90 if name.startswith('yaw') else 60
        for angle in sorted({-limit, -60, -30, -10, 10, 30, 60, limit}):
            q = [0.0] * 6
            q[j] = angle
            yield f'{name}_{angle:+d}', q, abs(angle) <= 30
    yield 'mixed_a', [20, 25, -15, 15, 10, 20], True
    yield 'mixed_b', [-25, 40, 20, 10, -15, 15], True


def place(model, data, degrees):
    mujoco.mj_resetData(model, data)
    for name, angle in zip(JOINTS, degrees):
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[j]] = math.radians(angle)
    mujoco.mj_forward(model, data)


def inspect(path):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    results = []
    for label, degrees, required in samples():
        place(model, data, degrees)
        penetration = []
        # Explicit cross-body distance probes include parent-child geoms;
        # engine contact filtering must not hide an assembly intersection.
        for a in range(model.ngeom):
            for b in range(a + 1, model.ngeom):
                if model.geom_bodyid[a] == model.geom_bodyid[b]:
                    continue
                dist = float(mujoco.mj_geomDistance(model, data, a, b, 0.05, None))
                if dist < -1e-6:
                    penetration.append({
                        'a': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, a),
                        'b': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, b),
                        'distance_m': dist,
                    })
        results.append({'name': label, 'degrees': degrees, 'required_clear': required,
                        'engine_contacts': data.ncon, 'penetrations': penetration})
    place(model, data, [0] * 6)
    return {'path': str(path.relative_to(ROOT)),
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'body_mass_kg': model.body_mass.tolist(),
            'zero_bias_nm': data.qfrc_bias.tolist(), 'samples': results}


def write_png(path, rgb):
    """Encode unmodified renderer pixels using the standard library."""
    h, w, _ = rgb.shape
    def chunk(kind, payload):
        crc = zlib.crc32(kind + payload) & 0xffffffff
        return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', crc)
    rows = b''.join(b'\0' + rgb[y].tobytes() for y in range(h))
    path.write_bytes(b'\x89PNG\r\n\x1a\n'
                     + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                     + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))


def render(path, label):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    place(model, data, [0] * 6)
    with mujoco.Renderer(model, height=400, width=600) as renderer:
        from OpenGL.GL import GL_RENDERER, glGetString
        backend = glGetString(GL_RENDERER).decode()
        if not any(token in backend.lower() for token in ('llvmpipe', 'softpipe', 'software')):
            raise RuntimeError(f'Expected software rendering, got {backend}')
        print('OpenGL renderer:', backend)
        views = [('overview', [0] * 6, False), ('joint', [0] * 6, True),
                 ('pose', [20, 25, -15, 15, 10, 20], False)]
        metadata = []
        for suffix, angles, detail in views:
            place(model, data, angles)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [.22, 0, .30] if not detail else [.20, 0, .35]
            cam.distance = 1.15 if not detail else .42
            if suffix == 'pose':
                cam.lookat[:] = [.17, 0, .43]
                cam.distance = 1.65
            cam.azimuth = 135
            cam.elevation = -22
            renderer.update_scene(data, camera=cam)
            write_png(HERE / f'{label}-{suffix}.png', renderer.render())
            metadata.append({'file': f'{label}-{suffix}.png', 'degrees': angles,
                             'lookat': cam.lookat.tolist(), 'distance': cam.distance,
                             'azimuth': cam.azimuth, 'elevation': cam.elevation})
        (HERE / f'{label}-render.json').write_text(json.dumps({
            'renderer': backend, 'width': 600, 'height': 400,
            'model_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'views': metadata}, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args()
    paths = {'before': HERE / 'before.xml', 'after': HERE / 'after.xml'}
    report = {'mujoco_version': mujoco.__version__, 'distance_tolerance_m': 1e-6,
              'method': 'instantaneous forward kinematics, no simulation steps',
              'models': {name: inspect(path) for name, path in paths.items()}}
    (HERE / 'geometry_samples.json').write_text(json.dumps(report, indent=2) + '\n')
    for name, model in report['models'].items():
        bad = [s['name'] for s in model['samples'] if s['penetrations']]
        required_bad = [s['name'] for s in model['samples'] if s['required_clear'] and s['penetrations']]
        print(name, 'samples', len(model['samples']), 'intersecting', len(bad),
              'required_clear_failures', required_bad)
        if name == 'after':
            print('large-angle intersections:', [n for n in bad if n not in required_bad])
    if args.render:
        for name, path in paths.items():
            render(path, name)
        print('Saved six 600x400 actual MuJoCo software renders')
    if any(s['required_clear'] and s['penetrations']
           for s in report['models']['after']['samples']):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
