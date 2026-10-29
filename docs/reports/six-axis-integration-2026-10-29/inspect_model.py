"""Short CPU-only snapshot; uses existing MuJoCo/PyOpenGL, no installs.
Run with OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 LIBGL_ALWAYS_SOFTWARE=1
MESA_LOADER_DRIVER_OVERRIDE=llvmpipe LP_NUM_THREADS=1 MUJOCO_GL=egl python inspect_model.py.
"""
import hashlib
import json
import math
import struct
import zlib
from pathlib import Path
import mujoco as mj
from OpenGL.GL import GL_RENDERER, glGetString
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
model_path=ROOT/'assets/six_axis_arm.xml'
m=mj.MjModel.from_xml_path(str(model_path));d=mj.MjData(m)
home=mj.mj_name2id(m,mj.mjtObj.mjOBJ_KEY,'home')
mj.mj_resetDataKeyframe(m,d,home);mj.mj_forward(m,d)
meta={'mujoco_version':mj.__version__,'model_sha256':hashlib.sha256(model_path.read_bytes()).hexdigest(),
      'pose_source':'formal model home keyframe, no manual qpos override',
      'angles_deg':[math.degrees(q) for q in d.qpos],'home_bias_nm':d.qfrc_bias.tolist(),
      'body_mass_kg':m.body_mass.tolist(),'home_contacts':d.ncon,'simulation_steps':0}
cam=mj.MjvCamera();cam.lookat[:]=[.19,0,.32];cam.distance=1.12;cam.azimuth=105;cam.elevation=-14
meta['camera']={'lookat':cam.lookat.tolist(),'distance':cam.distance,'azimuth':cam.azimuth,'elevation':cam.elevation}
with mj.Renderer(m,height=600,width=800) as renderer:
    backend=glGetString(GL_RENDERER).decode()
    assert 'llvmpipe' in backend.lower() or 'softpipe' in backend.lower(),backend
    meta['renderer']=backend
    renderer.update_scene(d,camera=cam);pixels=renderer.render()
    def chunk(t,b):return struct.pack('>I',len(b))+t+b+struct.pack('>I',zlib.crc32(t+b)&0xffffffff)
    raw=b''.join(b'\0'+row.tobytes() for row in pixels)
    (HERE/'preview.png').write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',800,600,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))
a=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'wrist_roll_cylinder');b=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'tool_flange')
meta['wrist_clearance_samples']=[]
for angle in [-55,-52,-50,-45,35,45,50,52,55]:
    mj.mj_resetDataKeyframe(m,d,home);d.qpos[4]=math.radians(angle);mj.mj_forward(m,d)
    meta['wrist_clearance_samples'].append({'J5_degrees':angle,'distance_m':mj.mj_geomDistance(m,d,a,b,1,None),'engine_contacts':d.ncon})
meta['limitations']='Instantaneous geometry samples, including diagnostic angles outside final J5 limits; no proof of global collision freedom. Dynamic evidence is separate CTest log; Windows GUI keys not exercised.'
(HERE/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps({'renderer':meta['renderer'],'home_bias_nm':meta['home_bias_nm'],'home_contacts':meta['home_contacts']}))
print(HERE/'preview.png')
