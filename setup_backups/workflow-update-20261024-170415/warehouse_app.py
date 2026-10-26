"""Local Isaac Sim 5.1 + Pegasus coal warehouse demo and acceptance checks."""
import argparse
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent
os.environ['PEGASUS_DISABLE_ROS2'] = '1'
parser = argparse.ArgumentParser()
parser.add_argument('--self-test', action='store_true')
parser.add_argument('--empty', action='store_true')
parser.add_argument('--workflow-test', action='store_true')
parser.add_argument('--scene', type=Path, default=ROOT/'coal_warehouse.usda')
args, kit_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + kit_args
PROJECT_PATH = args.scene.resolve()
TEST_MODE = args.self_test or args.workflow_test
LOGS = ROOT / 'logs'
LOGS.mkdir(exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({
    'headless': TEST_MODE, 'hide_ui': TEST_MODE, 'multi_gpu': False,
    'width': 960 if not TEST_MODE else 128,
    'height': 540 if not TEST_MODE else 128,
    'window_width': 1280, 'window_height': 800, 'limit_cpu_threads': 8,
    'renderer': 'RaytracedLighting', 'anti_aliasing': 3 if not TEST_MODE else 0,
    'fast_shutdown': True,
    'extra_args': ['--/rtx/post/dlss/execMode=0', '--/app/runLoops/main/rateLimitEnabled=true',
                   '--/app/runLoops/main/rateLimitFrequency=60'],
})

def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')

def setup():
    import numpy as np
    from pxr import UsdGeom, UsdPhysics, Gf
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import GroundPlane
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
    from pegasus.simulator.params import ROBOTS
    from manual_backend import ManualFlightBackend
    from scene_builder import build_scene
    from scene_storage import populate_stage, save_scene
    import omni.usd
    loaded = False
    if not TEST_MODE and not args.empty and PROJECT_PATH.exists():
        populate_stage(omni.usd.get_context().get_stage(), PROJECT_PATH)
        loaded = True
    pg = PegasusInterface()
    pg._world = World(physics_dt=1/120, rendering_dt=1/60, stage_units_in_meters=1,
                      backend='numpy', device='cpu')
    world = pg.world
    pc = world.get_physics_context()
    pc.enable_gpu_dynamics(False)
    pc.set_broadphase_type('MBP')
    pc.enable_fabric(False)
    pc.set_gravity(-9.81)
    if args.empty:
        world.scene.add(GroundPlane('/World/Ground', size=80))
        metadata = {'pile_count': 0}
    elif loaded:
        # Geometry comes from the saved USD, never from the generator on launch.
        meta_path = ROOT/'scene_metadata.json'
        metadata = json.loads(meta_path.read_text(encoding='utf-8')) if meta_path.exists() else {}
        metadata['loaded_from'] = str(PROJECT_PATH)
    else:
        metadata = build_scene(world.stage)
        if not TEST_MODE:
            save_scene(world.stage, PROJECT_PATH, stopped=True)
    metadata['scene_path'] = str(PROJECT_PATH)
    backend = ManualFlightBackend(max_altitude=6.5, horizontal_bounds=(-28,28,-19,19))
    config = MultirotorConfig()
    config.backends = [backend]
    config.sensors = []
    drone = Multirotor('/World/Drone', ROBOTS['Iris'], 0,
                      [-19.0,0.0,0.15], [0,0,0,1], config=config)
    # The official vehicle contributes small rotor masses in addition to body.
    masses = []
    for prim in world.stage.Traverse():
        if str(prim.GetPath()).startswith('/World/Drone') and prim.HasAPI(UsdPhysics.MassAPI):
            mass = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
            if mass and mass > 0:
                masses.append({'path':str(prim.GetPath()), 'kg':float(mass)})
    if masses:
        backend.mass = sum(item['kg'] for item in masses)
    metadata.update(drone_asset=ROBOTS['Iris'], drone_mass_kg=backend.mass, masses=masses,
                    physics_hz=120, controller='Pegasus custom Python backend', px4_required=False)
    save_json(LOGS/'runtime_metadata.json', metadata)
    world.reset()
    for _ in range(120):
        world.step(render=False)
    dc=drone.get_dc_interface()
    articulation=dc.get_articulation('/World/Drone')
    physical_masses=[]
    for index in range(dc.get_articulation_body_count(articulation)):
        body=dc.get_articulation_body(articulation,index)
        physical_masses.append(float(dc.get_rigid_body_properties(body).mass))
    backend.mass=sum(physical_masses)
    metadata['drone_mass_kg']=backend.mass
    metadata['physics_body_masses_kg']=physical_masses
    backend.ground_height = float(drone.state.position[2])
    metadata['resting_body_height_m'] = backend.ground_height
    save_json(LOGS/'runtime_metadata.json', metadata)
    return world, drone, backend, metadata

def test_flight(world, drone, backend, metadata):
    import numpy as np
    import omni.physx
    phases=[]
    trajectory=[]
    def phase(name, seconds, command=(0,0,0,0)):
        for i in range(round(seconds*120)):
            backend.set_velocity_command(*command)
            world.step(render=False)
            if i%24==0:
                t=backend.telemetry()
                trajectory.append({'phase':name, **t})
                assert not t['fault'], t['fault']
                assert np.all(np.isfinite(t['position_m'])), 'Nonfinite position'
        t=backend.telemetry()
        phases.append({'phase':name, **t})
        print('PHASE',name,json.dumps(t),flush=True)
        return t
    collision=[]
    if not args.empty:
        query=omni.physx.get_physx_scene_query_interface()
        for pile in metadata['piles']:
            x,y,_=pile['center']
            hit=query.raycast_closest((x,y,6.0),(0,0,-1),10.0)
            assert hit.get('hit') and 'Pile_' in str(hit.get('collision','')), str(hit)
            collision.append({'pile':pile['path'],'height_m':float(hit['position'][2]),'hit':True})
        hit=query.raycast_closest((0,0,6),(0,0,-1),10)
        assert hit.get('hit') and abs(hit['position'][2])<0.05, str(hit)
    backend.takeoff(2)
    t=phase('takeoff_and_hover',8)
    assert abs(t['position_m'][2]-2)<0.25, 'Takeoff height'
    assert np.linalg.norm(t['velocity_m_s'])<0.3, 'Hover velocity'
    start=np.array(t['position_m'])
    t=phase('forward',5,(1.5,0,0,0))
    assert t['position_m'][0]-start[0]>4, 'Forward travel'
    backend.hold()
    t=phase('brake_and_hold',3)
    assert np.linalg.norm(t['velocity_m_s'])<0.35, 'Hold velocity'
    t=phase('left',2,(0,1,0,0))
    assert t['position_m'][1]>0.9, 'Left travel'
    t=phase('right',2,(0,-1,0,0))
    backend.hold()
    phase('hold_before_yaw',2)
    t=phase('yaw_left',3,(0,0,0,0.5))
    assert t['yaw_deg']>60, 'Yaw rotation'
    t=phase('yaw_right',3,(0,0,0,-0.5))
    backend.hold()
    phase('hold_before_land',3)
    if not args.empty:
        # Traverse the main aisle and the gap between the two middle heaps.
        # These are test pilot velocity commands, using the same backend as keys.
        for name,goal in [('main_aisle',(0,0,2.2)),('north_pile_gap',(0,10,2.2)),
                          ('south_pile_gap',(0,-10,2.2)),('back_to_aisle',(0,0,2.2)),
                          ('far_exit',(22,0,2.2))]:
            goal=np.asarray(goal,dtype=float)
            arrived=False
            for i in range(60*120):
                t=backend.telemetry()
                error=goal-np.asarray(t['position_m'])
                velocity=np.clip(error*0.75,[-1.2,-1.2,-0.5],[1.2,1.2,0.5])
                yaw=math.radians(t['yaw_deg']); c,s=math.cos(yaw),math.sin(yaw)
                backend.set_velocity_command(c*velocity[0]+s*velocity[1],
                    -s*velocity[0]+c*velocity[1],velocity[2],0)
                world.step(render=False)
                if i%24==0:
                    trajectory.append({'phase':name,**backend.telemetry()})
                assert not backend.fault, backend.fault
                if np.linalg.norm(error)<0.30 and np.linalg.norm(t['velocity_m_s'])<0.25:
                    arrived=True
                    break
            assert arrived, f'Could not reach {name}: {backend.telemetry()}'
            backend.hold()
            phase(name+'_arrived',2)
    backend.land()
    t=phase('land',10)
    assert t['mode']=='GROUNDED', 'Did not land/disarm'
    assert abs(t['position_m'][2]-backend.ground_height)<0.12, 'Landing height'
    result={'ok':True,'empty_scene':args.empty,'metadata':metadata,
            'collision_checks':collision,'phases':phases}
    save_json(LOGS/('minimal_flight_result.json' if args.empty else 'acceptance_result.json'),result)
    save_json(LOGS/'flight_trajectory.json',trajectory)
    print('ACCEPTANCE_PASS',flush=True)

from flight_ui import FlightUI

try:
    world,drone,backend,metadata=setup()
    if args.workflow_test:
        from workflow_checks import run_workflow_checks
        run_workflow_checks(app, world, drone, backend, metadata, ROOT, LOGS)
    elif args.self_test:
        test_flight(world,drone,backend,metadata)
    else:
        ui=FlightUI(world,drone,backend,metadata,PROJECT_PATH,LOGS)
        print('WAREHOUSE_READY',flush=True)
        while app.is_running():
            ui.tick()
            world.step(render=True)
except Exception as exc:
    save_json(LOGS/'last_error.json',{'error':repr(exc),'traceback':traceback.format_exc()})
    traceback.print_exc()
    raise
finally:
    app.close()
