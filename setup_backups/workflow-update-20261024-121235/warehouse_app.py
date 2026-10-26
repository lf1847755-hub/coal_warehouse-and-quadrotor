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
args, kit_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + kit_args
LOGS = ROOT / 'logs'
LOGS.mkdir(exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({
    'headless': args.self_test, 'hide_ui': args.self_test, 'multi_gpu': False,
    'width': 960 if not args.self_test else 128,
    'height': 540 if not args.self_test else 128,
    'window_width': 1280, 'window_height': 800, 'limit_cpu_threads': 8,
    'renderer': 'RaytracedLighting', 'anti_aliasing': 3 if not args.self_test else 0,
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
    from isaacsim.core.utils.stage import save_stage
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
    from pegasus.simulator.params import ROBOTS
    from manual_backend import ManualFlightBackend
    from scene_builder import build_scene
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
    else:
        metadata = build_scene(world.stage)
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
    save_json(ROOT/'scene_metadata.json', metadata)
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
    save_json(ROOT/'scene_metadata.json', metadata)
    if not args.empty:
        save_stage(str(ROOT/'coal_warehouse.usda'), save_and_reload_in_place=False)
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

class FlightUI:
    def __init__(self,world,drone,backend,metadata):
        import carb
        import omni.ui as ui
        import omni.appwindow
        self.world,self.drone,self.backend,self.metadata=world,drone,backend,metadata
        self.carb=carb
        self.camera='follow'
        self.roof_visible=True
        self.reset_pending=False
        self.last_log=0
        self.frame=0
        self.last_key=''
        self.key_count=0
        self._input=carb.input.acquire_input_interface()
        self._appwindow=omni.appwindow.get_default_app_window()
        self._keyboard=self._appwindow.get_keyboard()
        self._sub=self._input.subscribe_to_keyboard_events(self._keyboard,self.key_event)
        self.window=ui.Window('Coal Warehouse - Manual Flight',width=330,height=465)
        self.window.position_x=930
        self.window.position_y=80
        for name in ('Content', 'Console'):
            editor_window=ui.Workspace.get_window(name)
            if editor_window:
                editor_window.visible=False
        with self.window.frame:
            with ui.VStack(spacing=7,style={'font_size':17}):
                ui.Label('COAL WAREHOUSE',height=30,style={'font_size':23,'color':0xFF66D9F2})
                ui.Label('Isaac Sim 5.1 + Pegasus 5.1',height=22)
                with ui.HStack(height=35,spacing=5):
                    ui.Button('Takeoff [T]',clicked_fn=lambda:backend.takeoff(2))
                    ui.Button('Hold [H]',clicked_fn=backend.hold)
                    ui.Button('Land [L]',clicked_fn=backend.land)
                ui.Label('W / S    Forward / Backward',height=23)
                ui.Label('A / D    Left / Right',height=23)
                ui.Label('Q / E    Turn left / right',height=23)
                ui.Label('Up / Down    Ascend / Descend',height=23)
                ui.Label('Release keys to hover. Use English input.',height=23,style={'font_size':14})
                with ui.HStack(height=32,spacing=5):
                    ui.Button('Follow [F]',clicked_fn=lambda:self.set_camera('follow'))
                    ui.Button('Overview [O]',clicked_fn=lambda:self.set_camera('overview'))
                with ui.HStack(height=32,spacing=5):
                    ui.Button('Roof view [V]',clicked_fn=self.toggle_roof)
                    ui.Button('Reset [R]',clicked_fn=self.request_reset)
                self.status=ui.Label('Ready - press T to take off',height=70,word_wrap=True)
                ui.Label('Speed 1.5 m/s  |  Altitude limit 6.5 m',height=23,style={'font_size':14})
        self.update_camera()
    def request_reset(self):
        self.reset_pending=True
    def set_camera(self,mode):
        self.camera=mode
        self.update_camera()
    def toggle_roof(self):
        from pxr import UsdGeom
        self.roof_visible=not self.roof_visible
        p=self.world.stage.GetPrimAtPath('/World/CoalWarehouse/Roof')
        UsdGeom.Imageable(p).GetVisibilityAttr().Set('inherited' if self.roof_visible else 'invisible')
    def key_event(self,event,*unused):
        key_name=getattr(event.input, 'name', str(event.input)).upper()
        if event.type==self.carb.input.KeyboardEventType.KEY_PRESS:
            self.last_key=key_name
            self.key_count+=1
            actions={'T':lambda:self.backend.takeoff(2),'H':self.backend.hold,'L':self.backend.land,
                     'R':self.request_reset,'F':lambda:self.set_camera('follow'),
                     'O':lambda:self.set_camera('overview'),'V':self.toggle_roof}
            if key_name in actions:
                actions[key_name]()
                return False
        if key_name in ('W','A','S','D','Q','E','UP','DOWN'):
            return False
        return True
    def update_camera(self):
        import numpy as np
        from isaacsim.core.utils.viewports import set_camera_view
        if self.camera=='overview':
            eye=[-35,-31,20]; target=[0,0,1.5]
        else:
            p=np.asarray(self.drone.state.position)
            yaw=math.radians(self.backend.telemetry()['yaw_deg'])
            direction=np.array([math.cos(yaw),math.sin(yaw),0])
            eye=p-4.5*direction+np.array([0,0,2.1])
            target=p+3.0*direction+np.array([0,0,0.35])
        set_camera_view(eye=np.asarray(eye),target=np.asarray(target))
    def tick(self):
        import numpy as np
        if self.reset_pending or self.world.is_stopped():
            self.world.reset()
            for _ in range(60): self.world.step(render=False)
            self.backend.ground_height=float(self.drone.state.position[2])
            self.reset_pending=False
        k=self.carb.input.KeyboardInput
        def pressed(name):
            return float(self._input.get_keyboard_value(self._keyboard,getattr(k,name))>0.5)
        command=((pressed('W')-pressed('S'))*1.5,(pressed('A')-pressed('D'))*1.5,
                 (pressed('UP')-pressed('DOWN'))*0.7,(pressed('Q')-pressed('E'))*0.6)
        if not self._appwindow.is_focused():
            command=(0,0,0,0)
        self.backend.set_velocity_command(*command)
        if self.frame%2==0:self.update_camera()
        self.frame+=1
        t=self.backend.telemetry()
        p=t['position_m']; speed=float(np.linalg.norm(t['velocity_m_s']))
        self.status.text=f"{t['mode']}  |  Height {p[2]:.2f} m\nSpeed {speed:.2f} m/s  |  Heading {t['yaw_deg']:.0f} deg\n{t['fault'] or 'Ready for manual flight'}"
        if time.monotonic()-self.last_log>1:
            save_json(LOGS/'live_status.json',{'scene_ready':True,**t,'camera':self.camera,
                'last_key':self.last_key,'key_press_count':self.key_count,
                'window_focused':self._appwindow.is_focused(),'manual_command':command})
            self.last_log=time.monotonic()

try:
    world,drone,backend,metadata=setup()
    if args.self_test:
        test_flight(world,drone,backend,metadata)
    else:
        ui=FlightUI(world,drone,backend,metadata)
        print('WAREHOUSE_READY',flush=True)
        while app.is_running():
            ui.tick()
            world.step(render=True)
except Exception as exc:
    save_json(LOGS/'last_error.json',{'error':repr(exc),'traceback':traceback.format_exc()})
    traceback.print_exc()
finally:
    app.close()
