from pathlib import Path
import argparse,json,numpy as np,gzip,subprocess
p=argparse.ArgumentParser();p.add_argument('--config',type=Path);p.add_argument('--resolution',nargs=2,type=int,default=[320,240]);p.add_argument('--repeats',type=int,default=2);p.add_argument('--seconds',type=float,default=3);p.add_argument('--output-dir',type=Path);args=p.parse_args()
root=Path(__file__).resolve().parent;out=args.output_dir or root/'repair_evidence/training';out.mkdir(parents=True,exist_ok=True)
cfg=json.loads(args.config.read_text()) if args.config else {};cfg['resolution']=args.resolution
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'active_gpu':cfg.get('gpu',0),'physics_gpu':cfg.get('gpu',0),'max_gpu_count':1,'width':320,'height':240,'renderer':'RayTracedLighting','extra_args':['--portable-root',str(root/'.runtime'),'--reset-user']})
env=None
try:
 from training_env import TrainingScene
 from PIL import Image
 env=TrainingScene(app,cfg);runs=[];starts=[];images=[]
 for episode in range(args.repeats):
  if episode:env.reset(0)
  starts.append(env.state());obs=env.observe();image_info={}
  for name,camera in obs['cameras'].items():
   Image.fromarray(camera['rgb']).save(out/f'{episode}_{name}_initial.png');Image.fromarray(camera['model_rgb']).save(out/f'{episode}_{name}_model.png');np.save(out/f'{episode}_{name}_depth.npy',camera['depth_m'])
   image_info[name]={k:v for k,v in camera.items() if k not in ['rgb','depth_m','depth_valid','model_rgb']};image_info[name].update(rgb_shape=list(camera['rgb'].shape),depth_valid_fraction=float(camera['depth_valid'].mean()))
  trace=[]
  with gzip.open(out/f'{episode}_physics.jsonl.gz','wt') as log:
   for i in range(round(args.seconds*env.cfg['control_hz'])):
    t=i/env.cfg['control_hz'];action=np.r_[env.cfg['home'][:6],.04];action[0]+=.06*np.sin(np.pi*t/args.seconds)
    state=env.step(action,appliances={'airfryer':.12 if .1<t<1.6 else 0,'microwave':70 if .1<t<1.6 else 0},record_contacts=True)
    # Direct native appliance position readback, not target echoed back.
    state['appliances']={}
    for n in ['airfryer','microwave']:
     prim=next(p for p in __import__('pxr.Usd',fromlist=['PrimRange']).PrimRange(env.stage.GetPrimAtPath('/World/TaskAssets/'+n)) if p.HasAPI(__import__('pxr.UsdPhysics',fromlist=['ArticulationRootAPI']).ArticulationRootAPI))
     view=env.sv.create_articulation_view(str(prim.GetPath()));state['appliances'][n]=float(view.get_dof_positions()[0,0])
    trace.append(state)
    for frame in env.frames:log.write(json.dumps(frame)+'\n')
    if i%30==0:print('TRAINING_STEP',episode,i,state['time_s'],flush=True)
  obs2=env.observe()
  for name,camera in obs2['cameras'].items():Image.fromarray(camera['rgb']).save(out/f'{episode}_{name}_final.png')
  runs.append({'initial_cameras':image_info,'states':trace,'physics_stats':__import__('omni.physx',fromlist=['get_physxunittests_interface']).get_physxunittests_interface().get_physics_stats()})
 errors=[float(np.max(np.abs(np.array(x['joint_q'])-starts[0]['joint_q']))) for x in starts]
 assert max(errors)<1e-4,errors
 object_errors={name:max(float(np.max(np.abs(np.array(x['objects'][name]['xyz_xyzw'])-starts[0]['objects'][name]['xyz_xyzw']))) for x in starts) for name in starts[0]['objects']}
 assert max(object_errors.values())<1e-4,object_errors
 summary={'status':'PASS','repeats':len(runs),'seconds_each':args.seconds,'reset_joint_max_error':max(errors),'image_resolution':args.resolution,'depth_and_rgb':True,'physics_and_image_time_aligned':True,'runs':runs,'initial_states':starts,'scope':'Simulation interface, cameras, contact recording, reset, small joint actions and appliance controls. Not VLA training or real robot calibration.'}
 summary['reset_object_pose_max_component_error']=object_errors
 (out/'validation.json').write_text(json.dumps(summary,indent=2));print('TRAINING_INTERFACE_PASS',flush=True)
except BaseException as error:
 import traceback
 traceback.print_exc();(out/'last_failure.json').write_text(json.dumps({'status':'FAIL','error':repr(error),'traceback':traceback.format_exc()},indent=2));print('TRAINING_INTERFACE_FAILED',repr(error),flush=True)
 raise
finally:
 if env:env.close()
 app.close()
