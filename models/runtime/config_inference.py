# Snapshot of original inference config; unused training initializer replaced by NoOp.
from pathlib import Path
import dataclasses
import flax.nnx as nnx
from openpi import transforms
from openpi.policies import libero_policy
from openpi.training import config, optimizer, weight_loaders
from openpi.shared import nnx_utils
from openpi.models.pi0_config import Pi0Config
ROOT=Path(__file__).resolve().parents[1]
OUT=Path(__file__).resolve().parent
MASK=(True,True,True,True,True,True,False)
@dataclasses.dataclass(frozen=True)
class AbsoluteGraspData(config.DataConfigFactory):
 def create(self, assets_dirs, model_config):
  return dataclasses.replace(self.create_base_config(assets_dirs,model_config),
   repack_transforms=transforms.Group(inputs=[transforms.RepackTransform({'observation/image':'observation.images.head','observation/wrist_image':'observation.images.hand','observation/state':'observation.state_absolute','actions':'action_waypoint','prompt':'prompt'})]),
   data_transforms=transforms.Group(inputs=[libero_policy.LiberoInputs(model_type=model_config.model_type)],outputs=[libero_policy.LiberoOutputs()]),
   model_transforms=config.ModelTransformFactory()(model_config),action_sequence_keys=('action_waypoint',),prompt_from_task=True)
def make_config(resume=False):
 model=Pi0Config(pi05=True,action_horizon=16,paligemma_variant='gemma_2b_lora',action_expert_variant='gemma_300m_lora')
 return dataclasses.replace(config.get_config('pi05_aloha'),name='piper_three_tasks',exp_name='warmstart_v1',model=model,
  data=AbsoluteGraspData(repo_id=str(ROOT/'prepared/train'),assets=config.AssetsConfig(assets_dir=str(ROOT/'prepared/assets'),asset_id='piper_three_tasks')),
  freeze_filter=nnx.Not(nnx_utils.PathRegex('.*(lora|action_in_proj|action_out_proj|time_mlp).*')),ema_decay=None,
  weight_loader=weight_loaders.NoOpWeightLoader(),
  batch_size=4,num_train_steps=15000,num_workers=0,seed=20261005,fsdp_devices=1,wandb_enabled=False,
  checkpoint_base_dir=str(OUT/'runs/checkpoints'),assets_base_dir=str(ROOT/'prepared/assets'),overwrite=False,resume=resume,log_interval=25,save_interval=1000,keep_period=1000,
  lr_schedule=optimizer.CosineDecaySchedule(warmup_steps=100,peak_lr=1e-5,decay_steps=15000,decay_lr=2.5e-6),
  policy_metadata={'simulator':'isaacsim','tasks':['grasp_only','lift_only','grasp_lift'],'observation_encoding':'absolute_joint_rad_full_width_m','model_action_encoding':'absolute_phase_endpoint_joint_rad_full_width_m','server_action_encoding':'absolute_joint_rad_full_width_m','fps':30,'horizon':16,'real_robot_ready':False})
