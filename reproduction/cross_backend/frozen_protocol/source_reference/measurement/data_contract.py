"""Simulator-independent schema and native outcome gates. No learned-policy claims."""
import numpy as np
FPS=30
TASKS=('grasp','push','pull','pick_place','pick_rotate_place','pick_place_plate')
def encode(q):
    q=np.asarray(q);return np.concatenate((q[...,:6],np.clip(q[...,6:7]-q[...,7:8],0,.07)),axis=-1).astype(np.float32)
def relative(state, action):
    ref=state[0,:6].copy();s=state.copy();a=action.copy();s[:,:6]-=ref;a[:,:6]-=ref
    return s,a,ref
def outcome(plan, initial, final, final_yaw_change_deg, held_s, lift_s, released_s):
    delta=np.asarray(final)-initial;task=plan['task']
    if task=='grasp':return bool(held_s>.5 and lift_s>.5 and final[2]>.05)
    if task=='push':
        direction=next(x for x in ('forward','left','right') if f'cube {x} ' in plan['prompt'])
        v={'forward':np.array([1.,0]),'left':np.array([0.,1]),'right':np.array([0.,-1])}[direction]
        return bool(delta[:2]@v>.035 and abs(v[0]*delta[1]-v[1]*delta[0])<.025 and abs(delta[2])<.01 and released_s>.4)
    if task=='pull':return bool(delta[0]<-.035 and abs(delta[1])<.025 and abs(final[2]-.0125)<.01 and held_s>.5 and released_s>.4)
    ok=np.linalg.norm(np.asarray(final[:2])-plan['target_xy'])<.018 and released_s>.5 and lift_s>.5
    if task=='pick_rotate_place':ok=ok and abs(final_yaw_change_deg-plan['requested_yaw_change_deg'])<8
    return bool(ok)
def validate_arrays(state,action,stamps,head_shape,wrist_shape):
    assert state.shape==action.shape and state.shape[1]==7 and len(state)>90
    assert np.isfinite(state).all() and np.isfinite(action).all() and np.isfinite(stamps).all()
    assert np.allclose(np.diff(stamps),1/FPS,atol=1e-8)
    assert head_shape==wrist_shape and len(head_shape)==3 and head_shape[2]==3
    assert np.all((state[:,6]>=0)&(state[:,6]<=.070001))
