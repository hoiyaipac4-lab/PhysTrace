import ast,numpy as np
def verified_physical_guard(error,native_reply,advance_in_progress):
    """Unknown runtime errors stay infrastructure failures, never policy scores."""
    if not advance_in_progress or not isinstance(native_reply,dict):return False
    if native_reply.get('recording_error'):return False
    snapshot=native_reply.get('failure_snapshot')
    if not isinstance(snapshot,dict) or not isinstance(snapshot.get('observed_after_error'),dict):return False
    message=error.strip().splitlines()[-1]
    known=('Native MuJoCo contact guard ','Native PyBullet v4 contact guard ','Drake native patch substep contact guard ')
    for prefix in known:
        marker='RuntimeError: '+prefix
        if message.startswith(marker):
            try:row=np.asarray(ast.literal_eval(message[len(marker):]),dtype=float)
            except Exception:return False
            return row.shape==(8,) and bool(np.isfinite(row).all()) and bool(row[0]>12 or max(row[1:3])>12 or row[3]>8 or row[4]>.002)
    if message.startswith('AssertionError: '):
        try:item=ast.literal_eval(message[len('AssertionError: '):])
        except Exception:return False
        if isinstance(item,tuple) and len(item)==4 and item[0]=='Native continuous visual clearance':
            _,minimum,tip,phase=item
            return bool(np.isfinite([minimum,tip]).all()) and (minimum<.005 or phase in ('close','grasp_hold') and tip<=.008)
    return False
