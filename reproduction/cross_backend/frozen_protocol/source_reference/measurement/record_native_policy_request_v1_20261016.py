"""Lossless archive of the exact existing HTTP body; no pixel transformation."""
from pathlib import Path
import gzip,hashlib,json
def save_query_request(folder,seed,frame,body):
 folder=Path(folder);d=folder/'inference_inputs';d.mkdir(exist_ok=True);target=d/f'frame_{frame:06d}.npz.gz'
 compressed=gzip.compress(body,compresslevel=1,mtime=0);digest=hashlib.sha256(body).hexdigest();assert hashlib.sha256(gzip.decompress(compressed)).hexdigest()==digest
 with target.open('xb') as f:f.write(compressed)
 row={'seed':int(seed),'frame':int(frame),'file':str(target.relative_to(folder)),'original_body_bytes':len(body),'compressed_bytes':len(compressed),'original_body_sha256':digest,'compressed_sha256':hashlib.sha256(compressed).hexdigest(),'no_pixel_or_state_change':True}
 with (d/'manifest.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
 return row
