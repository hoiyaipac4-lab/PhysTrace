"""Anonymous snapshot syntax/manifest check; no Isaac Sim, no GPU, no code execution."""
from pathlib import Path
import argparse, ast, hashlib, json

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--full',action='store_true',help='Also hash all anonymous bundle files (requires git lfs pull).')
    args=parser.parse_args()
    scripts=list((ROOT/'simulation').rglob('*.py'))+list((ROOT/'tools').rglob('*.py'))
    for p in scripts:ast.parse(p.read_text(encoding='utf-8-sig'),filename=str(p.relative_to(ROOT)))
    cfg=json.loads((ROOT/'simulation/training_config.json').read_text(encoding='utf-8'))
    assert cfg['physics_hz']>0 and cfg['control_hz']>0 and cfg['physics_hz']%cfg['control_hz']==0
    assert len(cfg['home'])==8
    manifest=json.loads((ROOT/'docs/bundle_manifest.json').read_text(encoding='utf8'))
    missing=[];mismatch=[]
    for item in manifest['files']:
        p=ROOT/item['path']
        if not p.is_file():missing.append(item['path']);continue
        if args.full:
            if p.stat().st_size!=item['size']:mismatch.append(item['path']);continue
            with p.open('rb') as f:digest=hashlib.file_digest(f,'sha256').hexdigest()
            if digest!=item['sha256']:mismatch.append(item['path'])
    if missing or mismatch:
        raise SystemExit(json.dumps({'missing':missing,'mismatch':mismatch,'hint':'Run git lfs pull for pointer files; inspect deliberate local modifications.'},ensure_ascii=False,indent=2))
    print(json.dumps({'python_syntax_files':len(scripts),'bundle_files':len(manifest['files']),'full_hash_verification':args.full,'physics_hz':cfg['physics_hz'],'control_hz':cfg['control_hz'],'scope':'Static packaging check only; no dynamic or real-robot execution.'},indent=2))

if __name__=='__main__':main()
