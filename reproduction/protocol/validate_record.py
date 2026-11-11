"""Check the documented cradle record fields and units."""
import argparse
import json
import math
from pathlib import Path
import re

def validate(record):
    errors=[]
    try:
        identity=record['identity']; c=record['condition']; a=record['actual_observation']; i=record['applied_input']
        for name in ['task','case','platform','package_version','version_provenance','record_kind']:
            if not identity.get(name):errors.append('identity:'+name)
        for k,v in {'length':'m','mass':'kg','time':'s','angle':'rad','force':'N','torque':'N m'}.items():
            if c['units'].get(k)!=v:errors.append('unit:'+k)
        for k in ['radius_m','pendulum_length_m','ball_mass_kg','physics_hz','dt_s','duration_s']:
            if not isinstance(c.get(k),(float,int)) or not math.isfinite(c[k]) or c[k]<=0: errors.append('positive:'+k)
        if abs(c['physics_hz']*c['dt_s']-1)>1e-10:errors.append('clock:dt_hz')
        clock=a['clock']
        if clock['samples']!=clock['intervals']+1:errors.append('clock:count')
        if abs(clock['intervals']*c['dt_s']-c['duration_s'])>1e-8:errors.append('clock:duration')
        if abs(clock['end_s']-clock['start_s']-c['duration_s'])>1e-8:errors.append('clock:range')
        if c['gravity_m_s2']!=i['prescribed_gravity_m_s2']:errors.append('input:gravity')
        if len(c['initial_com_positions_m'])!=len(c['joint_angles_rad']):errors.append('state:shape')
        if i['kind']=='passive release' and (i['post_initialization_pose_forcing'] or i['motor_force_limit_Nm']!=0):errors.append('input:passive')
        if a['contact_raw_available_in_local_archive'] and not a.get('raw_contact_file'):
            errors.append('evidence:contact_raw_missing')
        for x in record['sources']:
            if not re.fullmatch('[0-9a-f]{64}',x['sha256']):errors.append('source:hash')
        if not record['sources']:errors.append('source:empty')
    except (KeyError,TypeError,ValueError) as e:errors.append('schema:'+str(e))
    return errors

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record", type=Path, nargs="?",
                        default=Path(__file__).resolve().with_name("example_cradle_record.json"))
    args = parser.parse_args()
    errors = validate(json.loads(args.record.read_text(encoding="utf-8")))
    print(json.dumps({"valid": not errors, "errors": errors}))
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
