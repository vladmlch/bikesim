"""One-factor-at-a-time human replay: retain all failures and diagnoses."""
from pathlib import Path
import argparse
from bike_sim.validation.rider_replay import run_replay,write_report


def run_matrix(output_dir, *, physics_profile=None):
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    base=Path(__file__).resolve().parents[3]/'examples/research'
    variants=[('human_flat','human-only','rider_resume_flat.toml',None),
              ('human_incline','human-only','rider_resume_incline.toml',None),
              ('motor_40','motor40','rider_resume_flat.toml',None),
              ('assist','assist','rider_resume_flat.toml',None),
              ('shifting','shifting','rider_resume_flat.toml',None),
              ('rollback','rollback','rider_resume_incline.toml',None),
              ('active_tau0','human-only','rider_resume_flat.toml',0.),
              ('active_tau005','human-only','rider_resume_flat.toml',.05),
              ('motor_open_loop','open-loop','rider_resume_flat.toml',None)]
    summary={'schema_version':1,'duration_s':10.,'resume_window_s':[4.,10.],
             'synthetic':True,'anti_wheelie_policy_used':False,'variants':[]}
    for name,mode,track,tau in variants:
        physics='plant_reference_open_loop.toml' if mode=='open-loop' else 'rider_resume_physics.toml'
        file=name+'.json.gz'
        result=run_replay(base/physics if physics_profile is None else physics_profile,
            base/track,duration_s=10.,mode=mode,activation_tau_s=tau,output=out/file)
        summary['variants'].append({k:v for k,v in result.items() if k!='rows'}|{'name':name,'file':file})
        write_report(out/'report.json',summary)
        print(name,result.get('evidence',{}).get('diagnosis',result.get('error')),flush=True)
    reference=[r for r in summary['variants'] if r['name'] in ('human_flat','human_incline')]
    # Unique diagnosed limitations are explicit outcomes, not claimed pedaling successes.
    summary['baseline_resumed']=all(r.get('evidence',{}).get('diagnosis')=='resumed' for r in reference)
    summary['causal_replay_resolved']=all(r.get('evidence',{}).get('diagnosis') in
        ('resumed','support_unavailable','actuator_saturated','assist_gated','mechanically_stalled')
        and r.get('completed_requested_duration') is True for r in reference)
    write_report(out/'report.json',summary);return summary

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',default='results/rider_factors')
    p.add_argument('--physics-config')
    a=p.parse_args();run_matrix(a.output, physics_profile=a.physics_config)
