"""Independent contact quadrature, road and time-resolution report.

A failed fit or convergence series remains in this artifact and prevents
promotion. Synthetic radial-fit checks do not count as field calibration.
"""
from pathlib import Path
from dataclasses import asdict
import argparse
import numpy as np
from bike_sim.validation.contact_manifold_rigs import (
    distributed_flat_rig,distributed_incline_rig,distributed_road_trace)
from bike_sim.physics.distributed_tire import HingeDensity,flat_load,fit_density
from bike_sim.validation.rider_replay import write_report
from bike_sim.validation.environment import environment_contract,source_fingerprint


def _events(rows,persistence_s=.005):
    events=[];accepted=None;pending=None;start=0.
    for r in rows:
        state='loaded' if r['normal_load_n']>10. else 'unloaded'
        if state!=pending:pending=state;start=r['time_s']
        if state!=accepted and r['end_time_s']-start+1e-12>=persistence_s:
            events.append({'state':state,'time_s':start});accepted=state
    return events


def contact_comparison(a,b):
    criteria={k:abs(a[k]-b[k])/max(abs(b[k]),.1)<=.02 for k in ('normal_impulse_ns','vertical_impulse_ns')}
    errors={k:abs(a[k]-b[k])/max(abs(b[k]),.1) for k in criteria}
    x,y=a['contact_events'],b['contact_events']
    matched=len(x)==len(y) and all(p['state']==q['state'] for p,q in zip(x,y))
    event_error=max((abs(p['time_s']-q['time_s']) for p,q in zip(x,y)),default=0.) if matched else None
    criteria['events']=matched and event_error<=.005
    criteria['energy']=max(a['energy_residual_ratio'],b['energy_residual_ratio'])<=.05
    criteria['no_native_contact']=a['native_wheel_contact_count']==b['native_wheel_contact_count']==0
    return {'passed':all(criteria.values()),'criteria':criteria,'relative_errors':errors,'event_error_s':event_error}


def run_contact_resolution(output_dir):
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    report={'schema_version':1,'source_sha256':source_fingerprint(Path(__file__).resolve().parents[1]),
        'environment':environment_contract(),'material':asdict(HingeDensity()),
        'synthetic':True,'series':[],'stand_checks':[],'comparisons':[],'passed':False}
    depths=np.linspace(.001,.012,12)
    for label,loads in (
        ('synthetic-original-linear-radial-130000',130000.*depths),
        ('synthetic-density-identifiability',flat_load(depths,.37,HingeDensity(),1024)[0])):
        density,fit=fit_density(depths,loads,.37,dataset_id=label)
        report.setdefault('fits',[]).append(fit)
    # The rejected legacy fit is NOT used to replace the declared synthetic law.
    report['material_selection']='unchanged declared synthetic density; no automatic promotion from fit'
    for n in (128,256,512):
        for load in (100.,300.,600.,1000.):
            metrics,bounds=distributed_flat_rig(.0003125,n,load)
            report['stand_checks'].append({'case':'flat_load','n':n,'load_n':load,'metrics':metrics,
                'passed':all(lo<=metrics[k]<=hi for k,(lo,hi) in bounds.items())})
    for angle in (5.,15.,30.):
        metrics,bounds=distributed_incline_rig(.0003125,256,angle)
        report['stand_checks'].append({'case':'incline_no_slip_fixture','angle_deg':angle,'metrics':metrics,
                'passed':all(lo<=metrics[k]<=hi for k,(lo,hi) in bounds.items())})
    variants={'time':[(dt,256,.005) for dt in (.00125,.000625,.0003125)],
              'road':[(.0003125,256,dx) for dx in (.01,.005,.0025)],
              'stations':[(.0003125,n,.005) for n in (128,256,512)]}
    for case in ('step','two_support','edge_unload','recontact','bump','incline'):
        lookup={}
        for axis,keys in variants.items():
            for dt,n,dx in keys:
                key=(dt,n,dx)
                if key in lookup:continue
                name=f'{case}_dt{dt:.7f}_dx{dx:g}_n{n}.json.gz'
                item={'case':case,'dt_s':dt,'station_count':n,'dx_m':dx,'file':name}
                try:
                    rows,metrics=distributed_road_trace(dt,n,dx,case=case)
                    metrics['contact_events']=_events(rows)
                    item.update(metrics=metrics,termination='duration_completed')
                    write_report(out/name,{'configuration':item.copy(),'rows':rows})
                except Exception as exc:item.update(metrics=None,termination='numerical_failure',error=f'{type(exc).__name__}: {exc}')
                lookup[key]=item;report['series'].append(item)
                write_report(out/'report.json',report)
            for i,j in ((0,1),(1,2),(0,2)):
                a,b=lookup[keys[i]],lookup[keys[j]]
                comparison={'case':case,'axis':axis,'coarse':a['file'],'reference':b['file'],'finest_pair':(i,j)==(1,2)}
                if a['metrics'] is not None and b['metrics'] is not None:comparison.update(contact_comparison(a['metrics'],b['metrics']))
                else:comparison.update(passed=False,reason='failed series')
                report['comparisons'].append(comparison)
        print(case,'contact series complete',flush=True)
    finest=[x for x in report['comparisons'] if x['finest_pair']]
    report['axes_passed']={axis:all(c['passed'] for c in finest if c['axis']==axis) for axis in variants}
    report['passed']=all(report['axes_passed'].values()) and all(x['passed'] for x in report['stand_checks'])
    report['status']='numerically_verified_synthetic_stands' if report['passed'] else 'experimental_gate_failed'
    write_report(out/'report.json',report);return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',default='results/contact_resolution')
    args=p.parse_args();r=run_contact_resolution(args.output);print(r['status'])
