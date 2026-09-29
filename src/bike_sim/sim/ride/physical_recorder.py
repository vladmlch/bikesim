"""Schema-2 recorder: immutable solved intervals, not a second force calculator."""
import csv
import json
from pathlib import Path
from collections.abc import Mapping
import numpy as np
from bike_sim.sim.ride.physical_samples import plain


def flatten_numbers(value,prefix=''):
    result={}
    if isinstance(value,Mapping):
        for key,item in value.items():
            result.update(flatten_numbers(item,f'{prefix}.{key}' if prefix else str(key)))
    elif isinstance(value,(list,tuple,np.ndarray)):
        for i,item in enumerate(value):
            result.update(flatten_numbers(item,f'{prefix}.{i}'))
    elif isinstance(value,(bool,int,float,np.number)):
        result[prefix]=float(value)
    return result


class PhysicalRecorder:
    schema_version=2

    def __init__(self,sim,decimate=1):
        if sim.physical.interactive_preview or not sim.physical.research_accounting_valid:
            raise ValueError('interactive preview has no research force intervals')
        if isinstance(decimate,bool) or not isinstance(decimate,int) or decimate<1:
            raise ValueError('decimate must be a positive integer')
        self.sim=sim
        self.decimate=decimate
        self.timestep_s=float(sim.model.opt.timestep)
        self.samples=[]
        self._last_id=None
        self._generation=sim.physical.generation

    def record(self,sim):
        if sim.physical.interactive_preview or not sim.physical.research_accounting_valid:
            raise ValueError('interactive preview has no research force intervals')
        if sim is not self.sim:
            raise ValueError('recorder belongs to a different simulation')
        if sim.physical.generation != self._generation:
            raise ValueError('reset requires a new recorder')
        sample=sim.physical.sample
        if sample is None:
            return  # Initial state has no solved force interval.
        constraints=sim.last_constraint_snapshot
        if constraints is not None and (
            abs(constraints.interval_start_s-sample.time_s)>1e-12
            or abs(constraints.interval_end_s-sample.end_time_s)>1e-12
            or not np.array_equal(constraints.qvel_start,sample.qvel)
        ):
            raise ValueError('constraint snapshot does not match the recorded physical interval')
        if self._last_id is not None and sample.interval_id<self._last_id:
            raise ValueError('reset requires a new recorder')
        if sample.interval_id==self._last_id:
            return
        self._last_id=sample.interval_id
        if sample.interval_id%self.decimate==0:
            self.samples.append(sample)

    @property
    def rows(self):
        return len(self.samples)

    @property
    def sample_interval_s(self):
        return self.timestep_s*self.decimate

    @property
    def component_work_j(self):
        return dict(self.sim.physical.history.work_j)

    @property
    def last_component_powers_w(self):
        sample=self.sim.physical.sample
        return {} if sample is None else sample.powers_w

    def _row(self,sample):
        row=flatten_numbers(sample.as_dict())
        row['x_m']=float(sample.qpos[self.sim.root_x_qposadr])
        row['speed_mps']=float(sample.qvel[self.sim.root_x_dofadr])
        row['pitch_rad']=float(sample.qpos[self.sim.root_pitch_qposadr])
        # Explicit aliases preserve the early schema-2 force-sample API.
        row['interval_start_s']=sample.time_s
        row['interval_dt_s']=sample.dt_s
        for i,value in enumerate(sample.qpos): row[f'prestep_qpos_{i}']=float(value)
        for i,value in enumerate(sample.qvel): row[f'prestep_qvel_{i}']=float(value)
        for name,power in sample.powers_w.items():
            row['power_'+name+'_w']=power
            row[name+'_power_w']=power
        for name,work in sample.channels.get('component_work_j',{}).items():row[name+'_work_j']=float(work)
        mass=sample.channels['mass']
        row['compiled_mass_kg']=float(mass['mass_kg'])
        for i,axis in enumerate('xyz'):row[f'compiled_com_{axis}_m']=float(mass['com_m'][i])
        row['kinetic_energy_j']=float(mass['kinetic_energy_j'])
        row['gravitational_energy_j']=float(mass['gravitational_energy_j'])
        row['mechanical_energy_j']=float(sample.channels['energy']['mechanical_energy_j'])
        row['energy_residual_j']=float(sample.channels['energy']['residual_j'])
        return row

    def columns(self):
        rows=[self._row(s) for s in self.samples]
        names=sorted(set().union(*(row.keys() for row in rows))) if rows else []
        return {name:np.array([row.get(name,np.nan) for row in rows]) for name in names}

    def column(self,name):
        return self.columns()[name]

    def array(self):
        columns=self.columns()
        return np.column_stack(list(columns.values())) if columns else np.empty((0,0))

    def write_csv(self,path):
        path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
        rows=[self._row(s) for s in self.samples]
        names=sorted(set().union(*(row.keys() for row in rows))) if rows else ['schema_version','time_s']
        with path.open('w',newline='',encoding='utf-8') as stream:
            writer=csv.DictWriter(stream,fieldnames=names)
            writer.writeheader();writer.writerows(rows)
        return path

    def write_jsonl(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('w',encoding='utf-8') as stream:
            for sample in self.samples:
                stream.write(json.dumps(sample.as_dict(),sort_keys=True,allow_nan=False)+'\n')
        return path
