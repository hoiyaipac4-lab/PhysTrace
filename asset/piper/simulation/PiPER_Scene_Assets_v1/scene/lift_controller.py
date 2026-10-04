"""Rate-limited tabletop height control; call tick before every physics step.

Works with a numpy omni.physics.tensors articulation view created after physics
initialization. Never teleports a rigid body or directly overwrites joint state.
"""
import numpy as np

class LiftController:
    def __init__(self, articulation, minimum=.635, maximum=1.235, speed=.04, acceleration=.08):
        self.view=articulation
        self.minimum,self.maximum=minimum,maximum
        self.speed,self.acceleration=speed,acceleration
        names=list(articulation.shared_metatype.dof_names)
        self.joints=[names.index(n) for n in ('LiftLower','LiftUpper')]
        q=articulation.get_dof_positions()[0]
        self.command=minimum+float(q[self.joints].sum())
        self.target=self.command;self.velocity=0.
        self.indices=np.arange(articulation.count,dtype=np.uint32)

    def set_height(self, metres):
        if not np.isfinite(metres):raise ValueError('Height must be finite')
        self.target=float(np.clip(metres,self.minimum,self.maximum))
        return self.target

    def stop(self):
        """Controlled deceleration, not a certified emergency-stop system."""
        self.target=float(np.clip(self.command+np.sign(self.velocity)*self.velocity**2/(2*self.acceleration),self.minimum,self.maximum))

    def measured_height(self):
        return self.minimum+float(self.view.get_dof_positions()[0,self.joints].sum())

    def tick(self, dt):
        if not np.isfinite(dt) or dt<=0:raise ValueError('dt must be positive and finite')
        error=self.target-self.command
        desired=np.sign(error)*min(self.speed,np.sqrt(2*self.acceleration*abs(error)))
        self.velocity+=float(np.clip(desired-self.velocity,-self.acceleration*dt,self.acceleration*dt))
        delta=self.velocity*dt
        if abs(delta)>abs(error) and delta*error>=0:delta=error;self.velocity=0.
        self.command=float(np.clip(self.command+delta,self.minimum,self.maximum))
        targets=self.view.get_dof_positions().copy()
        targets[:,self.joints]=(self.command-self.minimum)/2
        self.view.set_dof_position_targets(targets,self.indices)
        velocities=np.zeros_like(targets);velocities[:,self.joints]=self.velocity/2
        self.view.set_dof_velocity_targets(velocities,self.indices)
        return self.command
