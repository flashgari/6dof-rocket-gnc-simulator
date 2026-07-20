#!/usr/bin/env python3
import csv, math, sys
from pathlib import Path
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from rocket_sim import Environment, RocketParams, State
from rocket_sim.analysis import body_z_axis_inertial, summary_metrics, tilt_angle_deg, lateral_displacement_m, signed_pitch_deg, signed_yaw_deg
from rocket_sim.math3d import q_norm
from rocket_sim.sim import simulate

def write_csv(path, samples):
    raw_pitch = [signed_pitch_deg(s) for _, s in samples]
    unwrapped_pitch = []
    offset = 0.0
    previous = raw_pitch[0]
    for pitch in raw_pitch:
        delta = pitch - previous
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        unwrapped_pitch.append(pitch + offset)
        previous = pitch

    with path.open('w', newline='') as f:
        w=csv.writer(f); w.writerow(['time_s','x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','qw','qx','qy','qz','wx_radps','wy_radps','wz_radps','q_norm','tilt_deg','signed_pitch_deg','unwrapped_pitch_deg','signed_yaw_deg','body_z_x','body_z_y','body_z_z','lateral_m'])
        for (pitch_unwrapped, (t,s)) in zip(unwrapped_pitch, samples):
            body_z=body_z_axis_inertial(s)
            w.writerow([t,*s.as_tuple(),q_norm(s.attitude),tilt_angle_deg(s),signed_pitch_deg(s),pitch_unwrapped,signed_yaw_deg(s),*body_z,lateral_displacement_m(s)])

def week2_setup():
    misalign=math.radians(1.5)
    rocket=RocketParams(mass_kg=50.0,inertia_kg_m2=(3.0,3.0,0.45),thrust_n=850.0,reference_area_m2=0.045,drag_coefficient=0.35,normal_force_coefficient_per_rad=2.5,center_of_pressure_body_m=(0,0,0.35),thrust_offset_body_m=(0.004,0,0),thrust_direction_body=(math.sin(misalign),0,math.cos(misalign)))
    return rocket, Environment(wind_mps=(4.0,1.0,0.0)), State((0,0,0),(0,0,0),(1,0,0,0),(0,0,0))

def main():
    rocket,env,initial=week2_setup(); samples=list(simulate(initial,rocket,env,3.0,0.005)); out=PROJECT_ROOT/'outputs'; out.mkdir(exist_ok=True); path=out/'week2_disturbed_uncontrolled.csv'; write_csv(path,samples)
    m=summary_metrics(samples,rocket,env); print(f'Wrote {len(samples)} samples to {path}'); print(f"Final altitude: {m['final_altitude_m']:.2f} m"); print(f"Max tilt angle: {m['max_tilt_deg']:.2f} deg"); print(f"Max angular rate: {m['max_angular_rate_radps']:.2f} rad/s"); print(f"Max lateral displacement: {m['max_lateral_displacement_m']:.2f} m")
if __name__=='__main__': main()
