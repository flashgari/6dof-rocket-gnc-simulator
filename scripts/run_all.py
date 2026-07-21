#!/usr/bin/env python3
import subprocess, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def run(cmd): print('$ '+' '.join(cmd)); subprocess.run(cmd,cwd=ROOT,check=True)
def main():
    run([sys.executable,'scripts/run_week1_ascent.py'])
    run([sys.executable,'scripts/run_week2_disturbed_ascent.py'])
    run([sys.executable,'scripts/run_week3a_controlled_ascent.py'])
    run([sys.executable,'scripts/run_week3b_tvc_ascent.py'])
    run([sys.executable,'scripts/run_week4a_lqr_tvc_ascent.py'])
    run([sys.executable,'scripts/run_week4b_monte_carlo.py'])
    run([sys.executable,'scripts/run_week5_estimated_tvc_ascent.py'])
    run([sys.executable,'scripts/run_week6_actuator_limited_tvc.py'])
    run([sys.executable,'scripts/run_week7_variable_mass_ascent.py'])
    run([sys.executable,'scripts/plot_outputs.py'])
    run([sys.executable,'scripts/write_reports.py'])
    run([sys.executable,'scripts/build_animation.py'])
    run([sys.executable,'-m','unittest','discover','-s','tests'])
if __name__=='__main__': main()
