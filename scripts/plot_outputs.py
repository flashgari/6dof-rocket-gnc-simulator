#!/usr/bin/env python3
import csv
import shutil
from pathlib import Path
PROJECT_ROOT=Path(__file__).resolve().parents[1]; OUT=PROJECT_ROOT/'outputs'; FIG=PROJECT_ROOT/'figures'
def read(path):
    with path.open() as f: return list(csv.DictReader(f))
def col(rows,k): return [float(r[k]) for r in rows]
def pts(xs,ys,l,t,w,h):
    xmin,xmax=min(xs),max(xs); ymin,ymax=min(ys),max(ys)
    if xmax==xmin: xmax+=1
    if ymax==ymin: ymax+=1
    pad=.05*(ymax-ymin); ymin-=pad; ymax+=pad
    return ' '.join(f'{l+(x-xmin)/(xmax-xmin)*w:.2f},{t+h-(y-ymin)/(ymax-ymin)*h:.2f}' for x,y in zip(xs,ys))
def panel(name,xs,ys,label,top,color):
    l=96; w=670; h=92; b=top+h; p=pts(xs,ys,l,top,w,h)
    return f'<text x="{l}" y="{top-16}" class="title">{name}</text><text x="22" y="{top+h/2}" class="axis-label" transform="rotate(-90 22 {top+h/2})">{label}</text><line x1="{l}" y1="{b}" x2="{l+w}" y2="{b}" class="axis"/><line x1="{l}" y1="{top}" x2="{l}" y2="{b}" class="axis"/><polyline points="{p}" fill="none" stroke="{color}" stroke-width="2.5"/>'
def panel_multi(name,series,label,top):
    l=104; w=675; h=104; b=top+h
    all_x=[x for item in series for x in item['x']]; all_y=[y for item in series for y in item['y']]
    xmin,xmax=min(all_x),max(all_x); ymin,ymax=min(all_y),max(all_y)
    if xmax==xmin: xmax+=1
    if ymax==ymin: ymax+=1
    pad=.05*(ymax-ymin); ymin-=pad; ymax+=pad
    def p(xs,ys):
        return ' '.join(f'{l+(x-xmin)/(xmax-xmin)*w:.2f},{top+h-(y-ymin)/(ymax-ymin)*h:.2f}' for x,y in zip(xs,ys))
    lines=[f'<polyline points="{p(item["x"],item["y"])}" fill="none" stroke="{item["color"]}" stroke-width="2.5"/>' for item in series]
    return f'<text x="{l}" y="{top-20}" class="panel-title">{name}</text><text x="24" y="{top+h/2}" class="axis-label" transform="rotate(-90 24 {top+h/2})">{label}</text><line x1="{l}" y1="{b}" x2="{l+w}" y2="{b}" class="axis"/><line x1="{l}" y1="{top}" x2="{l}" y2="{b}" class="axis"/>{"".join(lines)}'
def make(csv_path,title,series,svg_path):
    rows=read(csv_path); x=col(rows,'time_s')
    body=''.join(panel(n,x,col(rows,k),lab,105+i*122,c) for i,(n,k,lab,c) in enumerate(series))
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="820" height="650" viewBox="0 0 820 650"><style>.bg{{fill:#f8fafc}}.title{{font:700 16px system-ui;fill:#111827}}.subtitle{{font:500 13px system-ui;fill:#475569}}.axis{{stroke:#334155;stroke-width:1.2}}.axis-label{{font:600 12px system-ui;fill:#334155}}</style><rect class="bg" width="820" height="650"/><text x="34" y="42" class="title">{title}</text><text x="34" y="64" class="subtitle">Generated directly from simulator CSV output.</text>{body}</svg>'
    svg_path.write_text(svg); print(f'Wrote {svg_path}')
def make_comparison(uncontrolled_path, ideal_path, tvc_path, svg_path):
    u=read(uncontrolled_path); i=read(ideal_path); t=read(tvc_path); ux=col(u,'time_s'); ix=col(i,'time_s'); tx=col(t,'time_s')
    panels=[
        ('Altitude',[{'name':'open','x':ux,'y':col(u,'z_m'),'color':'#dc2626'},{'name':'ideal','x':ix,'y':col(i,'z_m'),'color':'#2563eb'},{'name':'TVC','x':tx,'y':col(t,'z_m'),'color':'#059669'}],'z (m)'),
        ('Unwrapped Pitch',[{'name':'open','x':ux,'y':col(u,'unwrapped_pitch_deg'),'color':'#dc2626'},{'name':'ideal','x':ix,'y':col(i,'unwrapped_pitch_deg'),'color':'#2563eb'},{'name':'TVC','x':tx,'y':col(t,'unwrapped_pitch_deg'),'color':'#059669'}],'pitch (deg)'),
        ('Body Axis Vertical Component',[{'name':'open','x':ux,'y':col(u,'body_z_z'),'color':'#dc2626'},{'name':'ideal','x':ix,'y':col(i,'body_z_z'),'color':'#2563eb'},{'name':'TVC','x':tx,'y':col(t,'body_z_z'),'color':'#059669'}],'body z dot up'),
        ('Lateral Drift',[{'name':'open','x':ux,'y':col(u,'lateral_m'),'color':'#dc2626'},{'name':'ideal','x':ix,'y':col(i,'lateral_m'),'color':'#2563eb'},{'name':'TVC','x':tx,'y':col(t,'lateral_m'),'color':'#059669'}],'lateral (m)'),
        ('TVC Actuator Usage',[{'name':'gimbal','x':tx,'y':col(t,'gimbal_total_deg'),'color':'#7c3aed'}],'gimbal (deg)'),
    ]
    body=''.join(panel_multi(n,s,lab,130+i*142) for i,(n,s,lab) in enumerate(panels))
    legend_items=[
        ('open loop','#dc2626'),
        ('ideal body torque','#2563eb'),
        ('TVC actuator','#059669'),
        ('gimbal angle','#7c3aed'),
    ]
    legend=''.join(
        f'<line x1="{330+i*126}" y1="88" x2="{354+i*126}" y2="88" stroke="{color}" stroke-width="2.5"/>'
        f'<text x="{360+i*126}" y="92" class="legend">{name}</text>'
        for i,(name,color) in enumerate(legend_items)
    )
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="860" height="850" viewBox="0 0 860 850"><style>.bg{{fill:#f8fafc}}.title{{font:700 16px system-ui;fill:#111827}}.subtitle{{font:500 13px system-ui;fill:#475569}}.panel-title{{font:700 15px system-ui;fill:#111827}}.axis{{stroke:#334155;stroke-width:1.2}}.axis-label{{font:600 12px system-ui;fill:#334155}}.legend{{font:600 11px system-ui;fill:#475569}}</style><rect class="bg" width="860" height="850"/><text x="34" y="42" class="title">Week 3B Control Comparison</text><text x="34" y="64" class="subtitle">Open-loop failure, ideal body-torque control, and thrust-vector-control actuator model.</text>{legend}{body}</svg>'
    svg_path.write_text(svg); print(f'Wrote {svg_path}')
def make_lqr_comparison(uncontrolled_path, ideal_path, tvc_path, lqr_path, svg_path):
    u=read(uncontrolled_path); i=read(ideal_path); t=read(tvc_path); lqr=read(lqr_path)
    ux=col(u,'time_s'); ix=col(i,'time_s'); tx=col(t,'time_s'); lx=col(lqr,'time_s')
    panels=[
        ('Altitude',[{'name':'open','x':ux,'y':col(u,'z_m'),'color':'#dc2626'},{'name':'ideal','x':ix,'y':col(i,'z_m'),'color':'#2563eb'},{'name':'TVC PD','x':tx,'y':col(t,'z_m'),'color':'#059669'},{'name':'TVC LQR','x':lx,'y':col(lqr,'z_m'),'color':'#0891b2'}],'z (m)'),
        ('Unwrapped Pitch',[{'name':'open','x':ux,'y':col(u,'unwrapped_pitch_deg'),'color':'#dc2626'},{'name':'ideal','x':ix,'y':col(i,'unwrapped_pitch_deg'),'color':'#2563eb'},{'name':'TVC PD','x':tx,'y':col(t,'unwrapped_pitch_deg'),'color':'#059669'},{'name':'TVC LQR','x':lx,'y':col(lqr,'unwrapped_pitch_deg'),'color':'#0891b2'}],'pitch (deg)'),
        ('Body Axis Vertical Component',[{'name':'open','x':ux,'y':col(u,'body_z_z'),'color':'#dc2626'},{'name':'ideal','x':ix,'y':col(i,'body_z_z'),'color':'#2563eb'},{'name':'TVC PD','x':tx,'y':col(t,'body_z_z'),'color':'#059669'},{'name':'TVC LQR','x':lx,'y':col(lqr,'body_z_z'),'color':'#0891b2'}],'body z dot up'),
        ('Lateral Drift',[{'name':'open','x':ux,'y':col(u,'lateral_m'),'color':'#dc2626'},{'name':'ideal','x':ix,'y':col(i,'lateral_m'),'color':'#2563eb'},{'name':'TVC PD','x':tx,'y':col(t,'lateral_m'),'color':'#059669'},{'name':'TVC LQR','x':lx,'y':col(lqr,'lateral_m'),'color':'#0891b2'}],'lateral (m)'),
        ('TVC Actuator Usage',[{'name':'PD gimbal','x':tx,'y':col(t,'gimbal_total_deg'),'color':'#7c3aed'},{'name':'LQR gimbal','x':lx,'y':col(lqr,'gimbal_total_deg'),'color':'#0891b2'}],'gimbal (deg)'),
    ]
    body=''.join(panel_multi(n,s,lab,130+i*142) for i,(n,s,lab) in enumerate(panels))
    legend_items=[('open loop','#dc2626'),('ideal body torque','#2563eb'),('TVC PD','#059669'),('TVC LQR','#0891b2'),('gimbal angle','#7c3aed')]
    legend=''.join(f'<line x1="{250+i*112}" y1="88" x2="{274+i*112}" y2="88" stroke="{color}" stroke-width="2.5"/><text x="{280+i*112}" y="92" class="legend">{name}</text>' for i,(name,color) in enumerate(legend_items))
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="860" height="850" viewBox="0 0 860 850"><style>.bg{{fill:#f8fafc}}.title{{font:700 16px system-ui;fill:#111827}}.subtitle{{font:500 13px system-ui;fill:#475569}}.panel-title{{font:700 15px system-ui;fill:#111827}}.axis{{stroke:#334155;stroke-width:1.2}}.axis-label{{font:600 12px system-ui;fill:#334155}}.legend{{font:600 11px system-ui;fill:#475569}}</style><rect class="bg" width="860" height="850"/><text x="34" y="42" class="title">Week 4A LQR Control Comparison</text><text x="34" y="64" class="subtitle">Open loop, ideal torque, PD TVC, and small-angle LQR through the TVC actuator.</text>{legend}{body}</svg>'
    svg_path.write_text(svg); print(f'Wrote {svg_path}')
def make_estimator_plot(estimated_path, svg_path):
    e=read(estimated_path); x=col(e,'time_s')
    panels=[
        ('True vs Estimated Tilt',[{'name':'true','x':x,'y':col(e,'tilt_deg'),'color':'#2563eb'},{'name':'estimated','x':x,'y':col(e,'est_tilt_deg'),'color':'#0891b2'}],'tilt (deg)'),
        ('Attitude Estimation Error',[{'name':'error','x':x,'y':col(e,'attitude_error_deg'),'color':'#dc2626'}],'error (deg)'),
        ('Gyro Measurement',[{'name':'gx','x':x,'y':col(e,'gyro_x_radps'),'color':'#7c3aed'},{'name':'gy','x':x,'y':col(e,'gyro_y_radps'),'color':'#9333ea'}],'gyro (rad/s)'),
        ('Accelerometer Specific Force',[{'name':'ax','x':x,'y':col(e,'accel_x_mps2'),'color':'#059669'},{'name':'az','x':x,'y':col(e,'accel_z_mps2'),'color':'#16a34a'}],'accel (m/s^2)'),
        ('Estimated-State TVC Usage',[{'name':'gimbal','x':x,'y':col(e,'gimbal_total_deg'),'color':'#f97316'}],'gimbal (deg)'),
    ]
    body=''.join(panel_multi(n,s,lab,130+i*142) for i,(n,s,lab) in enumerate(panels))
    legend_items=[('true','#2563eb'),('estimated','#0891b2'),('attitude error','#dc2626'),('gyro','#7c3aed'),('accelerometer','#059669'),('gimbal','#f97316')]
    legend=''.join(f'<line x1="{170+i*108}" y1="88" x2="{194+i*108}" y2="88" stroke="{color}" stroke-width="2.5"/><text x="{200+i*108}" y="92" class="legend">{name}</text>' for i,(name,color) in enumerate(legend_items))
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="860" height="850" viewBox="0 0 860 850"><style>.bg{{fill:#f8fafc}}.title{{font:700 16px system-ui;fill:#111827}}.subtitle{{font:500 13px system-ui;fill:#475569}}.panel-title{{font:700 15px system-ui;fill:#111827}}.axis{{stroke:#334155;stroke-width:1.2}}.axis-label{{font:600 12px system-ui;fill:#334155}}.legend{{font:600 11px system-ui;fill:#475569}}</style><rect class="bg" width="860" height="850"/><text x="34" y="42" class="title">Week 5 Estimated-State TVC</text><text x="34" y="64" class="subtitle">Noisy IMU, low-rate attitude reference, quaternion estimator, and closed-loop TVC using estimated attitude/rate.</text>{legend}{body}</svg>'
    svg_path.write_text(svg); print(f'Wrote {svg_path}')
def make_estimated_control_comparison(lqr_path, estimated_path, svg_path):
    lqr=read(lqr_path); e=read(estimated_path); lx=col(lqr,'time_s'); ex=col(e,'time_s')
    panels=[
        ('Altitude',[{'name':'truth LQR','x':lx,'y':col(lqr,'z_m'),'color':'#0891b2'},{'name':'estimated LQR','x':ex,'y':col(e,'z_m'),'color':'#f97316'}],'z (m)'),
        ('Tilt',[{'name':'truth LQR','x':lx,'y':col(lqr,'tilt_deg'),'color':'#0891b2'},{'name':'estimated LQR','x':ex,'y':col(e,'tilt_deg'),'color':'#f97316'}],'tilt (deg)'),
        ('Body Axis Vertical Component',[{'name':'truth LQR','x':lx,'y':col(lqr,'body_z_z'),'color':'#0891b2'},{'name':'estimated LQR','x':ex,'y':col(e,'body_z_z'),'color':'#f97316'}],'body z dot up'),
        ('Lateral Drift',[{'name':'truth LQR','x':lx,'y':col(lqr,'lateral_m'),'color':'#0891b2'},{'name':'estimated LQR','x':ex,'y':col(e,'lateral_m'),'color':'#f97316'}],'lateral (m)'),
        ('Gimbal Usage',[{'name':'truth LQR','x':lx,'y':col(lqr,'gimbal_total_deg'),'color':'#0891b2'},{'name':'estimated LQR','x':ex,'y':col(e,'gimbal_total_deg'),'color':'#f97316'}],'gimbal (deg)'),
    ]
    body=''.join(panel_multi(n,s,lab,130+i*142) for i,(n,s,lab) in enumerate(panels))
    legend='<line x1="460" y1="88" x2="484" y2="88" stroke="#0891b2" stroke-width="2.5"/><text x="490" y="92" class="legend">truth-state LQR</text><line x1="590" y1="88" x2="614" y2="88" stroke="#f97316" stroke-width="2.5"/><text x="620" y="92" class="legend">estimated-state LQR</text>'
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="860" height="850" viewBox="0 0 860 850"><style>.bg{{fill:#f8fafc}}.title{{font:700 16px system-ui;fill:#111827}}.subtitle{{font:500 13px system-ui;fill:#475569}}.panel-title{{font:700 15px system-ui;fill:#111827}}.axis{{stroke:#334155;stroke-width:1.2}}.axis-label{{font:600 12px system-ui;fill:#334155}}.legend{{font:600 11px system-ui;fill:#475569}}</style><rect class="bg" width="860" height="850"/><text x="34" y="42" class="title">Week 5 Truth-State vs Estimated-State Control</text><text x="34" y="64" class="subtitle">Effect of sensor noise and estimator error on the same LQR TVC controller.</text>{legend}{body}</svg>'
    svg_path.write_text(svg); print(f'Wrote {svg_path}')
def make_actuator_limited_comparison(instant_path, actuator_path, estimated_path, svg_path):
    instant=read(instant_path); actuator=read(actuator_path); estimated=read(estimated_path)
    ix=col(instant,'time_s'); ax=col(actuator,'time_s'); ex=col(estimated,'time_s')
    panels=[
        ('Tilt Response',[{'name':'instant LQR','x':ix,'y':col(instant,'tilt_deg'),'color':'#0891b2'},{'name':'actuator-limited LQR','x':ax,'y':col(actuator,'tilt_deg'),'color':'#f97316'},{'name':'estimated + actuator','x':ex,'y':col(estimated,'tilt_deg'),'color':'#7c3aed'}],'tilt (deg)'),
        ('Lateral Drift',[{'name':'instant LQR','x':ix,'y':col(instant,'lateral_m'),'color':'#0891b2'},{'name':'actuator-limited LQR','x':ax,'y':col(actuator,'lateral_m'),'color':'#f97316'},{'name':'estimated + actuator','x':ex,'y':col(estimated,'lateral_m'),'color':'#7c3aed'}],'lateral (m)'),
        ('Commanded vs Achieved Gimbal',[{'name':'commanded','x':ax,'y':col(actuator,'cmd_gimbal_total_deg'),'color':'#dc2626'},{'name':'achieved','x':ax,'y':col(actuator,'ach_gimbal_total_deg'),'color':'#059669'}],'gimbal (deg)'),
        ('Gimbal Lag Error',[{'name':'truth-state actuator','x':ax,'y':col(actuator,'gimbal_lag_error_deg'),'color':'#f97316'},{'name':'estimated-state actuator','x':ex,'y':col(estimated,'gimbal_lag_error_deg'),'color':'#7c3aed'}],'lag (deg)'),
        ('Torque Authority Margin',[{'name':'truth-state actuator','x':ax,'y':col(actuator,'torque_authority_margin_nm'),'color':'#2563eb'},{'name':'estimated-state actuator','x':ex,'y':col(estimated,'torque_authority_margin_nm'),'color':'#9333ea'}],'margin (N m)'),
    ]
    body=''.join(panel_multi(n,s,lab,130+i*142) for i,(n,s,lab) in enumerate(panels))
    legend_items=[('instant LQR','#0891b2'),('actuator-limited','#f97316'),('estimated + actuator','#7c3aed'),('commanded gimbal','#dc2626'),('achieved gimbal','#059669')]
    legend=''.join(f'<line x1="{210+i*118}" y1="88" x2="{234+i*118}" y2="88" stroke="{color}" stroke-width="2.5"/><text x="{240+i*118}" y="92" class="legend">{name}</text>' for i,(name,color) in enumerate(legend_items))
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="860" height="850" viewBox="0 0 860 850"><style>.bg{{fill:#f8fafc}}.title{{font:700 16px system-ui;fill:#111827}}.subtitle{{font:500 13px system-ui;fill:#475569}}.panel-title{{font:700 15px system-ui;fill:#111827}}.axis{{stroke:#334155;stroke-width:1.2}}.axis-label{{font:600 12px system-ui;fill:#334155}}.legend{{font:600 11px system-ui;fill:#475569}}</style><rect class="bg" width="860" height="850"/><text x="34" y="42" class="title">Week 6 Actuator-Limited TVC</text><text x="34" y="64" class="subtitle">Finite gimbal bandwidth, command tracking error, and remaining TVC moment authority.</text>{legend}{body}</svg>'
    svg_path.write_text(svg); print(f'Wrote {svg_path}')
def refresh_figures():
    FIG.mkdir(exist_ok=True)
    copies={
        'week2_disturbed_uncontrolled_plots.svg':'week2-open-loop-failure.svg',
        'week3b_control_comparison_plots.svg':'week3b-control-comparison.svg',
        'week4a_lqr_control_comparison_plots.svg':'week4a-lqr-control-comparison.svg',
        'week4b_monte_carlo_summary.svg':'week4b-monte-carlo-robustness.svg',
        'week5_estimated_tvc_plots.svg':'week5-estimated-state-tvc.svg',
        'week5_estimated_vs_truth_control_plots.svg':'week5-truth-vs-estimated-control.svg',
        'week6_actuator_limited_tvc_plots.svg':'week6-actuator-limited-tvc.svg',
    }
    for src,dst in copies.items():
        source=OUT/src
        if source.exists():
            shutil.copyfile(source,FIG/dst)
def main():
    make(OUT/'week1_ascent.csv','Week 1 Baseline Ascent',[('Altitude','z_m','z (m)','#2563eb'),('Vertical Velocity','vz_mps','vz (m/s)','#059669'),('Quaternion Norm','q_norm','|q|','#dc2626')],OUT/'week1_ascent_plots.svg')
    make(OUT/'week2_disturbed_uncontrolled.csv','Week 2 Disturbed Uncontrolled Ascent',[('Altitude','z_m','z (m)','#2563eb'),('Unwrapped Pitch History','unwrapped_pitch_deg','pitch (deg)','#dc2626'),('Body Axis Vertical Component','body_z_z','body z dot up','#059669'),('Lateral Drift','lateral_m','lateral (m)','#7c3aed')],OUT/'week2_disturbed_uncontrolled_plots.svg')
    make(OUT/'week3a_controlled_ideal_torque.csv','Week 3A Controlled Ideal-Torque Ascent',[('Altitude','z_m','z (m)','#2563eb'),('Unwrapped Pitch History','unwrapped_pitch_deg','pitch (deg)','#dc2626'),('Body Axis Vertical Component','body_z_z','body z dot up','#059669'),('Lateral Drift','lateral_m','lateral (m)','#7c3aed'),('Control Torque','control_torque_norm_nm','torque (N m)','#9333ea')],OUT/'week3a_controlled_ideal_torque_plots.svg')
    make(OUT/'week3b_tvc_controlled.csv','Week 3B TVC-Controlled Ascent',[('Altitude','z_m','z (m)','#2563eb'),('Unwrapped Pitch History','unwrapped_pitch_deg','pitch (deg)','#dc2626'),('Body Axis Vertical Component','body_z_z','body z dot up','#059669'),('Lateral Drift','lateral_m','lateral (m)','#7c3aed'),('Gimbal Angle','gimbal_total_deg','gimbal (deg)','#9333ea')],OUT/'week3b_tvc_controlled_plots.svg')
    make(OUT/'week4a_lqr_tvc_controlled.csv','Week 4A LQR TVC-Controlled Ascent',[('Altitude','z_m','z (m)','#2563eb'),('Unwrapped Pitch History','unwrapped_pitch_deg','pitch (deg)','#dc2626'),('Body Axis Vertical Component','body_z_z','body z dot up','#059669'),('Lateral Drift','lateral_m','lateral (m)','#7c3aed'),('Gimbal Angle','gimbal_total_deg','gimbal (deg)','#9333ea')],OUT/'week4a_lqr_tvc_controlled_plots.svg')
    make_comparison(OUT/'week2_disturbed_uncontrolled.csv',OUT/'week3a_controlled_ideal_torque.csv',OUT/'week3b_tvc_controlled.csv',OUT/'week3b_control_comparison_plots.svg')
    make_comparison(OUT/'week2_disturbed_uncontrolled.csv',OUT/'week3a_controlled_ideal_torque.csv',OUT/'week3b_tvc_controlled.csv',OUT/'week3a_controlled_vs_uncontrolled_plots.svg')
    make_lqr_comparison(OUT/'week2_disturbed_uncontrolled.csv',OUT/'week3a_controlled_ideal_torque.csv',OUT/'week3b_tvc_controlled.csv',OUT/'week4a_lqr_tvc_controlled.csv',OUT/'week4a_lqr_control_comparison_plots.svg')
    make_estimator_plot(OUT/'week5_estimated_tvc_controlled.csv',OUT/'week5_estimated_tvc_plots.svg')
    make_estimated_control_comparison(OUT/'week4a_lqr_tvc_controlled.csv',OUT/'week5_estimated_tvc_controlled.csv',OUT/'week5_estimated_vs_truth_control_plots.svg')
    make_actuator_limited_comparison(OUT/'week4a_lqr_tvc_controlled.csv',OUT/'week6_lqr_tvc_actuator_limited.csv',OUT/'week6_estimated_lqr_tvc_actuator_limited.csv',OUT/'week6_actuator_limited_tvc_plots.svg')
    refresh_figures()
if __name__=='__main__': main()
