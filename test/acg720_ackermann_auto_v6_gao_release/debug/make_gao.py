"""Generate native RTL GAO config; no device programming or JTAG commands."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parent
SIGNALS=[('dbg_version',16),('dbg_sample',32),('dbg_phase',4),('dbg_code',4),
 ('dbg_fault_phase',4),('dbg_flags',16),('dbg_terminal',1),('dbg_elapsed',32),
 ('dbg_count_l',32),('dbg_count_r',32),('dbg_rpm_l',32),('dbg_rpm_r',32),
 ('dbg_target_l',32),('dbg_target_r',32),('dbg_pi_l',16),('dbg_pi_r',16),
 ('dbg_pwm_l',16),('dbg_pwm_r',16),('dbg_invalid_l',16),('dbg_invalid_r',16),
 ('dbg_fresh_l',16),('dbg_fresh_r',16),('dbg_imbalance_ms',16),
 ('dbg_progress_l',32),('dbg_progress_r',32),('dbg_balance_error',40),
 ('dbg_fault_progress_l',32),('dbg_fault_progress_r',32)]
root=ET.Element('GAO_CONFIG')
ET.SubElement(root,'Version').text='3.0'
ET.SubElement(root,'Mode').text='Standard'
core=ET.SubElement(root,'AoCore',index='0',sample_clock='probe_clk',trig_type='0',
 storage_depth='2048',window_num='1',capture_amount='2048',implementation='0',
 trigger_pos='1920',module_name='ackermann_auto_top',
 force_trigger_by_falling_edge='false',capture_init_data_enabled='false')
lst=ET.SubElement(core,'SignalList')
for name,width in SIGNALS:
    if width==1: ET.SubElement(lst,'Signal',capture_enable='true').text=name
    else:
        bus=ET.SubElement(lst,'Bus',capture_enable='true',name=f'{name}[{width-1}:0]')
        for bit in reversed(range(width)):ET.SubElement(bus,'Signal').text=f'{name}[{bit}]'
triggers=ET.SubElement(core,'Triggers')
for i in range(16):
    trig=ET.SubElement(triggers,'Trigger',index=str(i))
    if i==0:ET.SubElement(ET.SubElement(trig,'SignalList'),'Signal').text='dbg_terminal'
units=ET.SubElement(core,'MatchUnits')
for i in range(16):
    attrib=dict(index=str(i),enabled=str(int(i==0)),match_type='0',counter_enable='0',
      counter_width='2',counter='2',countinuous='0',func='0',
      value0='1' if i==0 else '',value1='0' if i==0 else '')
    if i==0:attrib['trigger']='0'
    ET.SubElement(units,'MatchUnit',attrib)
ET.SubElement(ET.SubElement(core,'Expressions',type='Static'),'Expression').text='M0'
ET.SubElement(root,'GAO_ID').text='0110000001100001'
ET.indent(root,space='    ')
ET.ElementTree(root).write(ROOT/'drive_v6.rao',encoding='utf-8',xml_declaration=True)
print(f'Native RTL GAO: {sum(w for _,w in SIGNALS)} bits x 2048 samples, 50Hz')
