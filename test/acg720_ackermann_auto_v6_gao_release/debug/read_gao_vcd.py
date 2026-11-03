"""Read an exported GAO VCD offline. Never connects to hardware or starts motors.
Usage: python debug/read_gao_vcd.py captures/run01.vcd
"""
import json
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

SIGNED={'dbg_count_l','dbg_count_r','dbg_rpm_l','dbg_rpm_r','dbg_target_l',
        'dbg_target_r','dbg_pi_l','dbg_pi_r','dbg_balance_error'}

def read_vcd(path):
    config=ET.parse(Path(__file__).with_name('drive_v6.rao'))
    widths={}
    for node in config.find('.//AoCore/SignalList'):
        name=node.get('name') if node.tag=='Bus' else node.text
        base=re.search(r'dbg_\w+',name).group()
        widths[base]=len(node) if node.tag=='Bus' else 1
    ids={};state={};records=[];header=True;time=0;var_decl=None
    def snapshot():
        if 'dbg_sample' not in state:return
        if set(state['dbg_sample'])-{'0','1'}:return
        out={}
        for name,width in widths.items():
            bits=state.get(name)
            if bits is None or set(bits)-{'0','1'}:out[name]=None;continue
            value=int(bits,2)
            if name in SIGNED and value&(1<<(width-1)):value-=1<<width
            out[name]=value
        out['vcd_time']=time
        if not records or out['dbg_sample']!=records[-1]['dbg_sample']:records.append(out)
    with path.open(encoding='utf-8-sig',errors='replace') as stream:
        for raw in stream:
            line=raw.strip()
            if header:
                if line.startswith('$var'):
                    var_decl=line
                elif var_decl is not None:
                    var_decl+=' '+line
                if var_decl is not None and '$end' in var_decl:
                    parts=var_decl.split();ref=''.join(parts[4:-1])
                    match=re.search(r'(dbg_\w+)(?:\[(\d+)(?::\d+)?\])?',ref)
                    if match:
                        name=match.group(1);width=int(parts[2])
                        bit=int(match.group(2)) if width==1 and widths.get(name,1)>1 else None
                        ids[parts[3]]=(name,width,bit)
                    var_decl=None
                if line.startswith('$enddefinitions'):header=False
                continue
            if not line or line[0]=='$':continue
            if line[0]=='#':snapshot();time=int(line[1:]);continue
            if line[0] in 'bB':
                bits,ident=line[1:].split()
            elif line[0] in '01xXzZ':bits,ident=line[0],line[1:]
            else:continue
            if ident not in ids:continue
            name,width,bit=ids[ident];bits=bits.lower()
            if bit is None:
                fill='0' if bits[0] in '01' else bits[0]
                state[name]=bits.rjust(width,fill)
            else:
                old=list(state.get(name,'x'*widths[name]));old[widths[name]-1-bit]=bits[-1]
                state[name]=''.join(old)
    snapshot()
    return records,{name for name,_,_ in ids.values()}

def summarize(records,names):
    required={'dbg_sample','dbg_phase','dbg_code','dbg_count_l','dbg_count_r',
              'dbg_rpm_l','dbg_rpm_r','dbg_pwm_l','dbg_pwm_r'}
    result={'samples':len(records),'missing_signals':sorted(required-names),
            'note':'20ms snapshots; PWM values are duty counts, not measured analog voltages.'}
    if not records:return result
    result['version_hex']=hex(records[-1]['dbg_version']) if records[-1]['dbg_version'] is not None else None
    samples=[r['dbg_sample'] for r in records if r['dbg_sample'] is not None]
    if samples:result['span_seconds']=(max(samples)-min(samples))*0.02
    transitions=[];previous=None
    for row in records:
        phase=row['dbg_phase']
        if phase!=previous:
            transitions.append({k:row[k] for k in ('dbg_sample','dbg_phase','dbg_code')});previous=phase
    result['phase_transitions']=transitions
    faults=[r for r in records if r['dbg_code'] not in (None,0)]
    if faults:
        result['first_fault']=faults[0]
        idx=records.index(faults[0])
        result['last_10_prefault_snapshots']=records[max(0,idx-10):idx]
    else:result['first_fault']=None
    for name in ('dbg_rpm_l','dbg_rpm_r','dbg_pwm_l','dbg_pwm_r','dbg_invalid_l','dbg_invalid_r'):
        values=[r[name] for r in records if r[name] is not None]
        if values:result[name+'_range']=[min(values),max(values)]
    return result

if __name__=='__main__':
    if len(sys.argv)!=2:raise SystemExit(__doc__)
    path=Path(sys.argv[1]);records,names=read_vcd(path)
    print(json.dumps(summarize(records,names),ensure_ascii=False,indent=2))
