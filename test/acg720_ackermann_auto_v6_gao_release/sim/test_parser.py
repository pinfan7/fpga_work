import importlib.util
import tempfile
from pathlib import Path
root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('reader',root/'debug/read_gao_vcd.py')
reader=importlib.util.module_from_spec(spec);spec.loader.exec_module(reader)
rows,names=reader.read_vcd(root/'sim/parser_fixture.vcd')
report=reader.summarize(rows,names)
assert len(rows)==3 and rows[0]['dbg_count_l']==-16
assert rows[0]['dbg_rpm_l']==-63 and rows[0]['dbg_rpm_r']==-60
assert report['span_seconds']==0.04 and report['first_fault']['dbg_code']==9
assert not report['missing_signals']
fixture=(root/'sim/parser_fixture.vcd').read_text()
# Native Gowin exports declarations across three lines and initializes X values.
fixture=fixture.replace('$var wire 32 " dbg_sample [31:0] $end',
                        '$var\nreg 32 " dbg_sample[31:0]\n$end')
fixture=fixture.replace('#0\n', '#0\nbx "\n#1\n', 1)
with tempfile.TemporaryDirectory() as directory:
    native=Path(directory)/'native.vcd'
    native.write_text(fixture)
    native_rows,native_names=reader.read_vcd(native)
    assert len(native_rows)==3 and native_rows[0]['dbg_sample']==1
    assert native_rows[0]['dbg_count_l']==-16
    assert 'dbg_sample' in native_names
print('PASS: offline VCD reader recognizes signed bus values, sample timing, and fault9')
print('PASS: multiline Gowin declarations and unknown initialization')
