"""Extract Gowin timing evidence without changing any timing constraints."""
from pathlib import Path
from html import unescape
from hashlib import sha256
import json
import re

root = Path(__file__).resolve().parents[1]
report = root / 'impl/pnr/drive_vision_tof_v24_tr_content.html'
if not report.is_file():
    report = root / 'reports/timing/drive_vision_tof_v24_tr_content.html'
html = report.read_text(encoding='utf-8', errors='replace')

def text(value):
    return unescape(re.sub('<[^>]*>', '', value)).strip()

def table(anchor):
    segment = html.split(f'name="{anchor}"', 1)[1]
    content = re.search(r'<table\b[^>]*>(.*?)</table>', segment, re.S)[1]
    return [[text(cell) for cell in re.findall(r'<t[dh]\b[^>]*>(.*?)</t[dh]>', row, re.S)]
            for row in re.findall(r'<tr\b[^>]*>(.*?)</tr>', content, re.S)]

messages = dict(table('STA_Tool_Run_Summary'))
data = {
    'report': report.relative_to(root).as_posix(),
    'report_sha256': sha256(report.read_bytes()).hexdigest(),
    'setup_violated_endpoints': int(messages['Numbers of Setup Violated Endpoints']),
    'hold_violated_endpoints': int(messages['Numbers of Hold Violated Endpoints']),
    'clock_fmax': table('Max_Frequency_Report'),
    'per_clock_tns': table('Total_Negative_Slack_Report'),
    'setup_paths': table('Setup_Slack_Table'),
    'hold_paths': table('Hold_Slack_Table'),
}
data['fully_closed'] = not (data['setup_violated_endpoints'] or data['hold_violated_endpoints'])
out = root / 'reports/timing_summary.json'
out.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(data, ensure_ascii=False, indent=2))
