"""Optional: pip install pyserial; python monitor.py COM5"""
import argparse
import re
import sys
import time
try:
    import serial
except ImportError:
    sys.exit('Install pyserial: python -m pip install pyserial')
p=argparse.ArgumentParser(description='TOF3 debug UART, 115200 8N1')
p.add_argument('port',help='USB TTL COM port, e.g. COM5')
a=p.parse_args()
pattern=re.compile(r'([LFR])=([0-9A-F]{4})/S=([0-9A-F]{2})/([01])')
try:
    with serial.Serial(a.port,115200,timeout=2) as connection:
        while True:
            line=connection.readline().decode('ascii',errors='replace').strip()
            if not line:
                print('No telemetry: check P7.33 -> adapter RX, common GND and firmware.')
                continue
            matches=pattern.findall(line)
            if len(matches)!=3:
                print('Unrecognized:',line)
                continue
            parts=[]
            for channel,distance,status,valid in matches:
                status=int(status,16)
                parts.append(f'{channel}: {int(distance,16):4d} mm  status={status>>4:X} '
                             f'raw={status:02X} '+('VALID' if valid=='1' else 'INVALID/STALE'))
            print(time.strftime('%H:%M:%S'), ' | '.join(parts),flush=True)
except KeyboardInterrupt:
    pass
except serial.SerialException as error:
    sys.exit(str(error))
