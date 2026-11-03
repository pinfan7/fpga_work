"""Plot independently simulated truth and estimated XY coordinates, not board data."""
from pathlib import Path
import csv, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Circle

root=Path(__file__).resolve().parents[1]
font=FontProperties(fname=r'C:\Windows\Fonts\msyh.ttc')
plt.rcParams['font.family']=font.get_name()
plt.rcParams['axes.unicode_minus']=False
with (root/'reports/navigation_trace.csv').open(encoding='utf-8') as f:
    rows=[r for r in csv.DictReader(f) if r['scenario']=='1']
def values(key):return [float(r[key])/10 for r in rows]
x,y=values('true_x_mm'),values('true_y_mm')
fig,ax=plt.subplots(figsize=(13,4.8),layout='constrained')
fig.patch.set_facecolor('#f6f8fb');ax.set_facecolor('#ffffff')
ax.axhspan(-2.5,2.5,color='#d8efdf',alpha=.65,label='控制器回线范围 ±2.5cm（估算判据）')
ax.axhline(0,color='#526075',lw=1.4,ls='--',label='原定两米直线 / X 轴')
ax.plot(x,y,color='#1768b4',lw=2.7,label='独立车辆模型的实际轨迹')
ax.plot(values('est_x_mm'),values('est_y_mm'),color='#ee8a26',lw=1.5,ls=':',label='编码器＋IMU 估算轨迹')
ax.add_patch(Circle((100,0),4.4,facecolor='#dbb838',edgecolor='#8d6e13',zorder=4))
ax.text(100,-8.2,'障碍物占地直径 8.8cm',ha='center',fontsize=10)
ax.scatter([0,200],[0,0],c=['#1768b4','#17803e'],s=45,zorder=5)
ax.annotate('起点 (0, 0)',(0,0),xytext=(4,-8),fontsize=10)
ax.annotate('终点 (200cm, 0)',(200,0),xytext=(163,-8),fontsize=10)
for target_x in (35,65,115,155,180):
    i=min(range(len(x)),key=lambda i:abs(x[i]-target_x))
    a=math.radians(float(rows[i]['heading_deg']))
    ax.arrow(x[i],y[i],6*math.cos(a),6*math.sin(a),width=.22,head_width=2.0,
             color='#1768b4',length_includes_head=True,zorder=6)
ax.set(xlim=(-5,210),ylim=(-13,36),xlabel='X：沿出发方向前进的位置（cm）',ylabel='Y：向左的横向偏移（cm）')
ax.set_aspect('equal',adjustable='box')
ax.grid(color='#e8edf3',lw=.6)
ax.set_title('V24 仿真：把原来的两米直线作为 X 轴，绕障后同时恢复位置与航向',loc='left',pad=15,fontsize=14)
ax.legend(loc='upper right',fontsize=8.5,framealpha=.95)
fig.text(.015,.01,'这是模型仿真，尚未实车验证。到点按 X≈200cm、Y≈0、相对航向≈0°共同判断；绕行总路程约 215cm。',fontsize=9,color='#526075')
fig.savefig(root/'reports/坐标回线仿真.png',dpi=150,bbox_inches='tight')
plt.close(fig)
print(root/'reports/坐标回线仿真.png')
