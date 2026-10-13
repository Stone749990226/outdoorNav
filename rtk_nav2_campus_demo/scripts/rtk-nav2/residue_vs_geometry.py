#!/usr/bin/env python3
# ★ 残留格 vs 他们场景的两套几何（★ 纯读 ✓ 不起仿真 ✓）
#   ★ 判据：若残留在**网格边界带**内显著集中 ⇒ 主因 = 那 3 cm 抬升面的边缘 ✓；否则如实否定 ✓
import json,math,csv,collections,sys
G=json.load(open('/home/xjy/projects/outdoorNav/gazebo_campus/generated/scene_geometry.json',encoding='utf-8'))
allc=json.load(open(sys.argv[2]))
cols=[c for c in allc if c['n']!='ground']      # ★ 剔掉 ground（★ 它铺满全场 ⇒ 判据恒真 ✗）
rows=list(csv.DictReader(open(sys.argv[1],encoding='utf-8')))
edges=[]
def is_pair(v): return isinstance(v,list) and len(v)==2 and all(isinstance(t,(int,float)) for t in v)
def walk(c):
    if is_pair(c): return
    if isinstance(c,list) and c and is_pair(c[0]):
        for i in range(len(c)-1): edges.append((c[i][0],c[i][1],c[i+1][0],c[i+1][1]))
        return
    for x in c: walk(x)
walk(G['free']['coordinates']); walk(G['plaza']['coordinates'])
B=collections.defaultdict(list)
for e in edges:
    x0,y0,x1,y1=e; n=max(2,int(max(abs(x1-x0),abs(y1-y0))/0.5)+1)
    for i in range(n+1): B[(int((x0+(x1-x0)*i/n)/0.5),int((y0+(y1-y0)*i/n)/0.5))].append(e)
def d_seg(px,py,e):
    x0,y0,x1,y1=e; dx,dy=x1-x0,y1-y0; L=dx*dx+dy*dy
    t=0.0 if L==0 else max(0.0,min(1.0,((px-x0)*dx+(py-y0)*dy)/L))
    return math.hypot(px-(x0+t*dx),py-(y0+t*dy))
def d_mesh(px,py): return min((d_seg(px,py,e) for e in B.get((int(px/0.5),int(py/0.5)),[])),default=1e9)
def d_col(px,py):
    best,who=1e9,None
    for c in cols:
        d=(math.hypot(max(0,abs(px-c['x'])-c['hx']),max(0,abs(py-c['y'])-c['hy'])) if c['k']=='box'
           else max(0.0,math.hypot(px-c['x'],py-c['y'])-c['hx']))
        if d<best: best,who=d,c['n']
    return best,who
out={'n_residue':len(rows),'edges':len(edges),'obstacles':len(cols),'by_X':{}}
for X in (0.05,0.10,0.20,0.30):
    a=b=c=0; fam=collections.Counter()
    for r in rows:
        px,py=float(r['wx']),float(r['wy'])
        dm=d_mesh(px,py); dc,who=d_col(px,py)
        if dm<=X: a+=1
        if dc<=X: b+=1; fam[who]+=1
        if dm>X and dc>X: c+=1
    out['by_X']['%.2f'%X]={'mesh_boundary':a,'obstacle':b,'neither':c,'obstacle_family':dict(fam)}
    print('  X=%.2f ⇒ ★a 网格边界带 %5.1f%% ｜ ★b 真障碍 %5.1f%% ｜ ★c 两边不沾 %5.1f%%'
          %(X,100.0*a/len(rows),100.0*b/len(rows),100.0*c/len(rows)))
json.dump(out,open(sys.argv[3],'w'),ensure_ascii=False,indent=2)
print('  ★ 已落盘',sys.argv[3])
