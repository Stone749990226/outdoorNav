import {expect,test} from '@playwright/test';
import {readFileSync} from 'node:fs';
const grid=JSON.parse(readFileSync(new URL('./fixtures/scene-costmap.json',import.meta.url),'utf8'));

async function fixture(page:any){
 const requests:string[]=[],mutations:string[]=[],errors:string[]=[];
 let imu='normal',recording=false,failed=false,localFailed=false,stale=false,unaligned=false;
 page.on('pageerror',(e:Error)=>errors.push(e.message));
 await page.route('**/api/**',async(route:any)=>{
  const request=route.request(),path=new URL(request.url()).pathname;
  if(request.method()!=='GET')mutations.push(path);
  if(path==='/api/outdoor/state'){
   if(failed)return route.fulfill({status:503,json:{detail:'offline'}});
   return route.fulfill({json:{ok:true,is_active:true,status:{session:{state:recording?'recording':'idle'},pose:{x:4.5,y:7.5,yaw:.6},ready:true,live_track:[]},
    map:{ok:true,map_id:'preview',version:'v1',network:{roads:[{id:'r',points:[{x:0,y:0},{x:5,y:0}],gait:'flat',sources:[],length_m:5}]}},places:[],annotations:[],
    navigation:{available:true,ready:imu!=='interrupted',localization_grade:'float',not_ready_reasons:imu==='interrupted'?['heading']:[],health:{heading:imu!=='interrupted'},
     fusion:{rtk_grade:'float',estimate_valid:true,imu_degraded:imu==='interrupted',heading_source:imu==='interrupted'?'held_imu':'imu'},
     imu_stream:{state:imu,raw_age_s:imu==='interrupted'?.8:.01,attitude_age_s:.01,serial_connected:true,events:[{kind:'interrupted',at_unix_s:1700000000},{kind:'recovered',at_unix_s:1700000001,duration_s:.85}]}}}});
  }
  if(path.startsWith('/api/outdoor/costmaps/')){
   requests.push(path);
   if(path.endsWith('/local')&&localFailed)return route.fulfill({status:503,json:{detail:'Waiting for local costmap'}});
   return route.fulfill({json:{...grid[path.endsWith('/local')?'local':'global'],age_s:stale?5:.01,stale,...(path.endsWith('/local')&&unaligned?{map_from_frame:null}:{})}});
  }
  return route.fulfill({json:{mapping:{mode:'localization'},map_binding:{map_id:'preview',map_type:'outdoor'}}});
 });
 await page.goto('/outdoor');
 await expect(page.locator('#od-localization')).toContainText('正常');
 return {requests,mutations,errors,setImu:(v:string)=>imu=v,setRecording:()=>recording=true,setFailed:()=>failed=true,setLocalFailed:()=>localFailed=true,setStale:()=>stale=true,setUnaligned:()=>unaligned=true};
}

test('historical single-map overlay aligns frames, toggles layers and keeps status compact',async({page})=>{
 const f=await fixture(page);
 await page.getByRole('button',{name:'代价地图',exact:true}).click();
 const svg=page.locator('#od-costmaps svg');
 await expect(svg).toHaveCount(1);
 await expect(svg.locator('[data-layer="global"]')).toHaveCount(1);
 await expect(svg.locator('[data-layer="local"]')).toHaveCount(1);
 const localTransform=await svg.locator('[data-layer="local"]').evaluate((node:any)=>{
  const matrix=node.transform.baseVal.consolidate().matrix;
  const p=new DOMPoint(0,0).matrixTransform(matrix),q=new DOMPoint(2,0).matrixTransform(matrix);
  return [p.x,p.y,q.x,q.y];
 });
 expect(localTransform).toEqual([4,-8,4,-9]);
 await page.getByRole('checkbox',{name:'局部 · 青色'}).uncheck();
 await expect(svg.locator('[data-layer="local"]')).toHaveCount(0);
 await page.getByRole('button',{name:'全图',exact:true}).click();
 const colors=await svg.evaluate(async(node:any)=>{
  const clone=node.cloneNode(true),w=node.clientWidth,h=node.clientHeight;
  clone.setAttribute('width',w);clone.setAttribute('height',h);
  const image=new Image();image.src='data:image/svg+xml;charset=utf-8,'+encodeURIComponent(new XMLSerializer().serializeToString(clone));await image.decode();
  const canvas=document.createElement('canvas');canvas.width=w;canvas.height=h;const ctx=canvas.getContext('2d')!;ctx.drawImage(image,0,0,w,h);
  const box=node.getBoundingClientRect();
  return [[4,8],[2,8],[2,10],[4,10]].map(([x,y])=>{const p=new DOMPoint(x,-y).matrixTransform(node.getScreenCTM());return Array.from(ctx.getImageData(Math.round(p.x-box.x),Math.round(p.y-box.y),1,1).data).slice(0,3);});
 });
 expect(colors).toEqual([[200,65,200],[199,65,200],[233,239,233],[78,88,98]]);
 await page.getByRole('checkbox',{name:'局部 · 青色'}).check();
 const locate=page.getByRole('button',{name:'定位山猫'});await locate.click();await expect(locate).toHaveClass(/primary/);
 const box=(await svg.boundingBox())!;await page.mouse.move(box.x+100,box.y+100);await page.mouse.down();await page.mouse.move(box.x+150,box.y+120);await page.mouse.up();
 await expect(locate).not.toHaveClass(/primary/);await page.getByRole('button',{name:'全图',exact:true}).click();
 await expect(page.locator('#od-health-details')).toBeHidden();
 const compact=await page.locator('#od-localization').boundingBox();expect(compact!.height).toBeLessThan(90);
 await page.getByRole('button',{name:'详细信息',exact:true}).click();await expect(page.locator('#od-health-details')).toBeVisible();
 await page.locator('#od-imu-events summary').click();
 await page.screenshot({path:'../artifacts/scene-observability-20261113/expanded-details.png',fullPage:true});
 await page.getByRole('button',{name:'收起详细信息',exact:true}).click();
 await page.screenshot({path:'../artifacts/scene-observability-20261113/overlaid-costmap.png',fullPage:true});
 f.setUnaligned();await expect(page.locator('#od-costmaps')).toContainText('坐标未对齐');await expect(svg.locator('[data-layer="local"]')).toHaveCount(0);
 await page.evaluate(()=>{Object.defineProperty(document,'hidden',{configurable:true,get:()=>true});document.dispatchEvent(new Event('visibilitychange'));});
 const hiddenCount=f.requests.length;await page.waitForTimeout(1200);expect(f.requests.length).toBe(hiddenCount);
 await page.evaluate(()=>{Object.defineProperty(document,'hidden',{configurable:true,get:()=>false});document.dispatchEvent(new Event('visibilitychange'));});
 await expect.poll(()=>f.requests.length).toBeGreaterThan(hiddenCount);
 await page.getByRole('button',{name:'查看',exact:true}).click();
 const count=f.requests.length;await page.waitForTimeout(1300);expect(f.requests.length).toBe(count);
 expect(f.mutations).toEqual([]);expect(f.errors).toEqual([]);
});

test('interruption, resumed sampling, stale maps and recording remain observable',async({page})=>{
 const f=await fixture(page);
 f.setImu('interrupted');await expect(page.locator('#od-localization')).toContainText('数据中断');
 await page.getByRole('button',{name:'详细信息',exact:true}).click();
 await expect(page.locator('#od-health-details')).toBeVisible();
 await expect(page.locator('#od-localization')).toContainText('在线');
 await expect(page.locator('#od-localization')).toHaveClass(/od-health-error/);
 f.setImu('recovered');await expect(page.locator('#od-localization')).toContainText('不代表姿态精度已恢复');
 f.setRecording();await expect(page.getByRole('button',{name:'代价地图',exact:true})).toBeEnabled();
 await page.getByRole('button',{name:'代价地图',exact:true}).click();
 await expect(page.locator('#od-costmaps')).toBeVisible();
 f.setStale();f.setLocalFailed();
 await expect(page.locator('#od-costmaps')).toContainText('数据已过期');
 await expect(page.locator('#od-costmaps')).toContainText('代价地图暂不可用');
 await page.screenshot({path:'../artifacts/scene-observability-20261113/stale-and-interrupted.png',fullPage:true});
 f.setFailed();await expect(page.locator('#od-localization')).toContainText('状态不可用');
 await expect(page.locator('#od-localization')).not.toContainText('采样已恢复');
 expect(f.mutations).toEqual([]);expect(f.errors).toEqual([]);
});
