"""Acceptance constraints on generated geometry, meshes and occupancy pixels."""
import copy
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import shapely
from shapely.geometry import LineString, Point, Polygon, box, shape
from shapely.ops import unary_union

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE/'scripts'))
from build_scene import geometry, read, survey_lines


class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=read(HERE/'config/scene.json')
        cls.scene=read(HERE/'generated/scene_geometry.json')
        cls.free=shape(cls.scene['free'])
        cls.network,cls.lines=survey_lines()
        cls.trace=unary_union(list(cls.lines.values()))
        cls.image=np.asarray(Image.open(HERE/'generated/campus.pgm'))

    def test_all_continuous_trajectories_have_required_clearance(self):
        for name,line in self.lines.items():
            with self.subTest(trace=name):
                self.assertTrue(self.free.covers(line.buffer(0.85)))
                self.assertGreaterEqual(line.distance(self.free.boundary),0.85)
        # Original approach was intentionally excluded from the old closed road graph.
        approach=list(self.lines['raw/1.ubx/0'].coords)[0]
        self.assertTrue(self.free.covers(Point(approach).buffer(0.85)))
        self.assertGreater(Point(approach).distance(unary_union([v for k,v in self.lines.items() if k.startswith('network/')])),5)

    def test_raster_is_conservative_and_preserves_trace_corridor(self):
        xmin,ymin,xmax,ymax=self.scene['bounds']; res=self.scene['resolution']
        rows,cols=self.image.shape
        self.assertEqual((cols*res,rows*res),(xmax-xmin,ymax-ymin))
        for row in range(0,rows,128):
            yy,xx=np.meshgrid(ymax-(np.arange(row,min(row+128,rows))+0.5)*res,xmin+(np.arange(cols)+0.5)*res,indexing='ij')
            free_pixels=self.image[row:row+128]==254
            for dx,dy in [(-.05,-.05),(-.05,.05),(.05,-.05),(.05,.05)]:
                self.assertTrue(np.all(shapely.intersects_xy(self.free,xx[free_pixels]+dx,yy[free_pixels]+dy)))
            corridor=shapely.intersects_xy(self.trace.buffer(.85),xx,yy)
            self.assertTrue(np.all(free_pixels[corridor]))

    def test_gazebo_obstacle_footprints_match_map(self):
        sdf=ET.parse(HERE/'generated/models/campus_scene/model.sdf')
        specs={o['name']:shape(o['footprint']) for o in self.scene['obstacles']}
        count=0
        for collision in sdf.findall('.//collision'):
            name=collision.attrib['name']
            if name=='ground': continue
            count+=1
            x,y,z,roll,pitch,yaw=map(float,collision.findtext('pose').split())
            size=collision.findtext('geometry/box/size')
            if size:
                w,d,h=map(float,size.split()); footprint=box(x-w/2,y-d/2,x+w/2,y+d/2)
            else:
                radius=float(collision.findtext('geometry/cylinder/radius')); footprint=Point(x,y).buffer(radius,quad_segs=16)
            self.assertLess(footprint.symmetric_difference(specs[name]).area,1e-7)
            self.assertLess(footprint.intersection(self.free).area,1e-7)
            self.assertGreaterEqual(footprint.distance(self.trace),.85)
        self.assertEqual(count,len(specs))

    def test_road_mesh_keeps_courtyard_holes(self):
        _,_,roads,plaza,*_=geometry(self.config)
        doc=ET.parse(HERE/'generated/models/campus_scene/meshes/roads.dae')
        ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
        values=np.array(list(map(float,doc.find('.//c:float_array',ns).text.split()))).reshape(-1,3,3)
        normals=np.cross(values[:,1]-values[:,0],values[:,2]-values[:,0])
        self.assertTrue(np.all(normals[:,2]>0), 'Road surfaces must face up in Gazebo')
        triangles=unary_union([Polygon(tri[:,:2]) for tri in values])
        self.assertLess(triangles.symmetric_difference(roads.difference(plaza)).area,.001)
        self.assertFalse(triangles.covers(Point(40,-25)))

    def test_plaza_requires_actual_obstacle_detour(self):
        tasks={t['name']:t for t in self.scene['tasks']}
        task=tasks['plaza_diagonal']
        direct=LineString([task['start'][:2],task['goal'][:2]])
        planter=next(shape(o['footprint']) for o in self.scene['obstacles'] if o['name']=='plaza_planter')
        self.assertTrue(direct.intersects(planter))
        self.assertGreater(Point(task['goal'][:2]).distance(self.trace),5)
        for task in tasks.values():
            if not task.get('expect_success',True):
                self.assertFalse(self.free.covers(Point(task['goal'][:2])))

    def test_generation_rejects_unplaceable_obstacle(self):
        config=copy.deepcopy(self.config)
        x,y=list(self.lines['raw/1.ubx/0'].coords)[0]
        config['obstacles'].append({'name':'invalid_obstacle','bounds':[x-.2,y-.2,x+.2,y+.2],'height':1,'color':'1 0 0 1'})
        with self.assertRaisesRegex(ValueError,'No safe placement'):
            geometry(config)


if __name__=='__main__': unittest.main()
