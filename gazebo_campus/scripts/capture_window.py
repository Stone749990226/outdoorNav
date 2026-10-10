#!/usr/bin/env python3
"""Capture a WSLg X11 window through XComposite (ordinary grabs may be black).

Find window IDs with: xwininfo -root -tree
Run with Ubuntu Python and python3-pil, or the scene-generation environment.
"""
import argparse
import ctypes as c
from pathlib import Path
from PIL import Image


class XImage(c.Structure):
    _fields_=[('width',c.c_int),('height',c.c_int),('xoffset',c.c_int),('format',c.c_int),
              ('data',c.c_void_p),('byte_order',c.c_int),('bitmap_unit',c.c_int),
              ('bitmap_bit_order',c.c_int),('bitmap_pad',c.c_int),('depth',c.c_int),
              ('bytes_per_line',c.c_int),('bits_per_pixel',c.c_int)]


def capture(window,output):
    x=c.CDLL('libX11.so.6'); composite=c.CDLL('libXcomposite.so.1')
    x.XOpenDisplay.argtypes=[c.c_char_p]; x.XOpenDisplay.restype=c.c_void_p
    display=x.XOpenDisplay(None)
    if not display: raise RuntimeError('No X11 display; start WSLg first')
    composite.XCompositeNameWindowPixmap.argtypes=[c.c_void_p,c.c_ulong]
    composite.XCompositeNameWindowPixmap.restype=c.c_ulong
    x.XGetGeometry.argtypes=[c.c_void_p,c.c_ulong,c.POINTER(c.c_ulong),c.POINTER(c.c_int),c.POINTER(c.c_int),
                            c.POINTER(c.c_uint),c.POINTER(c.c_uint),c.POINTER(c.c_uint),c.POINTER(c.c_uint)]
    x.XGetImage.argtypes=[c.c_void_p,c.c_ulong,c.c_int,c.c_int,c.c_uint,c.c_uint,c.c_ulong,c.c_int]
    x.XGetImage.restype=c.POINTER(XImage)
    x.XDestroyImage.argtypes=[c.POINTER(XImage)]
    x.XFreePixmap.argtypes=[c.c_void_p,c.c_ulong]
    x.XCloseDisplay.argtypes=[c.c_void_p]
    pixmap=composite.XCompositeNameWindowPixmap(display,window)
    image=None
    try:
        root=c.c_ulong(); left=c.c_int(); top=c.c_int()
        width=c.c_uint();height=c.c_uint();border=c.c_uint();depth=c.c_uint()
        if not x.XGetGeometry(display,pixmap,c.byref(root),c.byref(left),c.byref(top),c.byref(width),c.byref(height),c.byref(border),c.byref(depth)):
            raise RuntimeError('Cannot read window geometry')
        image=x.XGetImage(display,pixmap,0,0,width.value,height.value,0xffffffff,2)
        if not image: raise RuntimeError('Cannot capture window pixmap')
        value=image.contents
        if value.bits_per_pixel!=32 or value.byte_order!=0: raise RuntimeError('Unsupported X11 pixel layout')
        raw=c.string_at(value.data,value.bytes_per_line*value.height)
        screenshot=Image.frombytes('RGB',(value.width,value.height),raw,'raw','BGRX',value.bytes_per_line,1)
        if all(low==high for low,high in screenshot.getextrema()): raise RuntimeError('Window returned an empty frame')
        output.parent.mkdir(parents=True,exist_ok=True)
        screenshot.save(output)
        print(f'{output}: {value.width} x {value.height}')
    finally:
        if image: x.XDestroyImage(image)
        if pixmap: x.XFreePixmap(display,pixmap)
        x.XCloseDisplay(display)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('window',type=lambda s:int(s,0))
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    capture(args.window,args.output)
