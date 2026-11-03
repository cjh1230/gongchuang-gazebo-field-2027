#!/usr/bin/env python3
"""Save actual Gazebo camera images over local Gazebo Transport; no ROS DDS dependency."""
import argparse
import json
from pathlib import Path
import time
from PIL import Image
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as ImageMsg


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--timeout',type=float,default=90)
    p.add_argument('--views',nargs='+',default=['top','isometric','cross','stations','coarse','temporary','obstacles','qr_board','raw_close','stack_close'])
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    node=Node()
    received={}; subscriptions=[]
    def callback(name,msg):
        if name in received or time.monotonic()-start<6: return
        raw=bytes(msg.data)
        encoding=ImageMsg.DESCRIPTOR.fields_by_name['pixel_format_type'].enum_type.values_by_number[msg.pixel_format_type].name
        mode={'RGB_INT8':'RGB','RGBA_INT8':'RGBA','L_INT8':'L','BGR_INT8':'RGB'}.get(encoding)
        if mode is None: raise RuntimeError('Unsupported image encoding '+encoding)
        decoder={'BGR_INT8':'BGR'}.get(encoding,mode)
        img=Image.frombytes(mode,(msg.width,msg.height),raw,'raw',decoder,msg.step)
        img.save(a.output/(name+'.png'))
        received[name]={'source':'Gazebo Harmonic camera sensor via gz.transport13',
            'topic':'/audit/'+name,'width':msg.width,'height':msg.height,
            'sim_stamp_sec':msg.header.stamp.sec,'encoding':encoding}
        print('Saved',name,flush=True)
    names=a.views
    for name in names:
        subscriptions.append(node.subscribe(ImageMsg,'/audit/'+name,lambda msg,n=name:callback(n,msg)))
    start=time.monotonic()
    while len(received)<len(names) and time.monotonic()-start<a.timeout:
        time.sleep(.25)
    (a.output/'capture_metadata.json').write_text(json.dumps(received,indent=2)+'\n')
    for name in names: node.unsubscribe('/audit/'+name)
    if len(received)!=len(names): raise SystemExit('Missing views: '+str(set(names)-received.keys()))


if __name__=='__main__': main()
