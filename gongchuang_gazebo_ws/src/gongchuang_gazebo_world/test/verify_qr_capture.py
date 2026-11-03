#!/usr/bin/env python3
"""Optional OpenCV check of the QR code in an actual Gazebo screenshot."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

p=argparse.ArgumentParser(); p.add_argument('image',type=Path); p.add_argument('--output',type=Path,required=True)
p.add_argument('--bundle',type=Path,default=Path(__file__).resolve().parents[1])
a=p.parse_args(); root=Path(__file__).resolve().parents[1]
expected=json.loads((a.bundle/'config/generated_mission.json').read_text())['task_code']
image=cv2.imdecode(np.frombuffer(a.image.read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)
payload,points,_=cv2.QRCodeDetector().detectAndDecode(image)
assert payload==expected,(payload,expected)
a.output.write_text(json.dumps({'source_image':a.image.name,'expected':expected,'decoded':payload,'pass':True,'detector':'OpenCV QRCodeDetector'},indent=2)+'\n')
print('Actual Gazebo QR decode PASS:',payload)
