#!/usr/bin/python
#! -*- encoding: utf-8 -*-

binaryDir = "/home/sai/rsrch/loc/coloc/build/bin"
OPENMVG_SFM_BIN = "/home/sai/software/buildMVG/Linux-x86_64-RELEASE/"

intrinsics = "320;0;320;0;320;240;0;0;1"

import os
import subprocess
import sys
'''
if len(sys.argv) < 3:
    print ("Usage %s image_dir output_dir" % sys.argv[0])
    sys.exit(1)
'''
input_dir = "/home/sai/Dropbox/AirSimData/Sep8/newmap"
output_dir = input_dir
matches_dir = os.path.join(output_dir, "matches")
reconstruction_dir = os.path.join(output_dir, "reconstruction_sequential")
query_dir = "/home/sai/output/"
output_dir = os.path.join(query_dir, "output")
omatches_dir = os.path.join(query_dir, "features")

# Create the ouput/matches folder if not present
if not os.path.exists(output_dir):
  os.mkdir(output_dir)
if not os.path.exists(omatches_dir):
  os.mkdir(omatches_dir)

print ("1. Localization")
pLocalize = subprocess.Popen( [os.path.join(binaryDir, "localizer"),  "-i", reconstruction_dir+"/SfM_Data.bin", "-m", matches_dir, "-o", output_dir, "-u", omatches_dir, "-q", query_dir, "-s", "1"] )
pLocalize.wait()
