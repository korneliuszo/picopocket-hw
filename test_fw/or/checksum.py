#!/usr/bin/env python3

import sys
import os

SIZE=4*1024

a=open(sys.argv[1],"rb").read()

print("Size:",len(a));
if len(a) > SIZE-1:
    print("Too big")
    os.remove(sys.argv[1])
    exit(1)

a=a+b"\x00"*(SIZE-len(a)-1)
a=a+bytes([(-sum(a))%256])

open(sys.argv[1],"wb").write(a)
