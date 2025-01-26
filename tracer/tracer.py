#!/usr/bin/env python3

import usb1
from codecs import encode

import threading
import queue

ctx = usb1.USBContext()
handle = ctx.openByVendorIDAndProductID(0xcafe, 0x4010,skip_on_error=True)
if handle is None:
    raise Exception("usb not found")
iface = handle.claimInterface(0)

q = queue.Queue()

BUFF_SIZE = 64

def printer():
    while True:
        item = q.get()
        for i in range(0,len(item),4):
            data = item[i:i+4]
            #print(encode(data,'hex'))
            op = data[0] >>4
            addr = ((data[0]&0xf)<<16) | (data[1]<<8) | data[2]
            d = data[3]
            print("OP:%01x ADDR 0x%05x DATA: 0x%02x"%(op,addr,d))
        q.task_done()

# Turn-on the worker thread.
threading.Thread(target=printer, daemon=True).start()

def rx_complete(transfer):
    if transfer.getStatus() != usb1.TRANSFER_COMPLETED:
        # Transfer did not complete successfully, there is no data to read.
        # This example does not resubmit transfers on errors. You may want
        # to resubmit in some cases (timeout, ...).
        return
    buff = transfer.getBuffer()[:transfer.getActualLength()]
    
    if len(buff)%4 != 0:
        raise Exception("bufferror "+str(len(buff)))
    q.put(buff)
    transfer.submit()
    
# Build a list of transfer objects and submit them to prime the pump.
transfer_list = []
for _ in range(50):
    transfer = handle.getTransfer()
    transfer.setBulk(
        usb1.ENDPOINT_IN | 0x01,
        bytearray(BUFF_SIZE),
        callback=rx_complete,
        timeout=0,
    )
    transfer.submit()
    transfer_list.append(transfer)
# Loop as long as there is at least one submitted transfer.
while any(x.isSubmitted() for x in transfer_list):
    try:
        ctx.handleEvents()
    except usb1.USBErrorInterrupted:
        pass   
