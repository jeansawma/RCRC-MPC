"""Validate Intel HEX records, address bounds, boot vectors and BIN correspondence."""
from pathlib import Path
import hashlib
import struct
import sys

def verify(path):
    path=Path(path); memory={}; base=0; ended=False; count=0
    for line in path.read_text().splitlines():
        assert not ended,"data follows EOF"
        assert line.startswith(":"),"invalid record marker"
        record=bytes.fromhex(line[1:]); assert sum(record)%256==0,"record checksum"
        length=record[0]; assert len(record)==length+5,"record length"
        address=int.from_bytes(record[1:3],"big"); kind=record[3]; data=record[4:-1]
        if kind==0:
            for offset,value in enumerate(data):
                absolute=base+address+offset
                assert 0x08000000<=absolute<0x08100000,"outside STM32H723ZG flash"
                assert absolute not in memory,"overlapping HEX records"
                memory[absolute]=value
            count+=1
        elif kind==1: assert length==0; ended=True
        elif kind==4: assert length==2; base=int.from_bytes(data,"big")<<16
        elif kind==2: assert length==2; base=int.from_bytes(data,"big")<<4
        elif kind in (3,5): assert length==4
        else: raise AssertionError(f"unsupported record {kind}")
    assert ended and memory,"missing EOF or firmware"
    start=min(memory);assert start==0x08000000,"incorrect link address"
    stack,reset=struct.unpack("<II",bytes(memory[start+i] for i in range(8)))
    assert 0x20000000<stack<=0x20020000 and stack%8==0,"DTCM stack vector"
    assert reset&1 and start<=reset<0x08100000,"Thumb reset vector"
    binary=path.with_suffix(".bin").read_bytes()
    assert len(binary)==max(memory)-start+1,"BIN image length"
    assert all(binary[address-start]==value for address,value in memory.items()),"HEX/BIN mismatch"
    print(f"HEX verified: {count} data records, {len(memory)} bytes, MSP=0x{stack:08X}, Reset=0x{reset:08X}")
    print("SHA256",hashlib.sha256(path.read_bytes()).hexdigest())

if __name__=="__main__": verify(sys.argv[1] if len(sys.argv)>1 else "build/rcrc_h723.hex")
