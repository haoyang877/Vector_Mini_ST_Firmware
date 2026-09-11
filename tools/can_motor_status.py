"""Encode stream commands / decode 48-byte motor status; no hardware access."""
import argparse
import json
import math
import struct

FORMAT = '>HHiiiihhiihh12x'
ID_BASE = 0x7F0
FIELDS = ('fault','mode','position_target_rad','position_feedback_rad',
          'speed_target_rad_s','speed_feedback_rad_s','iq_reference_A','iq_feedback_A',
          'position_planned_rad','speed_planned_rad_s','temperature_C','bus_voltage_V')
SCALES = (None,None,1000,1000,100,100,1000,1000,1000,100,100,100)


def command(node, value):
    if type(node) is not int or not 0 <= node <= 7:
        raise ValueError('node must be 0..7')
    if not math.isfinite(value) or not (value in (0,1) or 10 <= value <= 200 and value == int(value)):
        raise ValueError('command: 0=stop, 1=resume, integer 10..200=Hz and start')
    return (node<<8)|0x64, struct.pack('>f',value)


def decode(identifier, payload):
    if not ID_BASE <= identifier <= ID_BASE+7 or len(payload) != 48:
        raise ValueError('status requires standard ID 0x7F0..0x7F7 and 48 bytes')
    values=struct.unpack(FORMAT,payload)
    result={'node':identifier-ID_BASE,'invalid_fields':[]}
    for index,(key,value) in enumerate(zip(FIELDS,values)):
        sentinel = -32768 if index in (6,7,10,11) else -2147483648
        if index < 2: result[key]=value
        elif value == sentinel:
            result[key]=None;result['invalid_fields'].append(key)
        else: result[key]=value/SCALES[index]
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    c=sub.add_parser('command');c.add_argument('--node',type=int,required=True);c.add_argument('--value',type=float,required=True)
    d=sub.add_parser('decode');d.add_argument('--id',type=lambda x:int(x,0),required=True);d.add_argument('--data',required=True)
    args=parser.parse_args()
    if args.action=='command':
        identifier,payload=command(args.node,args.value)
        result={'id':hex(identifier),'data':payload.hex(' '),'length':4}
    else: result=decode(args.id,bytes.fromhex(args.data))
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
