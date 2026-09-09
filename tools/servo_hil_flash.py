"""Download a HIL image only from confirmed idle; verify image and preserved parameters."""
from pathlib import Path
import argparse, json, sys, time, shutil, hashlib
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/servo_hil_20260908'
sys.path.insert(0,str(OUT/'.deps'))
import pylink
from elftools.elf.elffile import ELFFile
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('name')
parser.add_argument('--session-dir', type=Path,
    help='Motor-specific image metadata and expected calibration/parameter backup')
parser.add_argument('--image-axf', type=Path,
    default=ROOT/'MDK-ARM/Vector_Mini_ST_HIL/Vector_Mini_ST.axf',
    help='HIL AXF and adjacent HEX to install, including an archived tested image')
parser.add_argument('--installed-normal', type=Path,
    default=ROOT/'MDK-ARM/Vector_Mini_ST/Vector_Mini_ST.axf',
    help='Exact installed normal-image AXF and adjacent HEX for live handover verification')
args = parser.parse_args()
if args.session_dir:
    OUT = args.session_dir.resolve()
name=args.name
destination=OUT/name; destination.mkdir(exist_ok=False)
axf=args.image_axf
old=json.loads((OUT/'symbols.json').read_text())
with axf.open('rb') as f:
    e=ELFFile(f);symbols=e.get_section_by_name('.symtab')
    names=['servo_hil_mailbox','MotorControl','OnBoard_Encoder','FOC','_SEGGER_RTT',
           'hil_irq_last_cycles','hil_irq_max_cycles','hil_irq_histogram','servo_hil_current_guard']
    new={n:symbols.get_symbol_by_name(n)[0]['st_value'] for n in names if symbols.get_symbol_by_name(n)}
base=0;image={}
for line in axf.with_suffix('.hex').read_text().splitlines():
    b=bytes.fromhex(line[1:]);assert sum(b)%256==0
    n=b[0];a=int.from_bytes(b[1:3],'big');typ=b[3]
    if typ==4:base=int.from_bytes(b[4:6],'big')<<16
    elif typ==0:
        assert 0x08000000<=base+a and base+a+n<=0x0801c000
        image.update({base+a+k:x for k,x in enumerate(b[4:4+n])})
j=pylink.JLink(lib=pylink.library.Library(dllpath='C:/Program Files/SEGGER/JLink_V964/JLink_x64.dll'))
j.open(602722271);j.set_tif(pylink.enums.JLinkInterfaces.SWD);j.connect('STM32G431CB',speed=4000)
try:
    assert not j.halted(), 'CPU must be running for live disable verification'
    before = bytes(j.memory_read8(0x08000000,0x20000))
    (destination/'flash_before.bin').write_bytes(before)
    if j.memory_read32(old['servo_hil_mailbox'],1)[0] == 0x48494c31:
        assert j.memory_read32(old['servo_hil_mailbox']+48,1)[0]==0, 'must disable before flashing'
    else:
        # Normal-image handover: identify every load byte before trusting its ELF.
        normal = args.installed_normal
        address_base = 0
        verified = 0
        for line in normal.with_suffix('.hex').read_text().splitlines():
            record = bytes.fromhex(line[1:]); assert sum(record)%256 == 0
            size = record[0]; offset = int.from_bytes(record[1:3],'big')
            if record[3] == 4: address_base = int.from_bytes(record[4:6],'big') << 16
            elif record[3] == 0:
                address = address_base + offset
                assert 0x08000000 <= address and address+size <= 0x0801c000
                assert before[address-0x08000000:address-0x08000000+size] == record[4:4+size], 'unidentified installed image'
                verified += size
        assert verified > 0
        with normal.open('rb') as stream:
            normal_elf = ELFFile(stream)
            motor = normal_elf.get_section_by_name('.symtab').get_symbol_by_name('MotorControl')[0]['st_value']
        offsets = json.loads((OUT/'member_offsets.json').read_text())['MotorControl_TypeDef']
        assert j.memory_read8(motor+offsets['ModeNow'],1)[0] == 0, 'normal firmware must be disabled'
        assert j.memory_read8(motor+offsets['ErrorNow'],1)[0] == 0, 'normal firmware has a fault'
    # STM32G431 bench hardware check: all phase CH1/2/3 main/complementary enables off.
    assert j.memory_read32(0x40012c20,1)[0] & 0x555 == 0, 'phase PWM channels still enabled'

    params=bytes(j.memory_read8(0x0801c000,0x4000))
    expected=OUT/'expected_parameters.bin'
    assert params==(expected.read_bytes() if expected.exists() else
                    (OUT/'original_flash.bin').read_bytes()[0x1c000:])
    from servo_hil_emergency import disable_outputs
    disable_outputs(j)  # Gate phase PWM off before the programming tool can halt.
    j.flash_file(str(axf.with_suffix('.hex')),0x08000000)
    j.reset(halt=False);time.sleep(3)
    assert j.memory_read32(new['servo_hil_mailbox'],1)[0]==0x48494c31
    assert j.memory_read32(new['servo_hil_mailbox']+48,1)[0]==0
    actual=bytes(j.memory_read8(0x08000000,0x20000))
    assert all(actual[a-0x08000000]==v for a,v in image.items())
    assert actual[0x1c000:]==params
    (OUT/'symbols.json').write_text(json.dumps(new,indent=2))
    (destination/'symbols.json').write_text(json.dumps(new,indent=2))
    for ext in ['.axf','.hex','.map']:
        shutil.copy2(axf.with_suffix(ext),destination/axf.with_suffix(ext).name)
    (destination/'verification.json').write_text(json.dumps(dict(
        image_verified_bytes=len(image),parameters_unchanged=True,
        flash_sha256=hashlib.sha256(actual).hexdigest()),indent=2))
    (OUT/'active_image.json').write_text(json.dumps(dict(directory=name,
        axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
        hex_sha256=hashlib.sha256(axf.with_suffix('.hex').read_bytes()).hexdigest()),indent=2))
    print('Verified',len(image),'image bytes; parameters preserved; motor disabled')
finally:
    j.close()
