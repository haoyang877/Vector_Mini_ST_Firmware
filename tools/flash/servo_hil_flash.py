"""Download a HIL image only from confirmed idle; verify image and preserved parameters."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS

from pathlib import Path
import argparse, json, os, sys, time, shutil, hashlib
OUT=ROOT/'outputs/servo_hil_20260908'
try:
    import pylink
except ImportError:
    pylink = None
from elftools.elf.elffile import ELFFile


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


if not __debug__:
    raise RuntimeError('Flash safety checks require Python without -O/-OO')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('name')
parser.add_argument('--session-dir', type=Path, required=True,
    help='Motor-specific image metadata and expected calibration/parameter backup')
parser.add_argument('--bench-id', default=os.environ.get('VECTOR_BENCH_ID'))
parser.add_argument('--motor-profile', type=Path, required=True,
    help='Checked motor/axis profile used to identify this physical target')
parser.add_argument('--scenario', required=True, choices=('install-hil',),
    help='Explicit physical operation being authorized')
parser.add_argument('--operator-confirmation', required=True, choices=('POWER_LIMITS_VERIFIED',),
    help='Confirms physical isolation, limits, and emergency-stop readiness')
parser.add_argument('--expected-image-sha256', required=True,
    help='Expected SHA-256 of --image-axf')
parser.add_argument('--jlink-dll', type=Path,
    default=Path(os.environ['JLINK_DLL']) if os.environ.get('JLINK_DLL') else None)
parser.add_argument('--probe-serial', type=int,
    default=int(os.environ['JLINK_PROBE_SERIAL']) if os.environ.get('JLINK_PROBE_SERIAL') else None)
parser.add_argument('--image-axf', type=Path,
    default=ROOT/'outputs/build/keil/Vector_Mini_ST_HIL/Vector_Mini_ST.axf',
    help='HIL AXF and adjacent HEX to install, including an archived tested image')
parser.add_argument('--installed-normal', type=Path,
    default=ROOT/'outputs/build/keil/Vector_Mini_ST/Vector_Mini_ST.axf',
    help='Exact installed normal-image AXF and adjacent HEX for live handover verification')
args = parser.parse_args()
if pylink is None:
    parser.error('Install pylink-square with uv sync --locked --extra bench; no probe was opened')
if not args.bench_id:
    parser.error('--bench-id or VECTOR_BENCH_ID is required; no probe was opened')
if not args.jlink_dll or not args.jlink_dll.is_file():
    parser.error('--jlink-dll or JLINK_DLL must identify JLink_x64.dll; no probe was opened')
if args.probe_serial is None:
    parser.error('--probe-serial or JLINK_PROBE_SERIAL is required; no probe was opened')
OUT = args.session_dir.resolve()
name=args.name
destination=OUT/name; destination.mkdir(exist_ok=False)
axf=args.image_axf
profile = json.loads(args.motor_profile.read_text(encoding='utf-8'))
actual_axf_sha256 = hashlib.sha256(axf.read_bytes()).hexdigest()
if actual_axf_sha256 != args.expected_image_sha256.lower():
    (destination/'preflight_failure.json').write_text(json.dumps(dict(
        schema_version=1, bench_id=args.bench_id, scenario=args.scenario,
        hardware_contacted=False, failure='HIL image hash differs from explicit authorization',
        expected_image_sha256=args.expected_image_sha256.lower(), actual_image_sha256=actual_axf_sha256), indent=2))
    raise RuntimeError('HIL image hash differs from explicit authorization')
old=json.loads((OUT/'symbols.json').read_text())
with axf.open('rb') as f:
    e=ELFFile(f);symbols=e.get_section_by_name('.symtab')
    names=['servo_hil_mailbox','MotorControl','OnBoard_Encoder','FOC','_SEGGER_RTT',
           'hil_irq_last_cycles','hil_irq_max_cycles','hil_irq_histogram','servo_hil_current_guard']
    new={n:symbols.get_symbol_by_name(n)[0]['st_value'] for n in names if symbols.get_symbol_by_name(n)}
base=0;image={}
for line in axf.with_suffix('.hex').read_text().splitlines():
    b=bytes.fromhex(line[1:]);require(sum(b)%256==0, 'invalid HIL Intel HEX checksum')
    n=b[0];a=int.from_bytes(b[1:3],'big');typ=b[3]
    if typ==4:base=int.from_bytes(b[4:6],'big')<<16
    elif typ==0:
        require(0x08000000<=base+a and base+a+n<=0x0801c000, 'HIL image overlaps parameters or target bounds')
        image.update({base+a+k:x for k,x in enumerate(b[4:4+n])})
j=pylink.JLink(lib=pylink.library.Library(dllpath=str(args.jlink_dll.resolve())))
j.open(args.probe_serial);j.set_tif(pylink.enums.JLinkInterfaces.SWD);j.connect('STM32G431CB',speed=4000)
try:
    require(not j.halted(), 'CPU must be running for live disable verification')
    before = bytes(j.memory_read8(0x08000000,0x20000))
    (destination/'flash_before.bin').write_bytes(before)
    if j.memory_read32(old['servo_hil_mailbox'],1)[0] == 0x48494c31:
        require(j.memory_read32(old['servo_hil_mailbox']+48,1)[0]==0, 'must disable before flashing')
    else:
        # Normal-image handover: identify every load byte before trusting its ELF.
        normal = args.installed_normal
        address_base = 0
        verified = 0
        for line in normal.with_suffix('.hex').read_text().splitlines():
            record = bytes.fromhex(line[1:]); require(sum(record)%256 == 0, 'invalid installed-image HEX checksum')
            size = record[0]; offset = int.from_bytes(record[1:3],'big')
            if record[3] == 4: address_base = int.from_bytes(record[4:6],'big') << 16
            elif record[3] == 0:
                address = address_base + offset
                require(0x08000000 <= address and address+size <= 0x0801c000, 'installed image overlaps parameters or target bounds')
                require(before[address-0x08000000:address-0x08000000+size] == record[4:4+size], 'unidentified installed image')
                verified += size
        require(verified > 0, 'installed image contains no verified load bytes')
        with normal.open('rb') as stream:
            normal_elf = ELFFile(stream)
            motor = normal_elf.get_section_by_name('.symtab').get_symbol_by_name('MotorControl')[0]['st_value']
        offsets = json.loads((OUT/'member_offsets.json').read_text())['MotorControl_TypeDef']
        require(j.memory_read8(motor+offsets['ModeNow'],1)[0] == 0, 'normal firmware must be disabled')
        require(j.memory_read8(motor+offsets['ErrorNow'],1)[0] == 0, 'normal firmware has a fault')
    # STM32G431 bench hardware check: all phase CH1/2/3 main/complementary enables off.
    require(j.memory_read32(0x40012c20,1)[0] & 0x555 == 0, 'phase PWM channels still enabled')

    params=bytes(j.memory_read8(0x0801c000,0x4000))
    expected=OUT/'expected_parameters.bin'
    require(params==(expected.read_bytes() if expected.exists() else
                    (OUT/'original_flash.bin').read_bytes()[0x1c000:]), 'live parameters differ from the authorized backup')
    from servo_hil_emergency import disable_outputs
    disable_outputs(j)  # Gate phase PWM off before the programming tool can halt.
    j.flash_file(str(axf.with_suffix('.hex')),0x08000000)
    j.reset(halt=False);time.sleep(3)
    require(j.memory_read32(new['servo_hil_mailbox'],1)[0]==0x48494c31, 'installed HIL mailbox identity missing')
    require(j.memory_read32(new['servo_hil_mailbox']+48,1)[0]==0, 'installed HIL firmware did not start disabled')
    actual=bytes(j.memory_read8(0x08000000,0x20000))
    require(all(actual[a-0x08000000]==v for a,v in image.items()), 'readback differs from HIL image')
    require(actual[0x1c000:]==params, 'parameters changed during HIL installation')
    (OUT/'symbols.json').write_text(json.dumps(new,indent=2))
    (destination/'symbols.json').write_text(json.dumps(new,indent=2))
    for ext in ['.axf','.hex','.map']:
        shutil.copy2(axf.with_suffix(ext),destination/axf.with_suffix(ext).name)
    (destination/'verification.json').write_text(json.dumps(dict(
        schema_version=1, bench_id=args.bench_id, scenario=args.scenario,
        operator_confirmation=args.operator_confirmation,
        motor_profile=str(args.motor_profile.resolve()), motor_identity=profile.get('name'),
        motor_profile_sha256=hashlib.sha256(args.motor_profile.read_bytes()).hexdigest(),
        authorized_axf_sha256=args.expected_image_sha256.lower(),
        image_verified_bytes=len(image),parameters_unchanged=True,
        flash_sha256=hashlib.sha256(actual).hexdigest()),indent=2))
    (OUT/'active_image.json').write_text(json.dumps(dict(directory=name,
        axf_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
        hex_sha256=hashlib.sha256(axf.with_suffix('.hex').read_bytes()).hexdigest()),indent=2))
    print('Verified',len(image),'image bytes; parameters preserved; motor disabled')
finally:
    j.close()
