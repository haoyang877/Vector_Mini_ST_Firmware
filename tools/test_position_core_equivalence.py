"""Run the position scenarios against old/new cores and compare every state byte.

Reference sources and instrumented wrappers exist only under --out. No test
accessors or reference implementation are linked into firmware.
"""
import argparse
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--reference-ref', default='3980f92')
    parser.add_argument('--reference-source', type=Path,
                        help='Optional frozen pre-change core, instead of the Git reference')
    parser.add_argument('--out', type=Path, default=ROOT / 'outputs/position_core_equivalence')
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    header = (ROOT / 'Foc/position_cascade.h').read_text()
    names = sorted(set(re.findall(r'\bPositionCascade_\w+(?=\()', header)))
    old_header = header
    old = (args.reference_source.read_text() if args.reference_source else
           subprocess.check_output(['git', 'show', args.reference_ref + ':Foc/position_cascade.c'],
                                   cwd=ROOT).decode())
    for name in names:
        old = re.sub(r'\b' + name + r'\b', name + '_Reference', old)
        old_header = re.sub(r'\b' + name + r'\b', name + '_Reference', old_header)
    # Use the original type declarations once, followed by reference prototypes.
    prototypes = old_header[old_header.index('/** Reset'):old_header.rindex('#endif')]
    old = old.replace('#include "position_cascade.h"', '#include "position_cascade.h"\n' + prototypes)
    old += '\nconst void *PositionCore_ReferenceState(void) { return &state; }\n'
    old += 'size_t PositionCore_ReferenceSize(void) { return sizeof(state); }\n'
    actual = (ROOT / 'Foc/position_cascade.c').read_text()
    for name in ['Reset', 'Update', 'UpdateTarget', 'UpdateControl', 'UpdateTargetControl']:
        actual = actual.replace('PositionCascade_' + name + '(', 'PositionCascade_' + name + '_Actual(')
    actual += '\n' + prototypes + r'''
#include <stdio.h>
#include <stdlib.h>
extern const void *PositionCore_ReferenceState(void);
extern size_t PositionCore_ReferenceSize(void);
unsigned long PositionCore_ComparedTicks;
static void compare_state(void) {
    const unsigned char *a = (const unsigned char *)&state;
    const unsigned char *b = (const unsigned char *)PositionCore_ReferenceState();
    size_t i;
    if (sizeof(state) != PositionCore_ReferenceSize()) abort();
    for (i=0; i<sizeof(state); ++i) if (a[i] != b[i]) {
        fprintf(stderr,"State mismatch tick %lu byte %zu: %02x / %02x\n",
            PositionCore_ComparedTicks,i,a[i],b[i]); abort();
    }
}
void PositionCascade_Reset(void) {
    PositionCascade_Reset_Actual(); PositionCascade_Reset_Reference(); compare_state();
}
static bool compare_result(bool a, bool b, PositionCascadeOutput_TypeDef *output,
    const PositionCascadeOutput_TypeDef *actual_output,
    const PositionCascadeOutput_TypeDef *reference_output) {
    ++PositionCore_ComparedTicks;
    if (a != b || (output && memcmp(actual_output,reference_output,sizeof(*output)))) {
        fprintf(stderr,"Output mismatch tick %lu\n",PositionCore_ComparedTicks); abort();
    }
    compare_state();
    if (output) *output=*actual_output;
    return a;
}
bool PositionCascade_Update(const PositionCascadeConfig_TypeDef *config,
    float position, float speed, PositionCascadeOutput_TypeDef *output) {
    PositionCascadeOutput_TypeDef a={0},b={0};
    bool x=PositionCascade_Update_Actual(config,position,speed,output ? &a : NULL);
    bool y=PositionCascade_Update_Reference(config,position,speed,output ? &b : NULL);
    return compare_result(x,y,output,&a,&b);
}
bool PositionCascade_UpdateTarget(float target,float position,float speed,
    PositionCascadeOutput_TypeDef *output) {
    PositionCascadeOutput_TypeDef a={0},b={0};
    bool x=PositionCascade_UpdateTarget_Actual(target,position,speed,output ? &a : NULL);
    bool y=PositionCascade_UpdateTarget_Reference(target,position,speed,output ? &b : NULL);
    return compare_result(x,y,output,&a,&b);
}
static bool compare_control(bool x,bool y,PositionCascadeControlOutput_TypeDef *output,
    const PositionCascadeControlOutput_TypeDef *a,const PositionCascadeOutput_TypeDef *b) {
    PositionCascadeControlOutput_TypeDef expected={0};
    expected.position_reference=b->position_reference;
    expected.speed_reference=b->speed_reference;
    expected.speed_feedback=b->speed_feedback;
    expected.iq_reference=b->iq_reference;
    expected.target_reached=b->target_reached;
    ++PositionCore_ComparedTicks;
    if(x!=y || memcmp(a,&expected,sizeof(expected))) abort();
    compare_state();
    if(output && x) *output=*a;
    return x;
}
bool PositionCascade_UpdateControl(const PositionCascadeConfig_TypeDef *config,
    float position,float speed,PositionCascadeControlOutput_TypeDef *output) {
    PositionCascadeControlOutput_TypeDef a={0}; PositionCascadeOutput_TypeDef b={0};
    bool x=PositionCascade_UpdateControl_Actual(config,position,speed,output ? &a : NULL);
    bool y=PositionCascade_Update_Reference(config,position,speed,output ? &b : NULL);
    return compare_control(x,y,output,&a,&b);
}
bool PositionCascade_UpdateTargetControl(float target,float position,float speed,
    PositionCascadeControlOutput_TypeDef *output) {
    PositionCascadeControlOutput_TypeDef a={0}; PositionCascadeOutput_TypeDef b={0};
    bool x=PositionCascade_UpdateTargetControl_Actual(target,position,speed,output ? &a : NULL);
    bool y=PositionCascade_UpdateTarget_Reference(target,position,speed,output ? &b : NULL);
    return compare_control(x,y,output,&a,&b);
}
/* Run the original scenario assertions through the compact production API too.
 * Restore rich outputs only in this host bridge, from the same private state. */
bool PositionCore_CompactBridge(const PositionCascadeConfig_TypeDef *config,
    float position,float speed,PositionCascadeOutput_TypeDef *output) {
    PositionCascadeControlOutput_TypeDef compact;
    bool ok=PositionCascade_UpdateControl(config,position,speed,output ? &compact : NULL);
    if(ok && output) PositionCascade_CopyOutput(output);
    return ok;
}
'''
    fixture = (ROOT / 'tests/unit/position_servo_test.c').read_text().replace('int main(', 'int scenarios_main(')
    fixture += r'''
extern unsigned long PositionCore_ComparedTicks;
int main(int argc,char **argv) {
    int result=scenarios_main(argc,argv);
    PositionCascadeControlOutput_TypeDef output;
    PositionCascadeConfig_TypeDef tuning=config();
    unsigned tick;
    PositionCascade_Reset();
    assert(!PositionCascade_UpdateTargetControl(0,0,0,&output));
    tuning.call_divider=10;
    assert(PositionCascade_UpdateControl(&tuning,0,0,&output));
    for(tick=0;tick<2400;++tick) {
        float target=tick<800 ? .1f : tick<1600 ? -.1f : -0.0f;
        float position=.001f*(float)((int)(tick%17)-8);
        float speed=.002f*(float)((int)(tick%11)-5);
        assert(PositionCascade_UpdateTargetControl(target,position,speed,&output));
    }
    assert(!PositionCascade_UpdateTargetControl(NAN,0,0,&output));
    assert(!PositionCascade_UpdateTargetControl(0,NAN,0,&output));
    assert(!PositionCascade_UpdateTargetControl(0,0,NAN,&output));
    assert(!PositionCascade_UpdateTargetControl(0,0,0,NULL));
    printf("PASS %lu ticks: complete private state and outputs match reference byte-for-byte\n",
        PositionCore_ComparedTicks);
    return result;
}
'''
    paths = []
    for name, source in [('reference.c', old), ('actual.c', actual), ('scenarios.c', fixture)]:
        path = out / name
        path.write_text(source)
        paths.append(str(path))
    cc = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else [])
    exe = out / 'equivalence.exe'
    command = cc + ['-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-I', 'Foc'] + paths
    command += ['Foc/position_smooth_trajectory.c', 'Foc/foc_pid.c', '-o', str(exe)]
    logs = []
    compact_fixture = fixture.replace('PositionCascade_Update(', 'PositionCore_CompactBridge(')
    compact_fixture = compact_fixture.replace('#include "position_cascade.h"', '''#include "position_cascade.h"
bool PositionCore_CompactBridge(const PositionCascadeConfig_TypeDef *,float,float,
    PositionCascadeOutput_TypeDef *);''')
    compact_path = out / 'compact_scenarios.c'
    compact_path.write_text(compact_fixture)
    compact_exe = out / 'compact_equivalence.exe'
    compact_command = [str(compact_path) if x == paths[-1] else
                       str(compact_exe) if x == str(exe) else x for x in command]
    for cmd in [command, [str(exe)], compact_command, [str(compact_exe)]]:
        result = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True)
        logs.append(result.stdout + result.stderr)
        (out / 'test.log').write_text('\n'.join(logs))
        print(logs[-1], end='')
        result.check_returncode()


if __name__ == '__main__':
    main()
