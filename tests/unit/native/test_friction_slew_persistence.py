"""真实参数函数验证FSL1斜率、CRC、旧布局与标定保留，无硬件访问。"""
import json
import test_motor_load_profile as base
original_fixture = base.fixture


def fixture():
    vector = json.loads((base.ROOT / 'tests/vectors/friction_slew_v1.json').read_text(encoding='utf-8'))['vectors'][0]
    checks = r"""
static void slew_checks(void) {
    InterfaceParam_TypeDef p, saved, bad;
    const unsigned char golden[20] = {GOLDEN};
    assert(offsetof(InterfaceParam_TypeDef,friction_slew)==2292U);
    assert(sizeof(FrictionSlewRecord)==20U && sizeof(InterfaceParam_TypeDef)==2344U);
    assert(offsetof(InterfaceParam_TypeDef,speed_feedforward)==2312U);
    seed(&p);
    assert(MotorControl.friction_attack_slew_a_per_s==2 && MotorControl.friction_release_slew_a_per_s==15);
    assert(memcmp(&p.friction_slew,golden,20)==0);
    MotorControl.friction_attack_slew_a_per_s=.5f; MotorControl.friction_release_slew_a_per_s=3;
    Param_Upload(&saved);saved.magic_word=MAGIC_WORD;
    assert(memcmp(&p,&saved,offsetof(InterfaceParam_TypeDef,friction_slew))==0);
    assert(!Param_Download(&saved));
    assert(MotorControl.friction_attack_slew_a_per_s==.5f && MotorControl.friction_release_slew_a_per_s==3);
    assert(OnBoard_Encoder.electrical_zero_q15==12480 && OnBoard_Encoder.mechanical_zero_q15==213);
    for(unsigned bit=0;bit<160;bit++) {
        bad=saved;((unsigned char*)&bad.friction_slew)[bit/8]^=(unsigned char)(1U<<(bit%8));
        assert(!Param_Download(&bad) && !MotorControl.axis_profile_valid);
    }
    for(unsigned erased=0;erased<2;erased++) {
        bad=saved;memset(&bad.friction_slew,erased?255:0,20);
        assert(!Param_Download(&bad) && MotorControl.axis_profile_valid);
        assert(MotorControl.friction_attack_slew_a_per_s==2 && MotorControl.friction_release_slew_a_per_s==15);
    }
    const float invalid[]={0,-1,NAN,INFINITY,1001};
    for(unsigned i=0;i<5;i++) {
        for(unsigned which=0;which<2;which++) {
            bad=saved;
            if(which)bad.friction_slew.attack_a_per_s=invalid[i];else bad.friction_slew.release_a_per_s=invalid[i];
            bad.friction_slew.crc32=friction_slew_crc(&bad.friction_slew);
            assert(!Param_Download(&bad) && !MotorControl.axis_profile_valid);
        }
    }
    MotorControl.friction_attack_slew_a_per_s=1000;MotorControl.friction_release_slew_a_per_s=1000;
    Param_Upload(&saved);saved.magic_word=MAGIC_WORD;assert(!Param_Download(&saved));
    assert(MotorControl.axis_profile_valid && MotorControl.friction_release_slew_a_per_s==1000);
    puts("PASS FSL1 golden CRC/offset, live values roundtrip, preserved legacy/calibration, blank default, 160 corrupt bits and invalid/range values");
}
"""
    checks=checks.replace('GOLDEN',','.join('0x'+vector['hex'][i:i+2] for i in range(0,40,2)))
    return original_fixture().replace('int main(void) {',checks+'\nint main(void) {\n    slew_checks();')

if __name__=='__main__':
    base.fixture=fixture
    base.main()
