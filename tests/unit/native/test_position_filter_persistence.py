"""复用生产参数夹具验证PVF1持久化、旧布局和损坏记录，绝不接触硬件。"""

import json
import struct

import test_motor_load_profile as load_profile

original_fixture = load_profile.fixture


def fixture():
    """在现有参数生产函数夹具中增加独立滤波记录验收。"""
    vectors = json.loads(
        (load_profile.ROOT / "tests/vectors/position_filter_v1.json").read_text(encoding="utf-8")
    )
    checks = r"""
static void filter_checks(void) {
    InterfaceParam_TypeDef baseline, saved, expected, bad;
    assert(sizeof(PositionFilterRecord)==16U);
    assert(offsetof(InterfaceParam_TypeDef,position_filter)==2256U);
    assert(sizeof(InterfaceParam_TypeDef)==2292U);
    seed(&baseline);
    assert(!MotorControl.position_velocity_filter_half_cutoff);
    const uint32_t golden_crc[2]={GOLDEN_CRC_0,GOLDEN_CRC_1};
    for (unsigned value=0;value<2;value++) {
        seed(&baseline);
        MotorControl.position_velocity_filter_half_cutoff=value!=0;
        MotorControl.current_limit=8;
        Param_Upload(&saved); saved.magic_word=MAGIC_WORD;
        assert(saved.position_filter.magic==PARAM_POSITION_FILTER_MAGIC);
        assert(saved.position_filter.version==1 && saved.position_filter.half_cutoff==value);
        assert(saved.position_filter.crc32==golden_crc[value]);
        expected=baseline; expected.current_limit=8; expected.position_filter=saved.position_filter;
        assert(memcmp(&saved,&expected,sizeof(saved))==0);
        MotorControl.position_velocity_filter_half_cutoff=value==0;
        assert(!Param_Download(&saved));
        assert(MotorControl.axis_profile_valid && MotorControl.load_profile_valid);
        assert(MotorControl.position_velocity_filter_half_cutoff==(value!=0));
        assert(MotorControl.current_limit==8 && OnBoard_Encoder.linearization_lut_q15[71]==-42);
        assert(OnBoard_Encoder.electrical_zero_q15==12480 && OnBoard_Encoder.mechanical_zero_q15==213);
        for (unsigned bit=0;bit<128;bit++) {
            PositionFilterRecord damaged=saved.position_filter;
            bool output=value==0;
            ((unsigned char*)&damaged)[bit/8]^=(unsigned char)(1U<<(bit%8));
            assert(!position_filter_load(&damaged,&output) && output==(value==0));
        }
    }
    seed(&baseline);
    for (unsigned erased=0;erased<2;erased++) {
        saved=baseline; memset(&saved.position_filter,erased?255:0,sizeof(saved.position_filter));
        MotorControl.position_velocity_filter_half_cutoff=true;
        assert(!Param_Download(&saved));
        assert(!MotorControl.position_velocity_filter_half_cutoff && MotorControl.axis_profile_valid);
    }
    bad=baseline; bad.position_filter.half_cutoff=2;
    bad.position_filter.crc32=position_filter_crc(&bad.position_filter);
    assert(!Param_Download(&bad) && !MotorControl.axis_profile_valid);
    bad=baseline; bad.position_filter.version=2;
    bad.position_filter.crc32=position_filter_crc(&bad.position_filter);
    assert(!Param_Download(&bad) && !MotorControl.axis_profile_valid);
    bad=baseline; bad.position_filter.crc32^=1;
    assert(!Param_Download(&bad) && !MotorControl.axis_profile_valid);
    MotorControl.position_velocity_filter_half_cutoff=true;
    Param_Return_Default(); assert(!MotorControl.position_velocity_filter_half_cutoff);
    puts("PASS PVF1 golden CRC, roundtrip, unchanged calibration/offsets, erased legacy, corrupt/unsupported rejection");
}
"""
    for vector in vectors["vectors"]:
        crc = struct.unpack("<4I", bytes.fromhex(vector["hex"]))[3]
        checks = checks.replace(f"GOLDEN_CRC_{vector['half_cutoff']}", f"0x{crc:08x}U")
    return original_fixture().replace(
        "int main(void) {", checks + "\nint main(void) {\n    filter_checks();"
    )


if __name__ == "__main__":
    load_profile.fixture = fixture
    load_profile.main()
