"""离线验证两组速度 PI 的保存、旧值迁移、AXS2恢复和 CAN 参数隔离。"""

import json

import test_motor_load_profile as load_profile

original_fixture = load_profile.fixture


def fixture() -> str:
    """复用生产参数及 CAN 分派夹具，不连接设备。"""
    vectors = json.loads(
        (load_profile.ROOT / "tests/vectors/position_speed_pi_v1.json").read_text(encoding="utf-8")
    )
    checks = r"""
static void separate_pi_checks(void) {
    InterfaceParam_TypeDef p, saved;
    assert(sizeof(PositionSpeedPiRecord)==20U);
    assert(offsetof(InterfaceParam_TypeDef,position_speed_pi)==2272U);
    assert(sizeof(InterfaceParam_TypeDef)==2332U);
    assert(offsetof(InterfaceParam_TypeDef,speed_kp)==2116U);
    assert(offsetof(InterfaceParam_TypeDef,axis_profile)==2208U);
    seed(&p);
    MotorControl.speed_Kp=.06f; MotorControl.speed_Ki=.6f;
    MotorControl.position_speed_Kp=.5f; MotorControl.position_speed_Ki=1;
    Param_Upload(&saved); saved.magic_word=MAGIC_WORD;
    const unsigned char golden[]={GOLDEN_BYTES};
    assert(memcmp(&saved.position_speed_pi,golden,sizeof(golden))==0);
    MotorControl.speed_Kp=MotorControl.position_speed_Kp=9;
    assert(!Param_Download(&saved));
    assert(MotorControl.speed_Kp==.06f && MotorControl.speed_Ki==.6f);
    assert(MotorControl.position_speed_Kp==.5f && MotorControl.position_speed_Ki==1);
    assert(OnBoard_Encoder.electrical_zero_q15==12480 && OnBoard_Encoder.linearization_lut_q15[71]==-42);
    /* 新尾记录优先于 AXS2 控制基线，纯速度及位置增益都能独立保存。 */
    assert(MotorAxisProfile_CreateJoint(&saved.axis_profile,MOTOR_JOINT_PITCH,1,-.3f,.9f,.7f));
    assert(!Param_Download(&saved));
    assert(MotorControl.speed_Kp==.06f && MotorControl.speed_Ki==.6f);
    assert(MotorControl.position_speed_Kp==.5f && MotorControl.position_speed_Ki==1);
    Param_Upload(&saved); saved.magic_word=MAGIC_WORD;
    assert(!Param_Download(&saved) && MotorControl.position_speed_Ki==1);
    for (unsigned erased=0;erased<2;erased++) {
        seed(&p); memset(&p.position_speed_pi,erased?255:0,sizeof(p.position_speed_pi));
        assert(Param_Download(&p));
        assert(MotorControl.speed_Kp==.071f && MotorControl.speed_Ki==.64f);
        assert(MotorControl.position_speed_Kp==.071f && MotorControl.position_speed_Ki==.64f);
        assert(OnBoard_Encoder.calib_flag==7 && OnBoard_Encoder.mechanical_zero_q15==213);
        Param_Upload(&saved); saved.magic_word=MAGIC_WORD;
        assert(!Param_Download(&saved));
        /* 旧 AXS2 启动曾覆盖 PI；迁移保存同样的实际值，避免首次升级跳变。 */
        assert(MotorAxisProfile_CreateJoint(&p.axis_profile,MOTOR_JOINT_PITCH,1,-.3f,.9f,.7f));
        assert(Param_Download(&p));
        assert(MotorControl.speed_Kp==.5f && MotorControl.speed_Ki==2);
        assert(MotorControl.position_speed_Kp==.5f && MotorControl.position_speed_Ki==2);
    }
    seed(&p);
    for (unsigned bit=0;bit<160;bit++) {
        saved=p;
        ((unsigned char*)&saved.position_speed_pi)[bit/8]^=(unsigned char)(1U<<(bit%8));
        assert(!Param_Download(&saved) && !MotorControl.axis_profile_valid);
    }
    const float bad[]={NAN,INFINITY,-.01f};
    for (unsigned i=0;i<3;i++) {
        for (unsigned field=0;field<2;field++) {
            saved=p;
            if(field) saved.position_speed_pi.ki_per_rad=bad[i];
            else saved.position_speed_pi.kp_s_per_rad=bad[i];
            saved.position_speed_pi.crc32=position_speed_pi_crc(&saved.position_speed_pi);
            assert(!Param_Download(&saved) && !MotorControl.axis_profile_valid);
        }
    }
    saved=p; saved.position_speed_pi.version=2;
    saved.position_speed_pi.crc32=position_speed_pi_crc(&saved.position_speed_pi);
    assert(!Param_Download(&saved) && !MotorControl.axis_profile_valid);
    seed(&p);
    CAN_ReceiveMessage_Update(CAN_SET_POSITION_SPEED_KP,.5f);
    CAN_ReceiveMessage_Update(CAN_SET_POSITION_SPEED_KI,1);
    assert(MotorControl.speed_Kp==.071f && MotorControl.speed_Ki==.64f);
    assert(MotorControl.position_speed_Kp==.5f && MotorControl.position_speed_Ki==1);
    CAN_ReceiveMessage_Update(CAN_GET_POSITION_SPEED_KP,0);
    assert(reply_id==CAN_GET_POSITION_SPEED_KP && reply_value==.5f);
    CAN_ReceiveMessage_Update(CAN_GET_POSITION_SPEED_KI,0);
    assert(reply_id==CAN_GET_POSITION_SPEED_KI && reply_value==1);
    for (unsigned i=0;i<3;i++) {
        CAN_ReceiveMessage_Update(CAN_SET_POSITION_SPEED_KP,bad[i]);
        CAN_ReceiveMessage_Update(CAN_SET_POSITION_SPEED_KI,bad[i]);
        assert(MotorControl.position_speed_Kp==.5f && MotorControl.position_speed_Ki==1);
    }
    CAN_ReceiveMessage_Update(CAN_SET_POSITION_SPEED_KI,0);
    assert(MotorControl.position_speed_Ki==0 && MotorControl.speed_Ki==.64f);
    puts("PASS PSP1 golden layout/CRC, separate save/load, legacy migration, AXS2, corruption and CAN isolation");
}
"""
    checks = checks.replace(
        "GOLDEN_BYTES",
        ",".join(f"0x{byte:02x}" for byte in bytes.fromhex(vectors["vectors"][0]["hex"])),
    )
    return original_fixture().replace(
        "int main(void) {", checks + "\nint main(void) {\n    separate_pi_checks();"
    )


if __name__ == "__main__":
    load_profile.fixture = fixture
    load_profile.main()
