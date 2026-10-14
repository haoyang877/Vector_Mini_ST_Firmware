"""离线执行前馈、原速度路径、真实参数服务和CAN停机准入；不连接设备。"""

import json
import subprocess

import test_motor_load_profile as load_profile
from run_position_servo_tests import function_source

original_fixture = load_profile.fixture


def fixture() -> str:
    """复用现有生产参数夹具，增加前馈边界和兼容验证。"""
    source = (load_profile.ROOT / "firmware/app/foc_run.c").read_text(encoding="utf-8")
    old = subprocess.check_output(
        ["git", "show", "af70e737:firmware/app/foc_run.c"], cwd=load_profile.ROOT
    ).decode("utf-8")
    controls = function_source(source, "SpeedMode_UpdateControl") + "\n"
    controls += function_source(old, "SpeedMode_UpdateControl").replace(
        "SpeedMode_UpdateControl", "old_speed_control"
    )
    vector = json.loads(
        (load_profile.ROOT / "tests/vectors/speed_feedforward_v1.json").read_text(encoding="utf-8")
    )
    checks = r"""
static void feedforward_checks(void) {
    SpeedFeedforwardConfig config;
    SpeedFeedforwardState state, previous;
    InterfaceParam_TypeDef p, saved, bad;
    MotorControl_TypeDef old_motor;
    PI_Controller_TypeDef pi, old_pi;
    const float dt=.0005f;
    SpeedFeedforward_Default(&config);
    assert(!config.enabled && SpeedFeedforward_IsValid(&config));
    config.enabled=true; config.gain_a_per_rad_s2=.2f;
    SpeedFeedforward_Reset(&state);
    float peak=0;
    for(unsigned tick=0;tick<40000;tick++) {
        float target=tick<10000?20.f:tick<20000?0.f:tick<30000?-20.f:0.f;
        previous=state;
        assert(SpeedFeedforward_Update(&state,&config,target,0,dt,6));
        assert(fabsf(state.acceleration_rad_s2)<=20.0001f);
        assert(fabsf(state.acceleration_rad_s2-previous.acceleration_rad_s2)<=.10001f);
        assert(fabsf(state.current_a)<=2.00001f);
        assert(isfinite(state.speed_rad_s));
        if(fabsf(state.current_a)>peak) peak=fabsf(state.current_a);
        if(tick%10000==9999) {
            assert(fabsf(state.speed_rad_s-target)<.001f);
            assert(fabsf(state.acceleration_rad_s2)<.001f);
        }
    }
    assert(peak>1.9f);
    for(unsigned tick=0;tick<2000;tick++) {
        float target=tick<1200?20.f:-20.f;
        previous=state;
        assert(SpeedFeedforward_Update(&state,&config,target,0,dt,.5f));
        assert(fabsf(state.current_a)<=.25001f);
        assert(fabsf(state.acceleration_rad_s2-previous.acceleration_rad_s2)<=.10001f);
    }
    previous=state;
    assert(!SpeedFeedforward_Update(&state,&config,NAN,0,dt,6));
    assert(!SpeedFeedforward_Update(&state,&config,0,0,.01f,6));
    assert(!SpeedFeedforward_Update(&state,&config,0,0,dt,0));
    assert(memcmp(&state,&previous,sizeof(state))==0);
    /* 关闭以及标定场景仍逐拍执行原PI/斜坡，不能引入前馈状态。 */
    seed(&p); MotorControl.speedAcc=.8f;MotorControl.speedDec=.5f;
    MotorControl.speed_ff_config.enabled=false;MotorControl.ModeNow=Speed_Mode;
    memset(&pi,0,sizeof(pi));old_pi=pi;old_motor=MotorControl;
    for(unsigned tick=0;tick<10000;tick++) {
        float feedback=sinf(tick*.01f)*.1f;
        MotorControl.speedRef=old_motor.speedRef=tick%2000<1000?.8f:-.7f;
        SpeedMode_UpdateControl(&MotorControl,&pi,feedback);
        old_speed_control(&old_motor,&old_pi,feedback);
        if(MotorControl.speedShadow!=old_motor.speedShadow || MotorControl.iqRef!=old_motor.iqRef) {
            printf("legacy mismatch tick=%u speed=%a/%a iq=%a/%a\n",tick,
                MotorControl.speedShadow,old_motor.speedShadow,MotorControl.iqRef,old_motor.iqRef);
            fflush(stdout);
        }
        assert(MotorControl.speedShadow==old_motor.speedShadow && MotorControl.iqRef==old_motor.iqRef);
        assert(memcmp(&pi,&old_pi,sizeof(pi))==0);
    }
    MotorControl.ModeNow=Calib_EncoderObserver;old_motor.ModeNow=Calib_EncoderObserver;
    MotorControl.speed_ff_config.enabled=true;
    SpeedMode_UpdateControl(&MotorControl,&pi,.1f);old_speed_control(&old_motor,&old_pi,.1f);
    assert(MotorControl.iqRef==old_motor.iqRef && memcmp(&pi,&old_pi,sizeof(pi))==0);
    seed(&p);MotorControl.ModeNow=Speed_Mode;MotorControl.speed_ff_config=config;
    MotorControl.speedRef=20;SpeedFeedforward_Reset(&MotorControl.speed_ff_state);
    memset(&pi,0,sizeof(pi));
    SpeedMode_UpdateControl(&MotorControl,&pi,0);
    for(unsigned tick=0;tick<10000;tick++) {
        SpeedFeedforwardState preview=MotorControl.speed_ff_state;
        assert(SpeedFeedforward_Update(&preview,&config,20,0,dt,6));
        float feedback=tick<1000?-100.f:preview.speed_rad_s;
        SpeedMode_UpdateControl(&MotorControl,&pi,feedback);
        assert(fabsf(MotorControl.iqRef)<=6.00001f && isfinite(pi.Ui));
        assert(fabsf(MotorControl.speed_ff_state.current_a)<=2.00001f);
        if(tick>8000) assert(fabsf(MotorControl.iqRef)<.01f);
    }
    /* 旧布局不移动；新记录独立CRC，缺省关闭且不额外触发保存。 */
    assert(sizeof(SpeedFeedforwardRecord)==32 && sizeof(InterfaceParam_TypeDef)==2384);
    assert(offsetof(InterfaceParam_TypeDef,friction_slew)==2292);
    assert(offsetof(InterfaceParam_TypeDef,speed_feedforward)==2312);
    assert(offsetof(InterfaceParam_TypeDef,position_speed_pi)==2272);
    seed(&p);
    const uint8_t golden0[]={GOLDEN0};
    assert(memcmp(&p.speed_feedforward,golden0,32)==0);
    for(unsigned erased=0;erased<2;erased++) {
        saved=p;memset(&saved.speed_feedforward,erased?255:0,32);
        assert(!Param_Download(&saved) && !MotorControl.speed_ff_config.enabled);
        assert(MotorControl.axis_profile_valid && MotorControl.speed_Kp==p.speed_kp);
    }
    MotorControl.ModeNow=Motor_Disable;ModeLast=Motor_Disable;MotorControl.ErrorNow=No_Error;
    gate_off=true;pending_load_profile=UINT32_MAX;
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_ENABLE,1);
    assert(reply_id==CAN_GET_SPEED_FF_ENABLE && reply_value==1 && MotorControl.speed_ff_config.enabled);
    Param_Upload(&saved);saved.magic_word=MAGIC_WORD;
    const uint8_t golden1[]={GOLDEN1};
    assert(memcmp(&saved.speed_feedforward,golden1,32)==0);
    assert(memcmp(&saved,&p,2292)==0);
    assert(memcmp(&saved.friction_slew,&p.friction_slew,sizeof(FrictionSlewRecord))==0);
    assert(!Param_Download(&saved) && MotorControl.speed_ff_config.enabled);
    assert(OnBoard_Encoder.electrical_zero_q15==12480 && OnBoard_Encoder.linearization_lut_q15[71]==-42);
    for(unsigned bit=0;bit<256;bit++) {
        bad=saved;((uint8_t*)&bad.speed_feedforward)[bit/8]^=(uint8_t)(1U<<(bit%8));
        assert(!Param_Download(&bad) && !MotorControl.axis_profile_valid);
    }
    bad=saved;bad.speed_feedforward.enabled=2;
    bad.speed_feedforward.crc32=speed_feedforward_crc(&bad.speed_feedforward);
    assert(!Param_Download(&bad) && !MotorControl.axis_profile_valid);
    bad=saved;bad.speed_feedforward.gain_a_per_rad_s2=NAN;
    bad.speed_feedforward.crc32=speed_feedforward_crc(&bad.speed_feedforward);
    assert(!Param_Download(&bad) && !MotorControl.axis_profile_valid);
    assert(!Param_Download(&saved));
    bad=saved;bad.friction_slew.magic=PARAM_FRICTION_SLEW_MAGIC;
    bad.friction_slew.version=1;bad.friction_slew.attack_a_per_s=35;bad.friction_slew.release_a_per_s=300;
    bad.friction_slew.crc32=friction_slew_crc(&bad.friction_slew);
    assert(!Param_Download(&bad) && MotorControl.axis_profile_valid);
    assert(MotorControl.friction_attack_slew_a_per_s==35 && MotorControl.friction_release_slew_a_per_s==300);
    Param_Upload(&p);p.magic_word=MAGIC_WORD;
    assert(memcmp(&p.friction_slew,&bad.friction_slew,sizeof(FrictionSlewRecord))==0);
    assert(p.speed_feedforward.enabled==1);
    bad.friction_slew.crc32^=1;
    assert(!Param_Download(&bad) && !MotorControl.axis_profile_valid);
    assert(!Param_Download(&saved));
    MotorControl.ModeNow=Motor_Disable;ModeLast=Motor_Disable;gate_off=true;
    const float invalid[]={NAN,INFINITY,-1,2,.5f};
    for(unsigned i=0;i<5;i++) {
        CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_ENABLE,invalid[i]);
        assert(reply_value==-1 && MotorControl.speed_ff_config.enabled);
    }
    for(unsigned reason=0;reason<5;reason++) {
        MotorControl.ModeNow=reason==0?Speed_Mode:Motor_Disable;
        ModeLast=reason==1?Speed_Mode:Motor_Disable;
        MotorControl.ErrorNow=reason==2?Over_Current:No_Error;
        gate_off=reason!=3;pending_load_profile=reason==4?0:UINT32_MAX;
        CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_ENABLE,0);
        assert(reply_value==-1 && MotorControl.speed_ff_config.enabled);
    }
    MotorControl.ModeNow=Motor_Disable;ModeLast=Motor_Disable;MotorControl.ErrorNow=No_Error;
    gate_off=true;pending_load_profile=UINT32_MAX;
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_GAIN,.05f);assert(reply_value==.05f);
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_ACCELERATION,15);assert(reply_value==15);
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_JERK,150);assert(reply_value==150);
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_CURRENT_LIMIT,1);assert(reply_value==1);
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_GAIN,.3f);assert(reply_value==-1);
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_ACCELERATION,0);assert(reply_value==-1);
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_JERK,3000);assert(reply_value==-1);
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_CURRENT_LIMIT,11);assert(reply_value==-1);
    MotorControl.speed_ff_state.current_a=-.5f;MotorControl.speed_ff_state.acceleration_rad_s2=-10;
    CAN_ReceiveMessage_Update(CAN_GET_SPEED_FF_CURRENT,NAN);assert(reply_value==-.5f);
    CAN_ReceiveMessage_Update(CAN_GET_SPEED_FF_ACCELERATION_PLANNED,NAN);assert(reply_value==-10);
    CAN_ReceiveMessage_Update(CAN_SET_SPEED_FF_ENABLE,0);
    assert(reply_value==0 && !MotorControl.speed_ff_config.enabled && !MotorControl.speed_ff_state.initialized);
    puts("PASS SFF1 CRC/layout/legacy, bounded planner/FF/total current, legacy arithmetic and actual CAN stop guards");
}
"""
    for enabled in (0, 1):
        checks = checks.replace(
            f"GOLDEN{enabled}",
            ",".join(f"0x{b:02x}" for b in bytes.fromhex(vector["vectors"][enabled]["hex"])),
        )
    return original_fixture().replace(
        "int main(void) {", controls + checks + "\nint main(void) {\n    feedforward_checks();"
    )


if __name__ == "__main__":
    load_profile.fixture = fixture
    load_profile.main()
