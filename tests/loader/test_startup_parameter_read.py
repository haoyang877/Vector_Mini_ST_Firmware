"""编译真实参数读写及前台保存协调函数，用假Flash逐点验证保存失败路径。"""

from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests/unit/native"))
sys.path.insert(0, str(ROOT / "tools"))
from run_position_servo_tests import function_source  # noqa: E402
from project_paths import NATIVE_INCLUDE_FLAGS  # noqa: E402

PRELUDE = r"""
static union { InterfaceParam_TypeDef record; uint8_t bytes[4096]; } storage;
static InterfaceParam_TypeDef upload_image, downloaded;
#define PARAM_FLASH_ADDR ((uintptr_t)&storage)
#define FLASH_PAGE_SIZE 2048U
#define FLASH_BASE (PARAM_FLASH_ADDR-56U*FLASH_PAGE_SIZE)
#define MAGIC_WORD 0x12345678U
#define HAL_OK 0
#define HAL_ERROR 1
#define FLASH_BANK_1 1U
#define FLASH_TYPEERASE_PAGES 2U
#define FLASH_TYPEPROGRAM_DOUBLEWORD 3U
#define FLASH_FLAG_OPTVERR 4U
#define __HAL_FLASH_CLEAR_FLAG(flag) assert((flag)==FLASH_FLAG_OPTVERR && primask==1)
typedef struct { uint32_t Banks, TypeErase, Page, NbPages; } FLASH_EraseInitTypeDef;
enum { Motor_Disable, Save_Param, Speed_Mode };
enum { No_Error, MotorParam_Error };
static unsigned ModeLast, mode_now, error_now, stops;
static bool migrate, outputs_disabled, unlock_fails, lock_fails, corrupt_readback;
static unsigned fail_erase, fail_program, erases, programs, unlocks, locks;
static unsigned uploads, downloads, primask, inject_stage, reentries;
void flash_read_param(void);
FlashParamSaveResult flash_write_param(void);

static void inject_reentry(unsigned stage) {
    if (inject_stage!=stage) return;
    inject_stage=0;
    InterfaceParam_TypeDef before=parameter_buffer;
    unsigned old_erases=erases, old_programs=programs, old_uploads=uploads, old_downloads=downloads;
    unsigned old_mask=primask;
    ++reentries;
    flash_read_param();
    assert(flash_write_param()==FLASH_PARAM_SAVE_BUSY);
    assert(erases==old_erases && programs==old_programs && uploads==old_uploads && downloads==old_downloads);
    assert(memcmp(&before,&parameter_buffer,sizeof(before))==0 && primask==old_mask);
}
static unsigned __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) {
    primask=1;
    if (inject_stage==6) { inject_stage=0; mode_now=Motor_Disable; }
}
static void __enable_irq(void) { primask=0; inject_reentry(5); }
static bool motor_hw_phase_outputs_disabled(void) { assert(primask==1); return outputs_disabled; }
static unsigned Get_ModeNow(void) { return mode_now; }
static void Set_ModeNow(unsigned mode) { assert(primask==1); mode_now=mode; }
static void Set_ErrorNow(unsigned error) { assert(primask==1); error_now=error; }
static void Stop_PWM_Generate(void) { assert(primask==1); ++stops; outputs_disabled=true; }
static bool Param_Download(const InterfaceParam_TypeDef *p) {
    assert(p==&parameter_buffer && memcmp(p,&storage.record,sizeof(*p))==0);
    ++downloads; downloaded=*p; inject_reentry(1); return migrate;
}
static void Param_Upload(InterfaceParam_TypeDef *p) {
    assert(p==&parameter_buffer); ++uploads; *p=upload_image; inject_reentry(2);
}
static int HAL_FLASH_Unlock(void) { assert(primask==1); ++unlocks; return unlock_fails ? HAL_ERROR : HAL_OK; }
static int HAL_FLASH_Lock(void) { assert(primask==1); ++locks; return lock_fails ? HAL_ERROR : HAL_OK; }
static int HAL_FLASHEx_Erase(const FLASH_EraseInitTypeDef *init, uint32_t *page_error) {
    assert(primask==1 && init->Banks==FLASH_BANK_1 && init->TypeErase==FLASH_TYPEERASE_PAGES);
    assert(init->NbPages==1 && init->Page==56U+erases);
    ++erases; inject_reentry(3);
    if (erases==fail_erase) { *page_error=init->Page; return HAL_ERROR; }
    memset(storage.bytes+(init->Page-56U)*FLASH_PAGE_SIZE,255,FLASH_PAGE_SIZE);
    return HAL_OK;
}
static int HAL_FLASH_Program(uint32_t type, uintptr_t address, uint64_t value) {
    assert(primask==1 && type==FLASH_TYPEPROGRAM_DOUBLEWORD && address==PARAM_FLASH_ADDR+programs*8U);
    assert(programs<sizeof(InterfaceParam_TypeDef)/8U);
    ++programs; inject_reentry(4);
    if (programs==fail_program) return HAL_ERROR;
    memcpy((void *)address,&value,8);
    if (corrupt_readback && programs==sizeof(InterfaceParam_TypeDef)/8U) storage.bytes[17]^=1;
    return HAL_OK;
}
"""

CASES = r"""
static void reset_case(unsigned mask) {
    assert(!parameter_buffer_busy);
    memset(&storage,0xA5,sizeof(storage));
    memset(&upload_image,0x3C,sizeof(upload_image));
    upload_image.magic_word=MAGIC_WORD;
    migrate=unlock_fails=lock_fails=corrupt_readback=false;
    outputs_disabled=true;
    erases=programs=unlocks=locks=uploads=downloads=inject_stage=reentries=0;
    fail_erase=fail_program=stops=0;
    ModeLast=mode_now=Motor_Disable; error_now=No_Error;
    primask=mask; last_save_result=FLASH_PARAM_SAVE_NOT_ATTEMPTED;
}
static void check_released(unsigned mask, FlashParamSaveResult result) {
    assert(!parameter_buffer_busy && primask==mask);
    assert(flash_get_last_save_result()==result);
}
static void check_full_record(void) {
    assert(memcmp(&storage.record,&upload_image,sizeof(upload_image))==0);
    for (unsigned i=sizeof(upload_image); i<sizeof(storage); ++i) assert(storage.bytes[i]==255);
}
int main(void) {
    assert(sizeof(InterfaceParam_TypeDef)==2384);
    assert(offsetof(InterfaceParam_TypeDef,motor_thermal)==2344);
    assert(sizeof(parameter_buffer)==sizeof(InterfaceParam_TypeDef));
    for (unsigned mask=0; mask<=1; ++mask) {
        reset_case(mask);
        InterfaceParam_TypeDef original=storage.record;
        flash_read_param();
        assert(downloads==1 && erases==0 && programs==0);
        assert(memcmp(&storage.record,&original,sizeof(original))==0);
        check_released(mask,FLASH_PARAM_SAVE_NOT_ATTEMPTED);
        migrate=true;
        flash_read_param();
#ifdef APP_LOADER_LAYOUT
        assert(downloads==2 && uploads==0 && erases==0 && programs==0);
        assert(memcmp(&storage.record,&original,sizeof(original))==0);
        check_released(mask,FLASH_PARAM_SAVE_NOT_ATTEMPTED);
#else
        assert(uploads==1 && erases==2 && programs==298 && locks==1);
        check_full_record(); check_released(mask,FLASH_PARAM_SAVE_WRITTEN);
        flash_read_param();
        assert(erases==2 && programs==298 && unlocks==1);
        check_released(mask,FLASH_PARAM_SAVE_UNCHANGED);
        reset_case(mask); migrate=true; fail_program=14;
        flash_read_param();
        assert(programs==14 && locks==1);
        check_released(mask,FLASH_PARAM_SAVE_PROGRAM_FAILED);
#endif
        /* 交叉重入不能覆盖独占快照；写入全程屏蔽IRQ，启动读仍保持短获取临界区。 */
        for (unsigned stage=1; stage<=5; ++stage) {
            if (stage==5 && mask!=0) continue;
            reset_case(mask); inject_stage=stage;
            if (stage==1 || stage==5) flash_read_param();
            else assert(flash_write_param()==FLASH_PARAM_SAVE_WRITTEN);
            assert(reentries==1 && !parameter_buffer_busy && primask==mask);
            assert(stage==1 || stage==5 ? downloads==1 : uploads==1 && programs==298 && erases==2);
        }
        /* 忙/关相拒绝不上传、不擦写；忙时绝不能释放另一调用持有的缓冲。 */
        reset_case(mask); parameter_buffer_busy=true;
        assert(flash_write_param()==FLASH_PARAM_SAVE_BUSY && parameter_buffer_busy && primask==mask);
        flash_read_param(); assert(downloads==0 && uploads==0 && erases==0);
        parameter_buffer_busy=false;
        outputs_disabled=false;
        assert(flash_write_param()==FLASH_PARAM_SAVE_OUTPUTS_ACTIVE && uploads==0 && unlocks==0);
        check_released(mask,FLASH_PARAM_SAVE_OUTPUTS_ACTIVE);
        outputs_disabled=true;
        assert(flash_write_param()==FLASH_PARAM_SAVE_WRITTEN);
        check_released(mask,FLASH_PARAM_SAVE_WRITTEN); check_full_record();
        assert(flash_write_param()==FLASH_PARAM_SAVE_UNCHANGED);
        assert(unlocks==1 && erases==2 && programs==298 && locks==1);
        check_released(mask,FLASH_PARAM_SAVE_UNCHANGED);
        /* 两个页的任一失败均不编程；298个双字的任一失败均立即停止。 */
        for (unsigned page=1; page<=2; ++page) {
            reset_case(mask); fail_erase=page;
            assert(flash_write_param()==FLASH_PARAM_SAVE_ERASE_FAILED);
            assert(erases==page && programs==0 && locks==1);
            check_released(mask,FLASH_PARAM_SAVE_ERASE_FAILED);
        }
        for (unsigned word=1; word<=298; ++word) {
            reset_case(mask); fail_program=word;
            assert(flash_write_param()==FLASH_PARAM_SAVE_PROGRAM_FAILED);
            assert(erases==2 && programs==word && locks==1);
            check_released(mask,FLASH_PARAM_SAVE_PROGRAM_FAILED);
        }
        reset_case(mask); unlock_fails=true;
        assert(flash_write_param()==FLASH_PARAM_SAVE_UNLOCK_FAILED && erases==0 && programs==0 && locks==1);
        check_released(mask,FLASH_PARAM_SAVE_UNLOCK_FAILED);
        reset_case(mask); lock_fails=true;
        assert(flash_write_param()==FLASH_PARAM_SAVE_LOCK_FAILED && locks==1);
        check_released(mask,FLASH_PARAM_SAVE_LOCK_FAILED);
        reset_case(mask); corrupt_readback=true;
        assert(flash_write_param()==FLASH_PARAM_SAVE_VERIFY_FAILED && programs==298 && locks==1);
        check_released(mask,FLASH_PARAM_SAVE_VERIFY_FAILED);
        /* 失败后所有权已释放；下一次显式保存可重试。 */
        corrupt_readback=false; erases=programs=unlocks=locks=0;
        assert(flash_write_param()==FLASH_PARAM_SAVE_WRITTEN);
        check_released(mask,FLASH_PARAM_SAVE_WRITTEN); check_full_record();
        /* 前台必须等待快环SAVE确认；刚收到请求、运行模式或抢占撤销均不保存。 */
        reset_case(mask); mode_now=Save_Param;
        Main_SaveParametersPoll();
        assert(mode_now==Save_Param && uploads==0 && error_now==No_Error && primask==mask);
        ModeLast=Save_Param; mode_now=Speed_Mode;
        Main_SaveParametersPoll(); assert(mode_now==Speed_Mode && uploads==0 && primask==mask);
        mode_now=Save_Param; inject_stage=6;
        Main_SaveParametersPoll(); assert(mode_now==Motor_Disable && uploads==0 && primask==mask);
        mode_now=Save_Param;
        Main_SaveParametersPoll();
        assert(mode_now==Motor_Disable && error_now==No_Error && programs==298 && stops==0);
        check_released(mask,FLASH_PARAM_SAVE_WRITTEN);
        mode_now=Save_Param; Main_SaveParametersPoll();
        assert(mode_now==Motor_Disable && error_now==No_Error && programs==298);
        check_released(mask,FLASH_PARAM_SAVE_UNCHANGED);
        /* 各种失败统一发布既有参数故障并停机，不把保存失败当成功。 */
        for (unsigned failure=0; failure<7; ++failure) {
            reset_case(mask); mode_now=ModeLast=Save_Param;
            if (failure==0) parameter_buffer_busy=true;
            if (failure==1) outputs_disabled=false;
            if (failure==2) unlock_fails=true;
            if (failure==3) fail_erase=2;
            if (failure==4) fail_program=67;
            if (failure==5) corrupt_readback=true;
            if (failure==6) lock_fails=true;
            Main_SaveParametersPoll();
            assert(mode_now==Motor_Disable && error_now==MotorParam_Error && stops==1 && outputs_disabled);
            assert(primask==mask && (failure==0 ? parameter_buffer_busy : !parameter_buffer_busy));
            parameter_buffer_busy=false;
        }
    }
    /* 整条记录反复交替读写，包含LUT和扩展记录，不残留上一份数据。 */
    reset_case(1);
    for (unsigned trial=0; trial<100; ++trial) {
        memset(&upload_image,(int)trial,sizeof(upload_image)); upload_image.magic_word=MAGIC_WORD;
        erases=programs=unlocks=locks=0;
        assert(flash_write_param()==FLASH_PARAM_SAVE_WRITTEN); flash_read_param();
        assert(memcmp(&downloaded,&upload_image,sizeof(downloaded))==0);
        check_released(1,FLASH_PARAM_SAVE_WRITTEN);
    }
#ifdef APP_LOADER_LAYOUT
    puts("PASS loader: no startup write; 2384-byte ABI; stop gate, dedup, all 2-page/298-word failures, verify, IRQ/ownership");
#else
    puts("PASS legacy: migration preserved; 2384-byte ABI; stop gate, dedup, all 2-page/298-word failures, verify, IRQ/ownership");
#endif
    return 0;
}
"""


def main():
    """复用真实结构/保存实现/主循环协调，假HAL不会访问任何硬件。"""
    compiler, out = sys.argv[1], Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "firmware/platform/stm32g4/bsp/flash.c").read_text(encoding="utf-8")
    header = (ROOT / "firmware/platform/stm32g4/bsp/flash.h").read_text(encoding="utf-8")
    main_source = (ROOT / "firmware/platform/stm32g4/cubemx/Core/Src/main.c").read_text(encoding="utf-8")
    for name in ("heap.c", "heap.h"):
        if (ROOT / "firmware/common" / name).exists():
            raise RuntimeError("APP私有分配器应随最后调用者一起移除")
    for name in ("Vector_Mini_ST", "Vector_Mini_ST_HIL"):
        project = ET.parse(ROOT / "firmware/platform/stm32g4/cubemx/MDK-ARM" / (name + ".uvprojx"))
        if project.findall(".//File[FileName='heap.c']"):
            raise RuntimeError("Keil工程仍链接已删除的私有分配器")
    if '"heap.h"' in (ROOT / "firmware/app/common_inc.h").read_text(encoding="utf-8"):
        raise RuntimeError("聚合头仍引用已删除的私有分配器")
    if main_source.count("Main_SaveParametersPoll();") != 1:
        raise RuntimeError("真实主循环必须调用已验证的保存协调函数一次")
    parameter = (ROOT / "firmware/services/parameters/foc_param.h").read_text(encoding="utf-8")
    parameter = parameter[parameter.index("typedef struct") : parameter.index("void Param_Return_Default")]
    lut_size = re.search(r"^#define\s+ENCODER_OFFSET_LUT_SIZE\s+(\d+)U",
                        (ROOT / "firmware/platform/stm32g4/bsp/encoder.h").read_text(encoding="utf-8"), re.M)[1]
    declarations = source[source.index("static InterfaceParam_TypeDef parameter_buffer;") : source.index("/**")]
    result_enum = header[header.index("typedef enum") : header.index("} FlashParamSaveResult;") + len("} FlashParamSaveResult;")]
    fixture = out / "startup_parameter_read.c"
    fixture.write_text(
        "#include <assert.h>\n#include <stdbool.h>\n#include <stdint.h>\n"
        "#include <stddef.h>\n#include <stdio.h>\n#include <string.h>\n"
        '#include "motor_axis_profile.h"\n#include "motor_load_profile.h"\n#include "motor_thermal.h"\n'
        + "#define ENCODER_OFFSET_LUT_SIZE " + lut_size + "U\n"
        + parameter + result_enum + declarations + PRELUDE
        + "\n".join(function_source(source, name) for name in
                    ("parameter_buffer_acquire", "flash_store_snapshot", "flash_read_param",
                     "flash_write_param", "flash_get_last_save_result"))
        + function_source(main_source, "Main_SaveParametersPoll") + CASES,
        encoding="utf-8",
    )
    command = [compiler] + (["cc"] if Path(compiler).stem == "zig" else [])
    for name, flags in (("legacy", []), ("loader", ["-DAPP_LOADER_LAYOUT=1"])):
        exe = out / f"startup_parameter_read_{name}.exe"
        subprocess.run(
            command + ["-std=c99", "-O1", "-Wall", "-Wextra", "-Werror", "-UNDEBUG",
                       "-Wno-unused-function", *flags, *NATIVE_INCLUDE_FLAGS, str(fixture), "-o", str(exe)],
            check=True,
        )
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
