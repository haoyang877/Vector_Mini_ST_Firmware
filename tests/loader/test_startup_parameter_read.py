"""编译真实参数读写，验证固定缓冲互斥、ABI及升级版启动不迁移写Flash。"""

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
static InterfaceParam_TypeDef storage, upload_image, downloaded;
#define PARAM_FLASH_ADDR ((uintptr_t)&storage)
#define MAGIC_WORD 0x12345678U
static bool migrate, erase_fails;
static unsigned erases, writes, uploads, downloads, primask, inject_stage, reentries;
void flash_read_param(void);
void flash_write_param(void);

static void inject_reentry(unsigned stage) {
    if (inject_stage!=stage) return;
    inject_stage=0;
    InterfaceParam_TypeDef before=parameter_buffer;
    unsigned old_erases=erases, old_writes=writes, old_uploads=uploads, old_downloads=downloads;
    unsigned old_mask=primask;
    ++reentries;
    flash_read_param();
    flash_write_param();
    assert(erases==old_erases && writes==old_writes && uploads==old_uploads && downloads==old_downloads);
    assert(memcmp(&before,&parameter_buffer,sizeof(before))==0 && primask==old_mask);
}
static unsigned __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask=1; }
static void __enable_irq(void) { primask=0; inject_reentry(5); }
static bool Param_Download(const InterfaceParam_TypeDef *p) {
    assert(p==&parameter_buffer && memcmp(p,&storage,sizeof(storage))==0);
    ++downloads; downloaded=*p; inject_reentry(1); return migrate;
}
static void Param_Upload(InterfaceParam_TypeDef *p) {
    assert(p==&parameter_buffer); ++uploads; *p=upload_image; inject_reentry(2);
}
static bool flash_erase_pages(uintptr_t a, uintptr_t b) {
    assert(a==(uintptr_t)&storage && b==a+sizeof(storage)-1);
    ++erases; inject_reentry(3);
    if (erase_fails) return false;
    memset(&storage,255,sizeof(storage)); return true;
}
static void flash_write_data(uintptr_t a, const void *p, unsigned n) {
    assert(a==(uintptr_t)&storage && n==sizeof(storage) && p==&parameter_buffer);
    ++writes; inject_reentry(4); memcpy(&storage,p,n);
}
"""

CASES = r"""
static void reset_case(unsigned mask) {
    assert(!parameter_buffer_busy);
    memset(&storage,0xA5,sizeof(storage));
    memset(&upload_image,0x3C,sizeof(upload_image));
    upload_image.magic_word=MAGIC_WORD;
    migrate=erase_fails=false;
    erases=writes=uploads=downloads=inject_stage=reentries=0;
    primask=mask;
}
int main(void) {
    assert(sizeof(InterfaceParam_TypeDef)==2384);
    assert(offsetof(InterfaceParam_TypeDef,motor_thermal)==2344);
    assert(sizeof(parameter_buffer)==sizeof(InterfaceParam_TypeDef));
    for (unsigned mask=0; mask<=1; ++mask) {
        reset_case(mask);
        InterfaceParam_TypeDef original=storage;
        flash_read_param();
        assert(downloads==1 && erases==0 && writes==0 && !parameter_buffer_busy && primask==mask);
        assert(memcmp(&storage,&original,sizeof(storage))==0);
        migrate=true;
        flash_read_param();
#ifdef APP_LOADER_LAYOUT
        assert(downloads==2 && uploads==0 && erases==0 && writes==0);
        assert(memcmp(&storage,&original,sizeof(storage))==0);
#else
        assert(uploads==1 && erases==1 && writes==1);
        assert(memcmp(&storage,&upload_image,sizeof(storage))==0);
#endif
        assert(!parameter_buffer_busy && primask==mask);
        /* 获取后到发布期间，读写交叉重入不能覆盖快照；原中断屏蔽状态保持。 */
        for (unsigned stage=1; stage<=5; ++stage) {
            if (stage==5 && mask!=0) continue;
            reset_case(mask); inject_stage=stage;
            if (stage==1) flash_read_param(); else flash_write_param();
            assert(reentries==1 && !parameter_buffer_busy && primask==mask);
            assert(stage==1 ? downloads==1 : uploads==1 && writes==1 && erases==1);
        }
        /* 擦除失败仍释放所有权，下一次保存可继续；不改变现有失败返回契约。 */
        reset_case(mask); original=storage; erase_fails=true;
        flash_write_param();
        assert(!parameter_buffer_busy && primask==mask && erases==1 && writes==0);
        assert(memcmp(&storage,&original,sizeof(storage))==0);
        erase_fails=false;
        flash_write_param();
        assert(!parameter_buffer_busy && primask==mask && erases==2 && writes==1);
        assert(memcmp(&storage,&upload_image,sizeof(storage))==0);
    }
    /* 整条2384字节记录反复交替读写，包含LUT和各扩展记录，不残留上一份数据。 */
    reset_case(1);
    for (unsigned trial=0; trial<100; ++trial) {
        memset(&upload_image,(int)trial,sizeof(upload_image));
        upload_image.magic_word=MAGIC_WORD;
        flash_write_param(); flash_read_param();
        assert(memcmp(&downloaded,&upload_image,sizeof(downloaded))==0);
        assert(!parameter_buffer_busy && primask==1);
    }
#ifdef APP_LOADER_LAYOUT
    puts("PASS loader: fixed 2384-byte buffer, reentry exclusion, IRQ state, unchanged startup Flash");
#else
    puts("PASS legacy: fixed 2384-byte buffer, reentry exclusion, IRQ state, migration preserved");
#endif
    return 0;
}
"""


def main():
    """复用真实结构和函数，假存储覆盖两布局；不加载任何硬件库。"""
    compiler, out = sys.argv[1], Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "firmware/platform/stm32g4/bsp/flash.c").read_text(encoding="utf-8")
    for name in ("heap.c", "heap.h"):
        if (ROOT / "firmware/common" / name).exists():
            raise RuntimeError("APP私有分配器应随最后调用者一起移除")
    for name in ("Vector_Mini_ST", "Vector_Mini_ST_HIL"):
        project = ET.parse(ROOT / "firmware/platform/stm32g4/cubemx/MDK-ARM" / (name + ".uvprojx"))
        if project.findall(".//File[FileName='heap.c']"):
            raise RuntimeError("Keil工程仍链接已删除的私有分配器")
    if '"heap.h"' in (ROOT / "firmware/app/common_inc.h").read_text(encoding="utf-8"):
        raise RuntimeError("聚合头仍引用已删除的私有分配器")
    parameter = (ROOT / "firmware/services/parameters/foc_param.h").read_text(encoding="utf-8")
    parameter = parameter[parameter.index("typedef struct") : parameter.index("void Param_Return_Default")]
    lut_size = re.search(r"^#define\s+ENCODER_OFFSET_LUT_SIZE\s+(\d+)U",
                        (ROOT / "firmware/platform/stm32g4/bsp/encoder.h").read_text(encoding="utf-8"), re.M)[1]
    declarations = source[source.index("static InterfaceParam_TypeDef parameter_buffer;") : source.index("/**")]
    fixture = out / "startup_parameter_read.c"
    fixture.write_text(
        "#include <assert.h>\n#include <stdbool.h>\n#include <stdint.h>\n"
        "#include <stddef.h>\n#include <stdio.h>\n#include <string.h>\n"
        '#include "motor_axis_profile.h"\n#include "motor_load_profile.h"\n#include "motor_thermal.h"\n'
        + "#define ENCODER_OFFSET_LUT_SIZE " + lut_size + "U\n"
        + parameter + declarations + PRELUDE
        + "\n".join(function_source(source, name) for name in
                    ("parameter_buffer_acquire", "flash_read_param", "flash_write_param"))
        + CASES,
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
