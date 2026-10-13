"""编译真实启动读取函数，验证升级版启动不迁移写Flash。"""

from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests/unit/native"))
from run_position_servo_tests import function_source  # noqa: E402

PRELUDE = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef struct { uint32_t magic_word, value; } InterfaceParam_TypeDef;
static InterfaceParam_TypeDef storage, scratch;
#define PARAM_FLASH_ADDR ((uintptr_t)&storage)
#define MAGIC_WORD 0x12345678U
static bool migrate, allocation_fails;
static unsigned erases, writes, uploads, downloads, defaults, frees;
static void *HEAP_malloc(unsigned size) {
    assert(size==sizeof(scratch));
    return allocation_fails ? NULL : &scratch;
}
static void HEAP_free(void *p) { assert(p==&scratch); ++frees; }
static void Param_Return_Default(void) { ++defaults; }
static bool Param_Download(InterfaceParam_TypeDef *p) {
    assert(p->value==storage.value); ++downloads; return migrate;
}
static void Param_Upload(InterfaceParam_TypeDef *p) { ++uploads; p->value=99; }
static unsigned __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static bool flash_erase_pages(uintptr_t a, uintptr_t b) {
    assert(a==(uintptr_t)&storage && b==a+sizeof(storage)-1); ++erases;
    memset(&storage,255,sizeof(storage)); return true;
}
static void flash_write_data(uintptr_t a, const void *p, unsigned n) {
    assert(a==(uintptr_t)&storage && n==sizeof(storage)); ++writes;
    memcpy(&storage,p,n);
}
"""

CASES = r"""
int main(void) {
    InterfaceParam_TypeDef original = {7, 42};
    storage=original;
    flash_read_param();
    assert(downloads==1 && frees==1 && erases==0 && writes==0);
    assert(memcmp(&storage,&original,sizeof(storage))==0);
    migrate=true;
    flash_read_param();
#ifdef APP_LOADER_LAYOUT
    assert(downloads==2 && frees==2 && uploads==0 && erases==0 && writes==0);
    assert(memcmp(&storage,&original,sizeof(storage))==0);
    puts("PASS loader startup migration request: parameter bytes unchanged");
#else
    assert(uploads==1 && erases==1 && writes==1);
    assert(storage.magic_word==MAGIC_WORD && storage.value==99);
    puts("PASS legacy startup migration behavior preserved");
#endif
    allocation_fails=true;
    flash_read_param();
    assert(defaults==1 && downloads==2 && frees==2);
    return 0;
}
"""


def main():
    """假存储和I/O计数覆盖缺尾迁移与分配失败；不加载任何硬件库。"""
    compiler, out = sys.argv[1], Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    source = ROOT / "firmware/platform/stm32g4/bsp/flash.c"
    fixture = out / "startup_parameter_read.c"
    fixture.write_text(
        PRELUDE
        + function_source(source.read_text(encoding="utf-8"), "flash_read_param")
        + CASES,
        encoding="utf-8",
    )
    command = [compiler] + (["cc"] if Path(compiler).stem == "zig" else [])
    for name, flags in (("legacy", []), ("loader", ["-DAPP_LOADER_LAYOUT=1"])):
        exe = out / f"startup_parameter_read_{name}.exe"
        subprocess.run(
            command
            + [
                "-std=c99",
                "-O1",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-UNDEBUG",
                "-Wno-unused-function",
                *flags,
                str(fixture),
                "-o",
                str(exe),
            ],
            check=True,
        )
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
