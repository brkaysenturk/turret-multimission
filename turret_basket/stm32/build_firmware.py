import subprocess
import sys
from pathlib import Path

GCC_BIN = Path(r"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin")
CC = str(GCC_BIN / "arm-none-eabi-gcc.exe")
OBJCOPY = str(GCC_BIN / "arm-none-eabi-objcopy.exe")
SIZE = str(GCC_BIN / "arm-none-eabi-size.exe")

REPO_ROOT = Path(__file__).resolve().parents[2]
BASKET_DIR = REPO_ROOT / "turret_basket" / "stm32"
FCS_STM_DIR = REPO_ROOT / "turret_fcs" / "stm32"
BUILD_DIR = BASKET_DIR / "build"
BUILD_DIR.mkdir(parents=True, exist_ok=True)

INCLUDES = [
    f"-I{BASKET_DIR / 'Core' / 'Inc'}",
    f"-I{FCS_STM_DIR / 'Core' / 'Inc'}",
    f"-I{FCS_STM_DIR / 'Drivers' / 'STM32F4xx_HAL_Driver' / 'Inc'}",
    f"-I{FCS_STM_DIR / 'Drivers' / 'STM32F4xx_HAL_Driver' / 'Inc' / 'Legacy'}",
    f"-I{FCS_STM_DIR / 'Drivers' / 'CMSIS' / 'Device' / 'ST' / 'STM32F4xx' / 'Include'}",
    f"-I{FCS_STM_DIR / 'Drivers' / 'CMSIS' / 'Include'}",
]

CFLAGS = [
    "-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-mthumb",
    "-DSTM32F401xE", "-DUSE_HAL_DRIVER",
    "-O2", "-Wall", "-fdata-sections", "-ffunction-sections",
]

SOURCES = [
    # Basket custom sources
    BASKET_DIR / "Core" / "Src" / "main.c",
    BASKET_DIR / "Core" / "Src" / "basket_comm.c",
    BASKET_DIR / "Core" / "Src" / "flywheel_solenoid.c",
    BASKET_DIR / "Core" / "Src" / "luna_read.c",
    BASKET_DIR / "Core" / "Src" / "servo_driver.c",
    BASKET_DIR / "Core" / "Src" / "stm32f4xx_it.c",
    # Shared system sources
    FCS_STM_DIR / "Core" / "Src" / "stm32f4xx_hal_msp.c",
    FCS_STM_DIR / "Core" / "Src" / "system_stm32f4xx.c",
    FCS_STM_DIR / "Core" / "Src" / "sysmem.c",
    FCS_STM_DIR / "Core" / "Src" / "syscalls.c",
    # Startup asm
    FCS_STM_DIR / "Core" / "Startup" / "startup_stm32f401retx.s",
]

# HAL driver sources
HAL_SRC_DIR = FCS_STM_DIR / "Drivers" / "STM32F4xx_HAL_Driver" / "Src"
for hal_file in HAL_SRC_DIR.glob("*.c"):
    SOURCES.append(hal_file)

OBJECTS = []
print(f"Compiling {len(SOURCES)} source files...")
for src in SOURCES:
    obj = BUILD_DIR / (src.stem + ".o")
    OBJECTS.append(obj)
    cmd = [CC] + CFLAGS + INCLUDES + ["-c", str(src), "-o", str(obj)]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print(f"Error compiling {src.name}:")
        print(res.stderr)
        sys.exit(1)

LDSCRIPT = str(FCS_STM_DIR / "STM32F401RETX_FLASH.ld")
ELF_FILE = BUILD_DIR / "turret_basket.elf"
MAP_FILE = BUILD_DIR / "turret_basket.map"
BIN_FILE = BASKET_DIR / "turret_basket.bin"
HEX_FILE = BASKET_DIR / "turret_basket.hex"

LDFLAGS = [
    "-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-mthumb",
    f"-T{LDSCRIPT}",
    f"-Wl,-Map={MAP_FILE},--cref",
    "-Wl,--gc-sections",
    "-static",
    "--specs=nano.specs",
    "-lc", "-lm", "-lnosys"
]

print("Linking ELF...")
link_cmd = [CC] + [str(o) for o in OBJECTS] + LDFLAGS + ["-o", str(ELF_FILE)]
res = subprocess.run(link_cmd, capture_output=True, text=True)
if res.returncode != 0:
    print("Linker error:")
    print(res.stderr)
    sys.exit(1)

print("Generating BIN and HEX...")
subprocess.run([OBJCOPY, "-O", "binary", str(ELF_FILE), str(BIN_FILE)], check=True)
subprocess.run([OBJCOPY, "-O", "ihex", str(ELF_FILE), str(HEX_FILE)], check=True)

print("\nFirmware Size:")
subprocess.run([SIZE, str(ELF_FILE)])

print("\nSUCCESS! Created:")
print(f"  BIN: {BIN_FILE}")
print(f"  HEX: {HEX_FILE}")
