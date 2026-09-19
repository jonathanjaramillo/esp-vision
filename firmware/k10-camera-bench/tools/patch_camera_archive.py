"""Use a project-local esp32-camera archive with gc2145_init renamed.

The K10 framework ships esp32-camera only as a prebuilt archive. Renaming its
GC2145 entry point lets src/gc2145_wide.cpp wrap the real packaged driver and
replace only set_framesize, without modifying the shared PlatformIO package.
"""

Import("env")

import os
import shutil
import subprocess

framework = env.PioPlatform().get_package_dir("framework-arduinounihiker")
source_archive = os.path.join(framework, "tools", "sdk", "esp32s3", "lib",
                              "libesp32-camera.a")
patch_dir = os.path.join(env.subst("$BUILD_DIR"), "patched-camera")
archive = os.path.join(patch_dir, "libesp32-camera.a")
obj = os.path.join(patch_dir, "gc2145.c.obj")
toolchain = env.PioPlatform().get_package_dir("toolchain-xtensa-esp32s3")
prefix = os.path.join(toolchain, "bin", "xtensa-esp32s3-elf-")
ar = prefix + "ar"
objcopy = prefix + "objcopy"

os.makedirs(patch_dir, exist_ok=True)
shutil.copy2(source_archive, archive)
subprocess.run([ar, "x", archive, "gc2145.c.obj"], cwd=patch_dir, check=True)
subprocess.run([objcopy, "--redefine-sym",
                "gc2145_init=gc2145_original_init", obj], check=True)
subprocess.run([ar, "r", archive, obj], check=True)

# This path must precede the framework SDK library directory so its
# -lesp32-camera resolves to the project-local archive.
env.Prepend(LIBPATH=[patch_dir])
