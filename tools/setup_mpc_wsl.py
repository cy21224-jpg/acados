#!/usr/bin/env python3
"""Add MPC dependencies to an STM32F767 project without editing existing files.
Run from WSL: python3 setup_mpc_wsl.py /mnt/c/path/to/project [--build]
Supports this repository checkout and the standalone MPC_WSL_setup.zip bundle.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile

MANIFEST = '.mpc_import_manifest.json'
MODULES = ('MPC_param.h', 'MPC_function.h', 'MPC_param.c', 'MPC_function.c')


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def within(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def payload() -> dict[str, bytes]:
    here = Path(__file__).resolve().parent
    bundled = here / 'payload'
    if bundled.is_dir():
        return {p.relative_to(bundled).as_posix(): p.read_bytes()
                for p in bundled.rglob('*') if p.is_file()}
    repo = here.parent
    data: dict[str, bytes] = {}
    for name in MODULES:
        dest = 'Core/Inc/' if name.endswith('.h') else 'Core/Src/'
        data[dest + name] = (repo / name).read_bytes()
    with zipfile.ZipFile(repo / 'acadosdayo.zip') as z:
        for info in z.infolist():
            name = info.filename
            if info.is_dir():
                continue
            vendor = 'acadosdayo/acados_stm32_f767/include/'
            generated = 'acadosdayo/acados_model_generated/include/'
            sources = 'acadosdayo/Core/Src/acados_generated/'
            libraries = 'acadosdayo/acados_stm32_f767/lib/'
            if name.startswith(vendor):
                relative = name[len(vendor):]
                data['Core/Inc/' + relative] = z.read(info)
                if re.fullmatch(r'(blasfeo|hpipm)/include/[^/]+\.h', relative):
                    alias = Path(relative).name
                    forward = ('/* Added compatibility include; vendor headers are retained. */\n'
                               f'#include "{relative}"\n').encode()
                    if 'Core/Inc/' + alias in data and data['Core/Inc/' + alias] != forward:
                        raise ValueError(f'Header alias collision: {alias}')
                    data['Core/Inc/' + alias] = forward
            elif name.startswith(generated):
                data['Core/Inc/' + name[len(generated):]] = z.read(info)
            elif name.startswith(sources) and name.endswith('.c'):
                data['Core/Src/acados_generated/' + name[len(sources):]] = z.read(info)
            elif name.startswith(libraries) and name.endswith('.a'):
                data['MPC_lib/' + name[len(libraries):]] = z.read(info)
    return data


def safe_relative(name: str) -> str:
    if not re.fullmatch(r'[A-Za-z0-9_./+\-]+', name) or '..' in Path(name).parts or name.startswith('/'):
        raise ValueError(f'Unsupported relative build path: {name!r}')
    return name


def debug_profile(root: Path) -> tuple[list[str], list[str], str, str, str]:
    tree = ET.parse(root / '.cproject')
    configurations = tree.findall('.//storageModule[@moduleId="cdtBuildSystem"]/configuration')
    config = next((c for c in configurations if c.get('name') == 'Debug'), None)
    if config is None:
        raise ValueError('Debug configuration not found in .cproject; no files were changed.')
    options = config.findall('.//option')
    abi = next((o.get('value', '') for o in options if o.get('superClass', '').endswith('.floatabi')), '')
    fpu = next((o.get('value', '') for o in options if o.get('superClass', '').endswith('.fpu')), '')
    if not abi.endswith('.hard') or not fpu.endswith('.fpv5-d16'):
        raise ValueError('Existing FPU/ABI differs from the provided hard-float FPv5-D16 libraries; no settings were changed.')
    compiler = next(t for t in config.findall('.//tool')
                    if t.get('superClass', '').endswith('.tool.c.compiler'))
    defs: list[str] = []
    includes: list[str] = []
    opt, debug = '-O0', '-g3'
    for option in compiler.findall('option'):
        kind = option.get('superClass', '')
        values = [v.get('value', '') for v in option.findall('listOptionValue')]
        if kind.endswith('.definedsymbols'):
            for value in values:
                if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(=[A-Za-z0-9_]+)?', value):
                    raise ValueError(f'Unsupported compiler definition: {value!r}')
                defs.append(value)
        elif kind.endswith('.includepaths'):
            for value in values:
                value = value[3:] if value.startswith('../') else value
                includes.append(safe_relative(value))
        elif kind.endswith('.optimization.level'):
            suffix = option.get('value', '').rsplit('.', 1)[-1]
            opt = {'o0': '-O0', 'o1': '-O1', 'o2': '-O2', 'o3': '-O3',
                   'og': '-Og', 'os': '-Os'}.get(suffix.lower(), '-O0')
        elif kind.endswith('.debuglevel'):
            suffix = option.get('value', '').rsplit('.', 1)[-1]
            if suffix in ('g0', 'g1', 'g2', 'g3'):
                debug = '-' + suffix
    if 'STM32F767xx' not in defs or 'Core/Inc' not in includes:
        raise ValueError('Expected STM32F767xx and an existing Core/Inc include path.')
    ld_options = [o for o in config.findall('.//option')
                  if o.get('superClass', '').endswith('.c.linker.option.script')]
    if not ld_options:
        raise ValueError('Original linker script setting was not found.')
    linker = ld_options[0].get('value', '').rstrip('}"').rsplit('/', 1)[-1]
    safe_relative(linker)
    if not (root / linker).is_file():
        raise ValueError(f'Original linker script is missing: {linker}')
    return defs, includes, opt, debug, linker


def build_makefile(root: Path, data: dict[str, bytes]) -> bytes:
    defs, includes, opt, debug, linker = debug_profile(root)
    core_sources = {p.relative_to(root).as_posix() for p in (root / 'Core/Src').rglob('*.c')}
    core_sources.update(n for n in data if n.startswith('Core/Src/') and n.endswith('.c'))
    startup = {p.relative_to(root).as_posix() for p in (root / 'Core/Startup').glob('*.s')}
    driver_sources: set[str] = set()
    old_list = root / 'Debug/objects.list'
    if old_list.is_file():
        for line in old_list.read_text().splitlines():
            name = line.strip().strip('"')
            name = name[2:] if name.startswith('./') else name
            if name.startswith('Drivers/') and name.endswith('.o'):
                driver_sources.add(name[:-2] + '.c')
    else:
        driver_sources = {p.relative_to(root).as_posix()
                          for p in (root / 'Drivers/STM32F7xx_HAL_Driver/Src').glob('*.c')}
    for name in core_sources | driver_sources | startup:
        safe_relative(name)
        if name not in data and not (root / name).is_file():
            raise ValueError(f'Original source file missing: {name}')
    if not startup or not driver_sources:
        raise ValueError('Could not discover startup/driver sources.')
    name = safe_relative(root.name)
    source_lines = ' \\\n  '.join(sorted(core_sources | driver_sources))
    startup_lines = ' '.join(sorted(startup))
    text = f'''# Added WSL build. Original .cproject/.ioc/linker files are never rewritten.
# Debug compiler conditions copied from .cproject: {opt} {debug}, Cortex-M7 hard-float.
.DEFAULT_GOAL := all
MPC_CC ?= arm-none-eabi-gcc
MPC_SIZE ?= arm-none-eabi-size
MPC_NM ?= arm-none-eabi-nm
MPC_BUILD_DIR ?= MPC_build
MPC_OPT ?= {opt}
# Only needed for a custom relocated toolchain; normal WSL apt installs leave this empty.
MPC_EXTRA_FLAGS ?=
MPC_ARCH := -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb
MPC_INCLUDES := {' '.join('-I'+p for p in includes)}
MPC_DEFINES := {' '.join('-D'+p for p in defs)}
MPC_CFLAGS := $(MPC_ARCH) $(MPC_OPT) {debug} -std=gnu11 -Wall -ffunction-sections -fdata-sections --specs=nano.specs $(MPC_INCLUDES) $(MPC_DEFINES) $(MPC_EXTRA_FLAGS)
MPC_C_SOURCES := {source_lines}
MPC_S_SOURCES := {startup_lines}
MPC_OBJECTS := $(addprefix $(MPC_BUILD_DIR)/,$(MPC_C_SOURCES:.c=.o) $(MPC_S_SOURCES:.s=.o))
MPC_TARGET := $(MPC_BUILD_DIR)/{name}.elf
MPC_LINK_CHECK := $(MPC_BUILD_DIR)/MPC_link_check.elf
# The existing CubeIDE Makefiles already include this optional additive extension too.
include makefile.defs
MPC_LDFLAGS := $(MPC_ARCH) $(MPC_EXTRA_FLAGS) --specs=nosys.specs --specs=nano.specs -static -Wl,--gc-sections -T"{linker}" -u _printf_float -u _scanf_float
MPC_API_ROOTS := -Wl,-u,MPC_Init,-u,MPC_UpdateInitialState,-u,MPC_UpdateInputBounds,-u,MPC_Execute,-u,MPC_SetTickProvider,-u,MPC_DiscardState
.PHONY: all mpc-link-check
all: $(MPC_TARGET)
mpc-link-check: $(MPC_LINK_CHECK)
	@$(MPC_NM) "$(MPC_LINK_CHECK)" | awk '/ T MPC_(Init|Execute|UpdateInitialState|UpdateInputBounds)$$/'
$(MPC_TARGET): $(MPC_OBJECTS) {linker} makefile.defs MPC_build.mk
	$(MPC_CC) $(MPC_OBJECTS) $(LIBS) $(MPC_LDFLAGS) -Wl,-Map="$(MPC_BUILD_DIR)/application.map" -Wl,--start-group -lc -lm -Wl,--end-group -o "$@"
	$(MPC_SIZE) "$@"
$(MPC_LINK_CHECK): $(MPC_OBJECTS) {linker} makefile.defs MPC_build.mk
	$(MPC_CC) $(MPC_OBJECTS) $(LIBS) $(MPC_LDFLAGS) $(MPC_API_ROOTS) -Wl,-Map="$(MPC_BUILD_DIR)/MPC_link_check.map" -Wl,--start-group -lc -lm -Wl,--end-group -o "$@"
	$(MPC_SIZE) "$@"
$(MPC_OBJECTS): MPC_build.mk makefile.defs
$(MPC_BUILD_DIR)/%.o: %.c
	@mkdir -p "$(dir $@)"
	$(MPC_CC) $(MPC_CFLAGS) -MMD -MP -c "$<" -o "$@"
$(MPC_BUILD_DIR)/%.o: %.s
	@mkdir -p "$(dir $@)"
	$(MPC_CC) $(MPC_ARCH) {debug} $(MPC_DEFINES) $(MPC_EXTRA_FLAGS) --specs=nano.specs -x assembler-with-cpp -MMD -MP -c "$<" -o "$@"
-include $(MPC_OBJECTS:.o=.d)
'''
    return text.encode()


def install(root: Path, build: bool, jobs: int) -> None:
    if any(c in str(root) for c in ('$', '`', '"', '\\', '\n', '\r')):
        raise ValueError('Use a WSL path without shell expansion characters or backslashes.')
    if not (root / 'Core/Src/main.c').is_file():
        raise ValueError('Pass the STM32 project folder containing Core/Src/main.c.')
    data = payload()
    data['makefile.defs'] = b'''# Add MPC libraries; retain original compiler, linker script and IDE settings.
MPC_IMPORTED_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
LIBS += -Wl,--start-group "$(MPC_IMPORTED_ROOT)/MPC_lib/libacados.a" "$(MPC_IMPORTED_ROOT)/MPC_lib/libhpipm.a" "$(MPC_IMPORTED_ROOT)/MPC_lib/libblasfeo.a" -Wl,--end-group
'''
    data['MPC_build.mk'] = build_makefile(root, data)
    old = {}
    manifest = root / MANIFEST
    if manifest.exists() and not within(manifest.resolve(), root):
        raise ValueError('Import manifest escapes the project folder.')
    if (root / 'MPC_build').exists() and not within((root / 'MPC_build').resolve(), root):
        raise ValueError('Build output folder escapes the project folder.')
    if manifest.exists():
        old = json.loads(manifest.read_text()).get('files', {})
    if (root / 'MPC_build').exists() and not manifest.exists():
        raise ValueError('MPC_build already exists without an import manifest; refusing to reuse it.')
    conflicts = []
    newly_owned = set(old) | {name for name in data if not (root / name).exists()}
    for name, content in data.items():
        safe_relative(name)
        p = root / name
        if not within(p.resolve(), root):
            raise ValueError(f'Destination escapes project: {name}')
        if p.exists() and (not p.is_file() or (p.read_bytes() != content and sha(p.read_bytes()) != old.get(name))):
            conflicts.append(name)
    if conflicts:
        raise ValueError('Existing files differ; nothing was written:\n  ' + '\n  '.join(conflicts))
    originals = {p.relative_to(root).as_posix(): sha(p.read_bytes())
                 for p in root.rglob('*') if p.is_file()
                 and p.relative_to(root).as_posix() not in newly_owned
                 and p.name != MANIFEST
                 and 'MPC_build' not in p.relative_to(root).parts}
    written, reused = 0, 0
    for name, content in data.items():
        p = root / name
        if p.exists() and p.read_bytes() == content:
            reused += 1
            continue
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(content)
        written += 1
    saved = {'version': 1, 'files': {n: sha(b) for n, b in sorted(data.items()) if n in newly_owned}}
    encoded = json.dumps(saved, indent=2, sort_keys=True) + '\n'
    if not manifest.exists() or manifest.read_text() != encoded:
        manifest.write_text(encoded)
    if build:
        subprocess.run(['make', '-C', str(root), '-f', 'MPC_build.mk', f'-j{jobs}', 'all', 'mpc-link-check'], check=True)
    changed = [n for n, digest in originals.items() if sha((root / n).read_bytes()) != digest]
    if changed:
        raise RuntimeError('Original files changed unexpectedly: ' + ', '.join(changed))
    print(f'MPC import complete: wrote {written}, reused {reused}; {len(originals)} original files unchanged.')
    print('main.c, .cproject, .ioc and linker scripts were not edited. MPC is not yet called by the application.')
    print(f'Build: make -C "{root}" -f MPC_build.mk -j{jobs} all mpc-link-check')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project', type=Path)
    parser.add_argument('--build', action='store_true', help='Also compile and link using the WSL toolchain in PATH.')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    try:
        install(args.project.resolve(strict=True), args.build, args.jobs)
    except (OSError, ValueError, ET.ParseError, zipfile.BadZipFile, subprocess.CalledProcessError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
