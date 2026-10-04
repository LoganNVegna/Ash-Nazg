"""Run regression checks after `pio run -e upload_ota`; requires C++17 compiler."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile

project = Path(__file__).resolve().parents[1]
subprocess.run([sys.executable, str(project / 'tests/check_firmware.py')], check=True)
compiler = os.environ.get('CXX') or next(
    (p for name in ('cl', 'g++', 'clang++') if (p := shutil.which(name))), None)
if not compiler and os.name == 'nt' and os.environ.get('VCToolsInstallDir'):
    vc = Path(os.environ['VCToolsInstallDir']) / 'bin/Hostx64/x64/cl.exe'
    if vc.exists():
        compiler = str(vc)
if not compiler:
    sys.exit('C++ compiler missing. On Windows use a Visual Studio Developer Command Prompt.')
msvc = Path(compiler).name.lower() in ('cl', 'cl.exe')
if msvc:
    os.environ['PATH'] = str(Path(compiler).parent) + os.pathsep + os.environ.get('PATH', '')
suites = [
    ('control', ['tests/test_control.cpp'], ['include']),
    ('recovery', ['tests/test_recovery.cpp'], ['include']),
    ('driver', ['tests/generated/driver.cpp', 'tests/generated/test_driver.cpp'], ['tests/stubs', 'src', 'include', 'tests']),
    ('runtime', ['tests/generated/driver.cpp', 'tests/test_runtime.cpp'], ['tests/stubs', 'src', 'include', 'tests']),
    ('export', ['tests/test_csv_export.cpp'], ['include']),
]
with tempfile.TemporaryDirectory(prefix='ashnazg-tests-') as scratch:
    for name, sources, includes in suites:
        exe = Path(scratch) / (name + ('.exe' if os.name == 'nt' else ''))
        if msvc:
            args = [compiler, '/nologo', '/O2', '/EHsc', '/std:c++17', '/UNDEBUG']
            args += ['/I' + str(project / d) for d in includes]
            args += [str(project / s) for s in sources] + ['/Fe' + str(exe)]
        else:
            args = [compiler, '-O2', '-std=c++17', '-UNDEBUG']
            args += ['-I' + str(project / d) for d in includes]
            args += [str(project / s) for s in sources] + ['-o', str(exe)]
        subprocess.run(args, cwd=scratch, check=True)
        subprocess.run([str(exe)], check=True)
        if name=='runtime':subprocess.run([str(exe),'--no-sensor'],check=True)
print('PASS: competition image, control, recovery, actual runtime/transport/maintenance, and export suites.')
