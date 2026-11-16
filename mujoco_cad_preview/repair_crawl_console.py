"""Disable QuickEdit in an EXISTING crawl_night console without restarting it.

Run from a SECOND PowerShell window using the existing project venv.
Only a short-lived helper attaches to the target console; it changes one
console input mode. It never terminates/restarts the optimizer, changes its
stdin/stdout, sends keys, writes a STOP file or changes models/checkpoints.

This mitigates classic-console QuickEdit selection freezes. It is not a
proof that every apparent hang has that cause. Already-disabled QuickEdit
is explicitly reported rather than presented as a successful diagnosis.
"""
from __future__ import annotations
import argparse
import ctypes
from ctypes import wintypes as W
from datetime import datetime
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
QUICK_EDIT = 0x0040
EXTENDED_FLAGS = 0x0080


def no_quick_edit(mode: int) -> int:
    return (mode | EXTENDED_FLAGS) & ~QUICK_EDIT


def save(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def apply_to_console(pid: int, report: Path) -> int:
    """Called in an isolated helper with NO parent console and NO console output."""
    result = {'target_pid': pid, 'status': 'ERROR', 'optimizer_restarted': False}
    handle = None
    attached = False
    k = None
    try:
        if os.name != 'nt':
            raise RuntimeError('Windows only.')
        k = ctypes.WinDLL('kernel32', use_last_error=True)
        k.AttachConsole.argtypes = [W.DWORD]; k.AttachConsole.restype = W.BOOL
        k.FreeConsole.argtypes = []; k.FreeConsole.restype = W.BOOL
        k.CreateFileW.argtypes = [W.LPCWSTR, W.DWORD, W.DWORD, ctypes.c_void_p,
                                  W.DWORD, W.DWORD, W.HANDLE]
        k.CreateFileW.restype = W.HANDLE
        k.GetConsoleMode.argtypes = [W.HANDLE, ctypes.POINTER(W.DWORD)]
        k.GetConsoleMode.restype = W.BOOL
        k.SetConsoleMode.argtypes = [W.HANDLE, W.DWORD]; k.SetConsoleMode.restype = W.BOOL
        k.CloseHandle.argtypes = [W.HANDLE]; k.CloseHandle.restype = W.BOOL
    except Exception as exc:
        result['error'] = str(exc)
        save(report, result)
        return 1

    class Coord(ctypes.Structure):
        _fields_ = [('x', ctypes.c_short), ('y', ctypes.c_short)]
    class Rect(ctypes.Structure):
        _fields_ = [('left', ctypes.c_short), ('top', ctypes.c_short),
                    ('right', ctypes.c_short), ('bottom', ctypes.c_short)]
    class Selection(ctypes.Structure):
        _fields_ = [('flags', W.DWORD), ('anchor', Coord), ('rectangle', Rect)]
    try:
        k.GetConsoleSelectionInfo.argtypes = [ctypes.POINTER(Selection)]
        k.GetConsoleSelectionInfo.restype = W.BOOL
        k.FreeConsole()  # affects ONLY this disposable helper, not the optimizer
        if not k.AttachConsole(pid):
            raise ctypes.WinError(ctypes.get_last_error())
        attached = True
        # Do not use stdin: the helper's stdin is intentionally NUL.
        handle = k.CreateFileW('CONIN$', 0xC0000000, 0x00000003, None, 3, 0, None)
        if handle is None or handle == ctypes.c_void_p(-1).value:
            raise ctypes.WinError(ctypes.get_last_error())
        before = W.DWORD(); after = W.DWORD()
        if not k.GetConsoleMode(handle, ctypes.byref(before)):
            raise ctypes.WinError(ctypes.get_last_error())
        selection = Selection()
        if k.GetConsoleSelectionInfo(ctypes.byref(selection)):
            result['selection_flags_before'] = int(selection.flags)
        if not k.SetConsoleMode(handle, no_quick_edit(before.value)):
            raise ctypes.WinError(ctypes.get_last_error())
        if not k.GetConsoleMode(handle, ctypes.byref(after)):
            raise ctypes.WinError(ctypes.get_last_error())
        if after.value & QUICK_EDIT:
            raise RuntimeError('QuickEdit flag is still enabled after the API call.')
        result.update(status='QUICK_EDIT_DISABLED' if before.value & QUICK_EDIT else 'ALREADY_DISABLED',
                      quick_edit_before=bool(before.value & QUICK_EDIT),
                      quick_edit_after=bool(after.value & QUICK_EDIT),
                      mode_before=int(before.value), mode_after=int(after.value))
        if k.GetConsoleSelectionInfo(ctypes.byref(selection)):
            result['selection_flags_after'] = int(selection.flags)
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if handle not in (None, ctypes.c_void_p(-1).value):
            k.CloseHandle(handle)
        if attached:
            k.FreeConsole()
        # No prints while attached: target-console output may itself be paused.
        save(report, result)
    return 0 if result['status'] != 'ERROR' else 1


def find_target() -> tuple[int, Path, dict]:
    latest = ROOT / 'output/crawl_night/LATEST.txt'
    run = Path(latest.read_text(encoding='utf-8-sig').strip())
    match = re.fullmatch(r'\d{8}_\d{6}_(\d+)', run.name)
    if not match or not (run / 'manifest.json').is_file():
        raise RuntimeError('Cannot identify the recorded crawl_night process from LATEST.txt.')
    pid = int(match[1])
    # Check the live process identity. Refuse stale/reused/non-optimizer PIDs.
    command = (f"$p = Get-CimInstance Win32_Process -Filter 'ProcessId = {pid}'; "
               "if ($null -eq $p) { exit 3 }; "
               "$p | Select-Object ProcessId,Name,CommandLine | ConvertTo-Json -Compress")
    completed = subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-Command', command],
                               stdin=subprocess.DEVNULL, capture_output=True, timeout=15,
                               creationflags=subprocess.CREATE_NO_WINDOW)
    if completed.returncode != 0:
        raise RuntimeError(f'Recorded optimizer PID {pid} is not running or cannot be inspected. No new run was started.')
    identity = json.loads(completed.stdout.decode('utf-8-sig', errors='replace').strip())
    cmd = identity.get('CommandLine') or ''
    if not re.search(r'\bcrawl_night\.py\b', cmd, re.I) or '--view-best' in cmd:
        raise RuntimeError(f'PID {pid} is not the expected crawl_night optimizer. Nothing changed.')
    return pid, run, identity


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--worker', type=int, help=argparse.SUPPRESS)
    p.add_argument('--report', type=Path, help=argparse.SUPPRESS)
    a = p.parse_args()
    if a.worker is not None:
        if a.report is None:
            return 2
        return apply_to_console(a.worker, a.report)
    if os.name != 'nt':
        raise RuntimeError('This repair is only for Windows console sessions.')
    pid, run, identity = find_target()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    report = ROOT / 'output/console_repair' / f'{stamp}_{pid}.json'
    print('EXISTING OPTIMIZER PID:', pid, flush=True)
    print('RUN:', run, flush=True)
    print('No restart. No optimizer/model/result files are changed.', flush=True)
    helper = [sys.executable, str(Path(__file__).resolve()), '--worker', str(pid), '--report', str(report)]
    try:
        child = subprocess.run(helper, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, creationflags=subprocess.CREATE_NO_WINDOW,
                               timeout=15)
    except subprocess.TimeoutExpired:
        # subprocess.run only terminates our helper, NEVER the optimizer.
        raise RuntimeError('Console API timed out. Press Esc ONCE in the original window, then rerun this repair. Optimizer was not terminated.')
    if not report.is_file():
        raise RuntimeError(f'Repair helper exited {child.returncode} without a report. Optimizer was not restarted.')
    result = json.loads(report.read_text('utf-8'))
    print('STATUS:', result['status'])
    print('QUICK_EDIT_BEFORE:', result.get('quick_edit_before'))
    print('QUICK_EDIT_AFTER:', result.get('quick_edit_after'))
    print('REPORT:', report)
    if result['status'] == 'ERROR':
        raise RuntimeError(result.get('error', 'Unknown console API error'))
    if result.get('selection_flags_after', 0):
        print('Exit the EXISTING selection once with Esc in the original window.')
    if result['status'] == 'ALREADY_DISABLED':
        print('QuickEdit was already OFF. This does not establish the cause of the pause.')
    else:
        print('QuickEdit is now OFF in the optimizer console. Existing optimizer continues.')
    print('Keep the ORIGINAL optimizer window open; this repair window may be closed.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print('CONSOLE REPAIR FAILED:', exc, file=sys.stderr)
        raise SystemExit(1)
