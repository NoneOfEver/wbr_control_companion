#!/usr/bin/env python3
"""Manual physical USB unplug, with independent MCU power and stop-only output."""
import argparse
import json
import pathlib
import subprocess
import time
from run import cleanup, delta, read_ready

SYSFS = pathlib.Path('/sys/bus/usb/devices')


def is_gateway(path):
    try:
        return ((path/'idVendor').read_text().strip().lower() == '34b7' and
                (path/'idProduct').read_text().strip().lower() == 'ffff')
    except OSError:
        return False


def gateways(root=SYSFS):
    return [path for path in root.iterdir() if is_gateway(path)]


def device_number(path):
    try:
        return int((path/'devnum').read_text().strip())
    except (OSError, ValueError):
        return None


def assess_disconnect(before, after, returncode, stderr):
    reason = after['stop_flags'] & 255
    interval = (after['last_stop_submit_us']-after['last_accepted_us']) & 0xffffffff
    return {
        'commands_accepted_counters_preserved': 0 < delta(after,before,'accepted') <= 70000,
        'old_session_invalidated': after['prior_session'] == 0,
        'one_automatic_stop': delta(after,before,'lease_stops') == 1,
        'no_valid_explicit_stop': delta(after,before,'explicit_stops') == 0,
        'automatic_stop_reason': reason in (2,3),
        'stop_submission_no_error': not (after['stop_flags'] & 0x100),
        'stop_submission_interval': (20000 <= interval <= 30000 if reason == 2
                                     else 0 <= interval <= 30000 if reason == 3 else False),
        'host_reported_usb_failure': returncode == 1 and 'USB' in stderr,
    }


def query(command):
    result = subprocess.run(command+['--query'],capture_output=True,text=True,timeout=5)
    if result.returncode:
        raise RuntimeError('FLT1 query failed: '+result.stderr)
    return json.loads(result.stdout.splitlines()[0])['snapshot']


def run_case(command, directory):
    ports = gateways()
    if len(ports) != 1:
        raise RuntimeError('Expected exactly one HPM gateway, found '+str(len(ports)))
    port = ports[0]
    old_number = device_number(port)
    record = dict(port=port.name,old_device_number=old_number)
    def save():
        (directory/'report.json').write_text(json.dumps(record,indent=2))
    child = subprocess.Popen(command+['--seconds','60'],stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE,text=True)
    try:
        ready = read_ready(child)
        record.update(old_session=ready['session'],before=ready['snapshot'])
        save()
        time.sleep(.5)
        if child.poll() is not None:
            raise RuntimeError('Worker exited before unplug: '+child.stderr.read())
        print('阶段 1：请现在拔掉 NUC 与 MCU 的 USB 线，保持 MCU 外部供电。',flush=True)
        print('保持断开，收到下一条提示后再插回。',flush=True)
        deadline = time.monotonic()+45
        removed_ns = None
        early_exit = None
        while time.monotonic() < deadline:
            if not is_gateway(port) and removed_ns is None:
                removed_ns = time.monotonic_ns()
            if removed_ns is not None and child.poll() is not None:
                break
            if child.poll() is not None and removed_ns is None:
                if early_exit is None:
                    early_exit = time.monotonic()
                elif time.monotonic()-early_exit > 2:
                    raise RuntimeError('Worker exited without a verified USB removal')
            time.sleep(.02)
        else:
            raise RuntimeError('No USB removal/worker exit within 45 seconds')
        stdout,stderr = child.communicate(timeout=2)
        record.update(sysfs_removal_observed_ns=removed_ns,child_returncode=child.returncode,
                      child_stderr=stderr,child_stdout=stdout)
        save()
        print('阶段 2：设备已移除，旧进程已退出。请将 USB 插回原来的端口。',flush=True)
        deadline = time.monotonic()+90
        while time.monotonic() < deadline:
            if is_gateway(port):
                time.sleep(.3)
                if is_gateway(port):
                    break
            time.sleep(.05)
        else:
            raise RuntimeError('Gateway did not re-enumerate on the same port within 90 seconds')
        record['new_device_number'] = device_number(port)
        record['reenumerated_ns'] = time.monotonic_ns()
        save()
        print('阶段 3：已重新枚举；尚未打开新会话或恢复发送。',flush=True)
        input('按回车读取 MCU 停止记录，并验证一次新的停止态会话：')
        after = query(command)  # Snapshot precedes the querying Hello's stop/reset.
        checks = assess_disconnect(ready['snapshot'],after,child.returncode,stderr)
        checks['physical_removal_observed'] = removed_ns is not None
        checks['new_usb_device_number'] = (old_number is not None and
                                          record['new_device_number'] is not None and
                                          record['new_device_number'] != old_number)
        record.update(after_disconnect=after,disconnect_checks=checks)
        save()
        if not all(checks.values()):
            print('断开验证 FAIL',json.dumps(checks,ensure_ascii=False),flush=True)
            return 1
        print('断开验证 PASS',json.dumps(checks,ensure_ascii=False),flush=True)
        restarted = subprocess.Popen(command+['--seconds','.5'],stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE,text=True)
        try:
            fresh = read_ready(restarted)
            fresh_stdout,fresh_stderr = restarted.communicate(timeout=3)
            final = query(command)
            initial = fresh['snapshot']
            recovery = {
                'new_session_nonce': fresh['session'] != ready['session'],
                'no_session_before_explicit_reopen': initial['prior_session'] == 0,
                'new_commands_accepted': 0 < delta(final,initial,'accepted') <= 1000,
                'new_session_clean_exit': restarted.returncode == 0,
                'new_session_explicit_stop': delta(final,initial,'explicit_stops') == 1,
                'no_new_watchdog_stop': delta(final,initial,'lease_stops') == 0,
                'final_session_invalidated': final['prior_session'] == 0,
                'final_stop_no_error': final['stop_flags'] == 1,
            }
            record.update(recovery_before=initial,recovery_after=final,new_session=fresh['session'],
                          recovery_checks=recovery,recovery_stderr=fresh_stderr,
                          recovery_stdout=fresh_stdout,passed=all(recovery.values()))
            save()
            print('新停止态会话', 'PASS' if record['passed'] else 'FAIL',json.dumps(recovery),flush=True)
            print('仅验证停止帧提交和会话失效；CAN 完成、物理停机、算法重新使能尚未测量。')
            return 0 if record['passed'] else 1
        finally:
            cleanup(restarted)
    except Exception as error:
        record.update(passed=False,error=str(error))
        save()
        print('ERROR',str(error),flush=True)
        return 1
    finally:
        cleanup(child)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker',required=True,type=pathlib.Path)
    parser.add_argument('--output',required=True,type=pathlib.Path)
    parser.add_argument('--no-rt',action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    command = [str(args.worker.resolve())]+(['--no-rt'] if args.no_rt else [])
    try:
        return run_case(command,args.output)
    except Exception as error:
        (args.output/'report.json').write_text(json.dumps(
            {'passed': False, 'error': str(error)}, indent=2)+'\n')
        print('ERROR', str(error), flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
