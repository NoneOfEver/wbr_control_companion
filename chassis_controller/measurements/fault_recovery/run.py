#!/usr/bin/env python3
"""Stop-only host-process faults; no physical USB unplug or motor enable."""
import argparse
import json
import pathlib
import selectors
import signal
import subprocess
import time


def delta(after, before, field):
    return (after[field]-before[field]) & 0xffffffff


def assess(mode, before, after, returncode):
    checks = {
        'commands_were_accepted': delta(after, before, 'accepted') > 0,
        'session_was_invalidated': after['prior_session'] == 0,
        'stop_submission_no_error': not (after['stop_flags'] & 0x100),
    }
    reason = after['stop_flags'] & 255
    if mode == 'terminate':
        checks.update(explicit_stop=delta(after,before,'explicit_stops') == 1,
                      no_watchdog_stop=delta(after,before,'lease_stops') == 0,
                      reason_explicit=reason == 1, child_clean_exit=returncode == 0)
    else:
        checks.update(watchdog_stop=delta(after,before,'lease_stops') == 1,
                      no_valid_explicit_stop=delta(after,before,'explicit_stops') == 0,
                      reason_automatic=reason in (2,3))
        if reason == 2:
            interval=(after['last_stop_submit_us']-after['last_accepted_us']) & 0xffffffff
            checks['lease_to_stop_submission_20_to_30ms'] = 20000 <= interval <= 30000
        checks['expected_child_exit'] = returncode == -signal.SIGKILL if mode == 'kill' else returncode in (1,2)
    return checks


def read_ready(child, timeout=4):
    selector=selectors.DefaultSelector()
    selector.register(child.stdout,selectors.EVENT_READ)
    try:
        if not selector.select(timeout):
            raise RuntimeError('Timed out waiting for FLT1 worker handshake')
        line=child.stdout.readline()
        if not line:
            raise RuntimeError('Worker exited before handshake: '+child.stderr.read())
        event=json.loads(line)
        if event.get('event') != 'ready':
            raise RuntimeError('Unexpected worker event: '+line)
        return event
    finally:
        selector.close()


def cleanup(child):
    if child.poll() is None:
        child.send_signal(signal.SIGCONT)
        child.terminate()
        try:
            child.wait(timeout=2)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=2)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker',required=True,type=pathlib.Path)
    parser.add_argument('--output',required=True,type=pathlib.Path)
    parser.add_argument('--no-rt',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    command=[str(args.worker.resolve())]+(['--no-rt'] if args.no_rt else [])
    results=[]
    for mode in ('terminate','kill','freeze'):
        child=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            ready=read_ready(child)
            time.sleep(.5)  # Establish a live command lease before the fault.
            if child.poll() is not None:
                raise RuntimeError('Worker exited before fault: '+child.stderr.read())
            if mode == 'freeze':
                child.send_signal(signal.SIGSTOP)
                time.sleep(.12)  # Six lease intervals; stop only this owned PID.
                child.send_signal(signal.SIGCONT)
            elif mode == 'kill':
                child.kill()
            else:
                child.terminate()
            stdout,stderr=child.communicate(timeout=3)
            time.sleep(.12)  # Give the independent MCU watchdog time before querying.
            query=subprocess.run(command+['--query'],capture_output=True,text=True,timeout=5)
            if query.returncode:
                raise RuntimeError('Post-fault query failed: '+query.stderr)
            after=json.loads(query.stdout.splitlines()[0])['snapshot']
            before=ready['snapshot']
            checks=assess(mode,before,after,child.returncode)
            record=dict(mode=mode,before=before,after=after,checks=checks,
                        child_returncode=child.returncode,child_stderr=stderr,child_stdout=stdout,
                        accepted_delta=delta(after,before,'accepted'),
                        stop_interval_us=(after['last_stop_submit_us']-after['last_accepted_us']) & 0xffffffff,
                        passed=all(checks.values()))
            results.append(record)
            (args.output/'report.json').write_text(json.dumps(results,indent=2))
            print(mode, 'PASS' if record['passed'] else 'FAIL', json.dumps(checks),flush=True)
            if not record['passed']:
                return 1
        except Exception as error:
            (args.output/'error.txt').write_text(str(error)+'\n')
            print(mode,'ERROR',str(error),flush=True)
            return 1
        finally:
            cleanup(child)
    print('PASS: MCU stop SUBMISSION and session invalidation observed; motor stop/CAN completion UNMEASURED')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
