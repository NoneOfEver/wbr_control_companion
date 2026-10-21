"""Test the fault worker and result checker; never claim MCU hardware validation."""
import importlib.util
import json
import os
import pathlib
import signal
import subprocess
import sys
import time

binary, backend = sys.argv[1:]
path = pathlib.Path(__file__).parents[1]/'measurements/fault_recovery/run.py'
spec = importlib.util.spec_from_file_location('fault_runner',path)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
environment = dict(os.environ,LD_PRELOAD=backend,WBR_MOCK_FAULT_DIAG='1',
                   WBR_MOCK_FULL_LOAD='1',WBR_MOCK_REQUIRE_PREPOST='1')
query = subprocess.run([binary,'--no-rt','--query'],env=environment,capture_output=True,text=True,timeout=5)
assert query.returncode == 0, query.stderr
assert "USB fault trace (host monotonic ns):" in query.stderr
assert "event_calls=" in query.stderr and "link_error=[" in query.stderr
assert json.loads(query.stdout)['snapshot']['last_stop_submit_us'] == 120100
missing = dict(environment)
del missing['WBR_MOCK_FAULT_DIAG']
result = subprocess.run([binary,'--no-rt','--query'],env=missing,capture_output=True,text=True,timeout=5)
assert result.returncode == 1 and 'FLT1 diagnostics unavailable' in result.stderr
for mode in ('terminate','freeze'):
    child = subprocess.Popen([binary,'--no-rt'],env=environment,stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE,text=True)
    try:
        assert runner.read_ready(child)['event'] == 'ready'
        time.sleep(.04)
        if mode == 'freeze':
            child.send_signal(signal.SIGSTOP)
            time.sleep(.12)
            child.send_signal(signal.SIGCONT)
        else:
            child.terminate()
        stdout,stderr=child.communicate(timeout=3)
        assert child.returncode == (2 if mode == 'freeze' else 0), (stdout,stderr)
    finally:
        runner.cleanup(child)
before=dict(accepted=100,rejected=0,lease_stops=5,explicit_stops=3,
            last_accepted_us=0,last_stop_submit_us=0,stop_flags=0,prior_session=9)
after=dict(before,accepted=600,lease_stops=6,last_accepted_us=0xfffffff0,
           last_stop_submit_us=(0xfffffff0+20100)&0xffffffff,stop_flags=2,prior_session=0)
assert all(runner.assess('kill',before,after,-signal.SIGKILL).values())
assert all(runner.assess('freeze',before,after,2).values())
for change in (dict(lease_stops=5),dict(prior_session=9),dict(stop_flags=0x102),
               dict(last_stop_submit_us=1)):
    assert not all(runner.assess('kill',before,dict(after,**change),-signal.SIGKILL).values())
graceful=dict(before,accepted=600,explicit_stops=4,stop_flags=1,prior_session=0)
assert all(runner.assess('terminate',before,graceful,0).values())
print('FLT1 query, missing-image refusal, stop-only worker signals, counter/timing/wrap result checks passed')
