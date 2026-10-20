"""Exercise real full-load host with test-only USB sources and strict output checks."""
import csv
import os
import pathlib
import subprocess
import sys
import tempfile

binary, backend = sys.argv[1:]
with tempfile.TemporaryDirectory() as directory:
    for mode in (None, 'WBR_MOCK_FULL_DROP_IMU', 'WBR_MOCK_FULL_FAULT'):
        output = pathlib.Path(directory) / (mode or 'healthy')
        environment = dict(os.environ, LD_PRELOAD=backend, WBR_MOCK_FULL_LOAD='1',
                           WBR_MOCK_REQUIRE_PREPOST='1')
        if mode:
            environment[mode] = '1'
        result = subprocess.run([binary, '--no-rt', '--seconds', '0.12', '--output', str(output)],
                                env=environment, capture_output=True, text=True, timeout=8)
        assert result.returncode == (1 if mode else 0), (mode, result.stdout, result.stderr)
        report = (output / 'summary.txt').read_text()
        rows = list(csv.DictReader((output / 'received.csv').open()))
        if mode is None:
            expected = {'left_wheel','right_wheel','left_joint_b','left_joint_d',
                        'right_joint_b','right_joint_d','SBUS','HI91','onboard_IMU'}
            assert {row['channel'] for row in rows} == expected
            assert 'stream_coverage=PASS' in report and 'RESULT=HOST_OBSERVED_PASS' in report
            assert 'CAN TX completion/coalescing/errors: UNMEASURED' in report
            assert 'line_idle=1 idle_error=0 offset_error=0' in report
            timing = list(csv.DictReader((output / 'cycles.csv').open()))
            assert f'USB128_completed={len(timing)} failed_or_cancelled=0' in report
            assert len(timing) and all(int(row['decoder_valid_mask']) <= 511 for row in timing)
        else:
            assert 'stream_coverage=FAIL' in report and 'RESULT=FAIL' in report
print('Six-frame stop-only output, all stream coverage, missing IMU, motor faults, export and drain passed')
