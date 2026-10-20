"""Run the real capture and CSV exporter with a synthetic USB source, no motors."""
import csv
import os
import pathlib
import subprocess
import sys
import tempfile

binary, backend = sys.argv[1:]
with tempfile.TemporaryDirectory() as directory:
    output = pathlib.Path(directory) / 'uart.csv'
    result = subprocess.run(
        [binary, '--no-rt', '--probe-status', '--seconds', '0.12', '--csv', str(output)],
        env=dict(os.environ, LD_PRELOAD=backend, WBR_MOCK_UART='1'),
        capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, (result.stdout, result.stderr)
    rows = list(csv.DictReader(output.open()))
    assert {row['port'] for row in rows} == {'1', '2'}, rows
    assert all(int(row['observation_to_submit_us']) == 7 for row in rows)
    assert 'missing_chunks=1' in result.stdout, result.stdout
    assert 'capture_overflow=0' in result.stdout, result.stdout
    assert 'UART diagnostic before: image=URT3 probe=1 plic_enabled=1 irq_rx_available=0' in result.stdout
    assert 'UART diagnostic after: image=URT3 probe=1 plic_enabled=1' in result.stdout
    assert 'irq_timeout=1' in result.stdout
    assert 'last_total=128 last_offset=0 last_pending=103 last_added=25' in result.stdout
    assert 'Not measured:' in result.stdout
    ordered = subprocess.run(
        [binary, '--no-rt', '--wait-in-ready', '--seconds', '0.03', '--csv', str(output)],
        env=dict(os.environ, LD_PRELOAD=backend, WBR_MOCK_UART='1', WBR_MOCK_REQUIRE_PREPOST='1'),
        capture_output=True, text=True, timeout=8)
    assert ordered.returncode == 0, (ordered.stdout, ordered.stderr)
    assert 'USB startup wait_in_ready=1' in ordered.stdout
    failed = subprocess.run(
        [binary, '--no-rt', '--wait-in-ready', '--seconds', '0.03', '--csv', str(output)],
        env=dict(os.environ, LD_PRELOAD=backend, WBR_MOCK_IN_FAIL='1'),
        capture_output=True, text=True, timeout=8)
    assert failed.returncode == 1 and 'USB IN submit' in failed.stderr, failed.stderr
    # A working USB link with no UART stream must not report successful measurement.
    empty = subprocess.run(
        [binary, '--no-rt', '--seconds', '0.03', '--csv', str(output)],
        env=dict(os.environ, LD_PRELOAD=backend), capture_output=True, text=True, timeout=8)
    assert empty.returncode == 1 and 'no UART data' in empty.stderr, empty.stderr
print('UART capture, stream gaps, CSV timestamps, empty-data rejection and motor-free heartbeats passed')
