"""Offline grouping uses MCU intervals, including uint32 microsecond wrap."""
import importlib.util
import csv
import pathlib
import tempfile

path = pathlib.Path(__file__).parents[1] / 'tools/analyze_uart_capture.py'
spec = importlib.util.spec_from_file_location('uart_analysis', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
with tempfile.TemporaryDirectory() as directory:
    source = pathlib.Path(directory) / 'capture.csv'
    keys = ['port', 'stream_sequence', 'bytes', 'host_arrival_ns',
            'mcu_observed_us', 'observation_to_submit_us']
    with source.open('w', newline='') as output:
        writer = csv.DictWriter(output, fieldnames=keys)
        writer.writeheader()
        for group in range(2):
            for chunk, size in enumerate([40, 40, 40, 8]):
                writer.writerow(dict(port=1, stream_sequence=group*4+chunk,
                                     bytes=size, host_arrival_ns=group*2000000000+chunk*1000,
                                     mcu_observed_us=(0xfffffffe+group*2000000+chunk)&0xffffffff,
                                     observation_to_submit_us=7000 if group == 0 else 20))
    result = module.analyze(source)['1']
    assert result['observation_bursts'] == 2, result
    assert result['common_burst_patterns'] == [dict(chunk_bytes=[40,40,40,8], total_bytes=128, count=2)]
    assert result['observation_gap_us']['p50'] == 1
    assert result['queue_after_first_second_us']['max'] == 20
    assert result['largest_queue_samples'][0]['queue_us'] == 7000
print('MCU burst grouping, timestamp wrap and startup/steady queue distinction passed')
