#!/usr/bin/env python3
"""Offline timing evidence; MCU observation bursts are not UART frame boundaries."""
import argparse
import collections
import csv
import json
import math


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return dict(n=len(values), p50=values[math.ceil(len(values)*.5)-1],
                p99=values[math.ceil(len(values)*.99)-1], max=values[-1])


def analyze(path, burst_gap_us=200):
    ports = collections.defaultdict(list)
    with open(path, newline='') as source:
        for row in csv.DictReader(source):
            row = {key: int(value) for key, value in row.items()}
            ports[row['port']].append(row)
    result = {}
    for port, rows in sorted(ports.items()):
        groups, group, observation_gaps, arrival_gaps = [], [], [], []
        previous = None
        for row in rows:
            if previous:
                gap = (row['mcu_observed_us'] - previous['mcu_observed_us']) & 0xffffffff
                observation_gaps.append(gap)
                arrival_gaps.append((row['host_arrival_ns']-previous['host_arrival_ns'])/1000)
                if gap > burst_gap_us:
                    groups.append(group)
                    group = []
            group.append(row)
            previous = row
        if group:
            groups.append(group)
        patterns = collections.Counter(tuple(row['bytes'] for row in group) for group in groups)
        largest = sorted(enumerate(rows), key=lambda item: item[1]['observation_to_submit_us'], reverse=True)[:5]
        outliers = []
        for index, row in largest:
            before = rows[index-1] if index else None
            after = rows[index+1] if index+1 < len(rows) else None
            outliers.append(dict(sequence=row['stream_sequence'], queue_us=row['observation_to_submit_us'],
                                 host_arrival_ns=row['host_arrival_ns'],
                                 arrival_since_first_ms=(row['host_arrival_ns']-rows[0]['host_arrival_ns'])/1e6,
                                 previous_host_gap_us=(row['host_arrival_ns']-before['host_arrival_ns'])/1000 if before else None,
                                 next_host_gap_us=(after['host_arrival_ns']-row['host_arrival_ns'])/1000 if after else None))
        result[str(port)] = dict(chunks=len(rows), bytes=sum(row['bytes'] for row in rows),
                                host_capture_span_s=(rows[-1]['host_arrival_ns']-rows[0]['host_arrival_ns'])/1e9,
                                queue_after_first_second_us=distribution([
                                    row['observation_to_submit_us'] for row in rows
                                    if row['host_arrival_ns']-rows[0]['host_arrival_ns'] >= 1000000000]),
                                queue_after_first_10ms_us=distribution([
                                    row['observation_to_submit_us'] for row in rows
                                    if row['host_arrival_ns']-rows[0]['host_arrival_ns'] >= 10000000]),
                                over_1ms_count=sum(row['observation_to_submit_us'] > 1000 for row in rows),
                                last_over_1ms_since_first_ms=max((
                                    (row['host_arrival_ns']-rows[0]['host_arrival_ns'])/1e6 for row in rows
                                    if row['observation_to_submit_us'] > 1000), default=None),
                                observation_gap_us=distribution(observation_gaps),
                                host_gap_us=distribution(arrival_gaps),
                                observation_bursts=len(groups), burst_gap_threshold_us=burst_gap_us,
                                common_burst_patterns=[dict(chunk_bytes=list(pattern), total_bytes=sum(pattern), count=count)
                                                       for pattern, count in patterns.most_common(8)],
                                largest_queue_samples=outliers)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    parser.add_argument('--burst-gap-us', type=int, default=200)
    args = parser.parse_args()
    if args.burst_gap_us < 0:
        parser.error('burst gap must be nonnegative')
    print(json.dumps(analyze(args.csv, args.burst_gap_us), indent=2))
