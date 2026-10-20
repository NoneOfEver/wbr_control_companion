"""Compare migrated balance math against the original MCU sources, offline."""
import math
import subprocess
import sys

reference, migrated = sys.argv[1:]
def samples(binary):
    return [float(value) for value in subprocess.check_output([binary], text=True).split()]
old, new = samples(reference), samples(migrated)
assert len(old) == len(new) == 1200
assert all(math.isfinite(value) for value in old + new)
maximum = max(abs(a-b) for a,b in zip(old,new))
assert all(math.isclose(a,b,rel_tol=1e-12,abs_tol=1e-12) for a,b in zip(old,new)), maximum
print(f"Original MCU vs migrated balance: {len(old)} outputs over 2000 ticks, max difference={maximum}")
