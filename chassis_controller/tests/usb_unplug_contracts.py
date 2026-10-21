"""Check detach evidence without touching USB or requesting a physical unplug."""
import pathlib
import sys
import tempfile
sys.path.insert(0,str(pathlib.Path(__file__).parents[1]/'measurements/fault_recovery'))
import usb_unplug as test

before=dict(accepted=1000,rejected=0,lease_stops=2,explicit_stops=4,
            last_accepted_us=0,last_stop_submit_us=0,stop_flags=0,prior_session=10)
after=dict(before,accepted=1500,lease_stops=3,last_accepted_us=0xfffffff0,
           last_stop_submit_us=(0xfffffff0+21000)&0xffffffff,stop_flags=2,prior_session=0)
assert all(test.assess_disconnect(before,after,1,'USB session unhealthy').values())
immediate=dict(after,last_stop_submit_us=(0xfffffff0+200)&0xffffffff,stop_flags=3)
assert all(test.assess_disconnect(before,immediate,1,'USB disconnected').values())
for change in (dict(lease_stops=2),dict(prior_session=10),dict(stop_flags=0x102),
               dict(accepted=0),dict(explicit_stops=5),dict(last_stop_submit_us=5)):
    assert not all(test.assess_disconnect(before,dict(after,**change),1,'USB disconnected').values())
assert not all(test.assess_disconnect(before,after,0,'').values())
assert not all(test.assess_disconnect(before,after,2,'Command deadline expired').values())
with tempfile.TemporaryDirectory() as directory:
    root=pathlib.Path(directory)
    device=root/'3-4'
    device.mkdir()
    (device/'idVendor').write_text('34b7\n')
    (device/'idProduct').write_text('ffff\n')
    (device/'devnum').write_text('16\n')
    assert test.gateways(root) == [device]
    assert test.device_number(device) == 16
    (device/'idVendor').unlink()
    assert not test.is_gateway(device) and not test.gateways(root)
print('Detach prerequisites, reset rejection, MCU timing and host error evidence checks passed')
