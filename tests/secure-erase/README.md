Secure Erase Test
=================

This test verifies that secure erase works.

Synposis
--------

Flash `pass-1` first, which reboots Tomu.  Then flash `pass-2`.  Observe the red or green LED.

If the test passes, the green LED will light up.  If the test fails, the red LED will light up.

This test writes "secure" data at 0x4400, which should be erased.  It writes "insecure" data at 0x4800, which should not be erased.

pass-1
------

This pass stores "secure" data at offset 0x4400, and "insecure" data at offset 0x4800.  It indicates in its boot header that sector 17 (= 0x4400/1024) is secure, so that this sector will be erased when an update is loaded.

pass-2
------

This program simply checks offset 0x4400 and offset 0x4800.  If 0x4400 is anything but 0xff, then it turns on the red LED.  If 0x4800 doesn't match the expected values, then the red LED is turned on.  If both tests pass, then the green LED is turned on.
