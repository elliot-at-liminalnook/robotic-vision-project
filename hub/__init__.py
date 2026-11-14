"""The hub: everything that runs on the real machine (docs/MULTI_DEVICE_PLAN.md, 5.1).

It is started from ports and nothing else. It never imports the simulation (`sim/`): what it knows
is what came through a port (sensed) and what each kind of device is (built in). It is told nothing
about the place it is in.
"""
