"""The window on the hub's system (sim/scripts/system_window.py), on SDL's dummy display.

Aimed at classes of bugs:
  * a window that is only drawn between requests, so nothing can be watched as it happens;
  * a person's key acted on while a caller's command is running, or when the window was opened for watching only;
  * a key that does something the hub's API would have refused (keys go through the same API, with its protections);
  * the window changing what the hub is given (its pictures are the simulation's own, not the hub's cameras').
"""
import json

import numpy as np
import pytest

import system_window as W
from hub import api as A
from hub.hub import Hub
from so101sim.world import World


def call(hub, method, path, body=None):
    status, ctype, out = A.dispatch(hub, method, path, b"" if body is None else json.dumps(body).encode())
    return status, (json.loads(out) if ctype == "application/json" else out)


def press(window, key):
    window.pg.event.post(window.pg.event.Event(window.pg.KEYDOWN, key=key))


@pytest.fixture
def system():
    world = World("open_small")
    hub = Hub(world.ports)
    yield world, hub
    hub.shutdown()


def test_the_window_is_drawn_while_a_command_runs_and_shows_every_camera(system):
    world, hub = system
    window = W.Window(world, hub)
    frames = []
    world.on_frame.append(lambda: frames.append(world.time))
    t0 = world.time
    status, r = call(hub, "POST", "/devices/arm/move_tip", {"x": 0.22, "y": 0.05, "z": 0.10})
    assert status == 200 and r["reached"] and window.open
    took = world.time - t0
    assert len(frames) >= 10 and len(frames) == pytest.approx(took * W.FRAME_HZ, rel=0.15, abs=2), "one frame every 1/30 s of the world's time"
    cols, rows = window._shape
    assert len(window.cameras) == len(world.built.cameras) == 4 and cols * rows >= 4
    assert window.screen.get_size() == (cols * W.VIEW_W, rows * (W.VIEW_H + W.TITLE) + W.STATUS)
    shown = np.asarray(window.pg.surfarray.array3d(window.screen))
    for k in range(len(window.cameras)):                              # each view has a picture in it
        x, y = (k % cols) * W.VIEW_W, (k // cols) * (W.VIEW_H + W.TITLE) + W.TITLE
        assert shown[x:x + W.VIEW_W, y:y + W.VIEW_H].std() > 5, f"nothing drawn for {window.cameras[k]}"
    window.close()
    n = len(frames)
    assert window.frame not in world.on_frame and call(hub, "POST", "/wait", {"seconds": 0.2})[0] == 200 and len(frames) > n


def test_watching_changes_nothing_the_hub_is_given():
    """The same commands with and without a window give the same pictures to the hub and leave the world the same."""
    def run(watch):
        world = World("open_small")
        hub = Hub(world.ports)
        window = W.Window(world, hub) if watch else None
        call(hub, "POST", "/devices/arm/move_tip", {"x": 0.24, "y": -0.03, "z": 0.12})
        picture = bytes(call(hub, "GET", "/devices/camera_1/picture?width=320")[1])
        state = call(hub, "GET", "/devices/arm/state")[1]["joints"]["joints_rad"]
        if window:
            window.close()
        hub.shutdown()
        return picture, state, world.time
    assert run(True) == run(False)


@pytest.mark.parametrize("key,moves", [("K_w", (0.01, 0, 0)), ("K_s", (-0.01, 0, 0)), ("K_a", (0, 0.01, 0)), ("K_d", (0, -0.01, 0)),
                                       ("K_r", (0, 0, 0.01)), ("K_f", (0, 0, -0.01))])
def test_a_key_moves_the_tip_a_centimetre_the_way_it_says(system, key, moves):
    world, hub = system
    window = W.Window(world, hub, hands_on=True)
    call(hub, "POST", "/devices/arm/move_tip", {"x": 0.24, "y": 0.0, "z": 0.12})
    before = call(hub, "GET", "/devices/arm/state")[1]["tip_m"]
    press(window, getattr(window.pg, key))
    window.frame(idle=True)
    after = call(hub, "GET", "/devices/arm/state")[1]["tip_m"]
    went = np.array([after[k] - before[k] for k in "xyz"])
    assert np.linalg.norm(went - moves) < 0.006 and went @ np.array(moves) > 0.00004, (key, went)
    assert "POST /devices/arm/move_tip" in window.last_key and "-> 200" in window.last_key
    window.close()


def test_keys_drive_the_other_devices_too_and_only_through_the_hub(system):
    world, hub = system
    window = W.Window(world, hub, hands_on=True)
    pg = window.pg
    jaws = call(hub, "GET", "/devices/arm/state")[1]["gripper"]["target_opening"]
    press(window, pg.K_SPACE)
    window.frame(idle=True)
    assert abs(call(hub, "GET", "/devices/arm/state")[1]["gripper"]["target_opening"] - jaws) > 0.5, "Space did not move the jaws"
    heading = call(hub, "GET", "/devices/rover/drive/state")[1]["since_taken_over"]["heading_deg"]
    press(window, pg.K_LEFT)
    window.frame(idle=True)
    turned = call(hub, "GET", "/devices/rover/drive/state")[1]["since_taken_over"]["heading_deg"] - heading
    assert "/devices/rover/drive/turn" in window.last_key and (turned > 5.0 or "-> 409" in window.last_key), window.last_key
    # the head is stowed: the hub refuses to turn it, and the key's answer is the hub's own
    press(window, pg.K_COMMA)
    window.frame(idle=True)
    status, r = call(hub, "POST", "/devices/rover/head/point", {"pan_deg": 20.0})
    assert "/devices/rover/head/point" in window.last_key and f"-> {status}" in window.last_key
    assert status == 200 or r["error"][:40] in window.last_key
    # a key that would take the tip where the arm cannot go: refused by the hub, said on the status line, nothing moved
    call(hub, "POST", "/devices/arm/move_tip", {"x": 0.24, "y": 0.0, "z": 0.12})
    for _ in range(40):
        press(window, pg.K_r)
        window.frame(idle=True)
        if "-> 200" not in window.last_key:
            break
    assert "-> 4" in window.last_key and "nothing was moved" in window.last_key and window.open, window.last_key
    window.close()


def test_a_key_is_left_alone_while_a_command_runs_and_when_only_watching(system):
    world, hub = system
    window = W.Window(world, hub, hands_on=True)
    pg = window.pg
    jaws = call(hub, "GET", "/devices/arm/state")[1]["gripper"]["target_opening"]
    press(window, pg.K_SPACE)                                        # arrives while a caller's command is running
    status, r = call(hub, "POST", "/devices/arm/move_tip", {"x": 0.22, "y": 0.05, "z": 0.10})
    assert status == 200 and r["reached"]
    window.frame(idle=True)
    assert call(hub, "GET", "/devices/arm/state")[1]["gripper"]["target_opening"] == jaws and window.last_key == ""
    window.close()
    watching = W.Window(world, hub)                                  # opened for watching: keys do nothing
    t = world.time
    press(watching, pg.K_SPACE)
    press(watching, pg.K_w)
    watching.frame(idle=True)
    assert call(hub, "GET", "/devices/arm/state")[1]["gripper"]["target_opening"] == jaws and world.time == t
    press(watching, pg.K_ESCAPE)
    watching.frame(idle=True)
    assert not watching.open, "Esc closes the window either way"
    watching.close()


def test_the_window_follows_a_change_of_scene(system):
    world, hub = system
    window = W.Window(world, hub)
    world.load("no_arm")
    call(hub, "POST", "/discover")
    window.frame(idle=True)
    assert window.cameras == sorted(world.built.cameras) and "arm" not in hub.found()["devices"]
    assert window.screen.get_size()[0] == window._shape[0] * W.VIEW_W
    window.close()


def test_the_launcher_takes_the_window_flags():
    import subprocess
    import sys
    from pathlib import Path
    out = subprocess.run([sys.executable, str(Path(W.__file__).with_name("run_system.py")), "--help"], capture_output=True, text=True, timeout=60).stdout
    assert "--window" in out and "--hands-on" in out
