import types
import pytest


class FakeGamepadReader:
    def __init__(self, axes=None, buttons=None):
        # Default to 6 axes as in GamepadReader.axis_labels
        self._axes = axes if axes is not None else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._buttons = buttons if buttons is not None else [False] * 10
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def get_axes(self):
        return list(self._axes)

    def get_buttons(self):
        return list(self._buttons)


@pytest.fixture()
def patch_gamepad(monkeypatch):
    import user_command as uc

    fake_holder = types.SimpleNamespace(instance=None)

    def factory(*args, **kwargs):
        # Create a new fake per UserCommand instance unless set explicitly
        inst = FakeGamepadReader()
        fake_holder.instance = inst
        return inst

    monkeypatch.setattr(uc, "GamepadReader", factory)
    return fake_holder


def make_user_command_with_axes(monkeypatch, axes):
    import user_command as uc

    def factory(*args, **kwargs):
        return FakeGamepadReader(axes=axes)

    monkeypatch.setattr(uc, "GamepadReader", factory)
    return uc.UserCommand()


def test_constructor_starts_reader(patch_gamepad):
    import user_command as uc

    cmd = uc.UserCommand()
    assert patch_gamepad.instance is not None
    assert patch_gamepad.instance.started is True


@pytest.mark.parametrize(
    "axes,lim,deadzone,expected",
    [
        # axes index 1 controls throttle (negated), apply deadzone
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0, 0.05, 0.0),
        ([0.0, 0.04, 0.0, 0.0, 0.0, 0.0], 1.0, 0.05, 0.0),  # within deadzone
        ([0.0, 0.06, 0.0, 0.0, 0.0, 0.0], 1.0, 0.05, -0.06),
        ([0.0, -0.5, 0.0, 0.0, 0.0, 0.0], 2.0, 0.0, 1.0),  # scale by lim and sign
    ],
)
def test_throttle(monkeypatch, axes, lim, deadzone, expected):
    cmd = make_user_command_with_axes(monkeypatch, axes)
    assert pytest.approx(cmd.throttle(lim=lim, deadzone=deadzone), rel=1e-6) == expected


@pytest.mark.parametrize(
    "axes,lim,expected",
    [
        ([0.2, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0, -0.2),  # yaw uses axes[0], negated
        ([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5, 0.25),
    ],
)
def test_yaw(monkeypatch, axes, lim, expected):
    cmd = make_user_command_with_axes(monkeypatch, axes)
    assert pytest.approx(cmd.yaw(lim=lim), rel=1e-6) == expected


@pytest.mark.parametrize(
    "axes,lim,expected",
    [
        # roll uses axes[3], negated
        ([0.0, 0.0, 0.0, 0.3, 0.0, 0.0], 1.0, -0.3),
        ([0.0, 0.0, 0.0, -0.4, 0.0, 0.0], 2.0, 0.8),
    ],
)
def test_roll(monkeypatch, axes, lim, expected):
    cmd = make_user_command_with_axes(monkeypatch, axes)
    assert pytest.approx(cmd.roll(lim=lim), rel=1e-6) == expected


@pytest.mark.parametrize(
    "axes,lim,expected",
    [
        # pitch uses axes[4], negated
        ([0.0, 0.0, 0.0, 0.0, 0.7, 0.0], 1.0, -0.7),
        ([0.0, 0.0, 0.0, 0.0, -0.25, 0.0], 0.4, 0.1),
    ],
)
def test_pitch(monkeypatch, axes, lim, expected):
    cmd = make_user_command_with_axes(monkeypatch, axes)
    assert pytest.approx(cmd.pitch(lim=lim), rel=1e-6) == expected


def test_stop_calls_reader_stop(patch_gamepad):
    import user_command as uc

    cmd = uc.UserCommand()
    cmd.stop()
    assert patch_gamepad.instance.stopped is True


