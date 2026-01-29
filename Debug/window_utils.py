from typing import Optional


def place_dashboard_on_monitor(dashboard, app, monitor_index: int = -1, center: bool = True):
    """Place `dashboard` on the selected monitor.

    - `monitor_index`: index in `app.screens()` to choose. Negative selects from the end (default -1).
    - `center`: if True, center on the chosen monitor; otherwise place top-right.
    """
    app.processEvents()
    try:
        screens = app.screens()
        if not screens:
            raise RuntimeError("No screens available")
        # Normalize index
        if monitor_index < 0:
            screen = screens[monitor_index]
        else:
            idx = min(max(0, monitor_index), len(screens) - 1)
            screen = screens[idx]

        rect = screen.geometry()
        win_geo = dashboard.frameGeometry()
        win_w = win_geo.width()
        win_h = win_geo.height()

        if center:
            x = int(rect.x() + (rect.width() - win_w) / 2)
            y = int(rect.y() + (rect.height() - win_h) / 2)
        else:
            x = rect.x() + rect.width() - win_w - 10
            y = rect.y() + 10

        # Clamp into available geometry
        x = max(rect.x() + 10, min(x, rect.x() + rect.width() - win_w - 10))
        y = max(rect.y() + 10, min(y, rect.y() + rect.height() - win_h - 10))

        dashboard.move(x, y)
    except Exception:
        dashboard.move(200, 200)
