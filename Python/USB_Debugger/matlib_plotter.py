import time

import numpy as np
import queue
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

import config


def main_plot_loop(plot_queue, stop_event):

    while not stop_event.is_set():
        drained = []

        while True:
            try:
                drained.append(plot_queue.get_nowait())
            except queue.Empty:
                break

        for decoded in drained:
            current_signal_names = list(decoded.get("signals", {}).keys())

            if len(current_signal_names) == 0:
                continue

            needs_reinit = (
                config.plot_state is None
                or config.plot_state["signal_names"] != current_signal_names
            )

            if needs_reinit:
                config.plot_state = init_live_plot(
                    decoded,
                    decimation=config.LOG_PLOT_DECIMATION,
                    update_period_s=config.LOG_PLOT_UPDATE_PERIOD_S,
                    timestamp_hz=config.TIMESTAMP_HZ,
                    existing_plot_state=config.plot_state,
                )
                print(f"Initialized live plot with signals: {config.plot_state['signal_names']}")

            queue_live_plot_data(config.plot_state, decoded)

        if config.plot_state is not None:
            update_live_plot_if_due(config.plot_state)
            plt.pause(0.001)
        else:
            plt.pause(0.05)


def init_live_plot(decoded,
                   decimation=config.LOG_PLOT_DECIMATION,
                   max_points=config.LOG_PLOT_MAX_POINTS,
                   update_period_s=config.LOG_PLOT_UPDATE_PERIOD_S,
                   queue_maxsize=200,
                   timestamp_hz=config.TIMESTAMP_HZ,
                   existing_plot_state=None):
    signal_names = list(decoded["signals"].keys())
    n_signals = len(signal_names)

    if n_signals == 0:
        return None

    if existing_plot_state is not None and existing_plot_state.get("fig") is not None:
        fig = existing_plot_state["fig"]
        is_paused = existing_plot_state.get("is_paused", False)
        fig.clf()
        fig.set_size_inches(12, max(2.5 * n_signals, 4), forward=True)
        axes = fig.subplots(n_signals, 1, sharex=True, squeeze=False)
    else:
        fig, axes = plt.subplots(
            n_signals, 1,
            sharex=True,
            figsize=(12, max(2.5 * n_signals, 4)),
            squeeze=False
        )
        is_paused = False
        plt.ion()
        plt.show(block=False)

    axes = axes[:, 0]

    lines = {}
    x_buffer = deque(maxlen=max_points)
    y_buffers = {}

    for ax, name in zip(axes, signal_names):
        line, = ax.plot([], [], lw=1.0, label=name)
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

        lines[name] = line
        y_buffers[name] = deque(maxlen=max_points)

    axes[-1].set_xlabel(f"Time [s] (decimated x{decimation})")

    ax_pause = fig.add_axes([0.8, 0.025, 0.1, 0.075])
    pause_button = Button(ax_pause, 'Resume' if is_paused else 'Pause')

    plot_state = {
        "fig": fig,
        "axes": axes,
        "lines": lines,
        "signal_names": signal_names,
        "x_buffer": x_buffer,
        "y_buffers": y_buffers,
        "decimation": decimation,
        "max_points": max_points,
        "update_period_s": update_period_s,
        "last_plot_time": 0.0,
        "queue": queue.Queue(maxsize=queue_maxsize),
        "timestamp_hz": timestamp_hz,
        "is_paused": is_paused,
        "pause_button": pause_button,
        "pause_ax": ax_pause,
    }

    def toggle_pause(event):
        plot_state["is_paused"] = not plot_state["is_paused"]
        print(f"Paused: {plot_state['is_paused']}")
        pause_button.label.set_text('Resume' if plot_state["is_paused"] else 'Pause')
        fig.canvas.draw_idle()

    pause_button.on_clicked(toggle_pause)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

    return plot_state


def queue_live_plot_data(plot_state, decoded):
    decimation = plot_state["decimation"]
    timestamp_hz = plot_state["timestamp_hz"]

    sample_count = decoded["sample_count"]
    base_timestamp = decoded["timestamp"]

    idx = np.arange(0, sample_count, decimation)
    if idx.size == 0:
        return

    time_array = (
        np.arange(base_timestamp, base_timestamp + sample_count, dtype=np.float64)
        / timestamp_hz
    )[idx]

    payload = {
        "x": time_array,
        "signals": {
            name: np.asarray(decoded["signals"][name])[idx]
            for name in plot_state["signal_names"]
        }
    }

    try:
        plot_state["queue"].put_nowait(payload)
    except queue.Full:
        try:
            plot_state["queue"].get_nowait()
        except queue.Empty:
            pass
        try:
            plot_state["queue"].put_nowait(payload)
        except queue.Full:
            pass


def update_live_plot_if_due(plot_state, force=False):
    is_paused = plot_state["is_paused"]
    if is_paused:
        return

    now = time.perf_counter()

    if not force and (now - plot_state["last_plot_time"] < plot_state["update_period_s"]):
        return

    drained = []

    while True:
        try:
            drained.append(plot_state["queue"].get_nowait())
        except queue.Empty:
            break

    if not drained:
        if force:
            plot_state["fig"].canvas.draw_idle()
            plot_state["fig"].canvas.flush_events()
        plot_state["last_plot_time"] = now
        return

    for item in drained:
        plot_state["x_buffer"].extend(item["x"].tolist())

        for name in plot_state["signal_names"]:
            plot_state["y_buffers"][name].extend(np.asarray(item["signals"][name]).tolist())

    x = np.asarray(plot_state["x_buffer"])
    if x.size == 0:
        plot_state["last_plot_time"] = now
        return

    for ax, name in zip(plot_state["axes"], plot_state["signal_names"]):
        y = np.asarray(plot_state["y_buffers"][name])
        plot_state["lines"][name].set_data(x, y)

        if y.size > 0:
            ymin = np.min(y)
            ymax = np.max(y)
            if ymin == ymax:
                pad = 1.0 if ymin == 0 else abs(ymin) * 0.05
            else:
                pad = (ymax - ymin) * 0.1
            ax.set_ylim(ymin - pad, ymax + pad)

    if x.size > 1 and x[0] != x[-1]:
        plot_state["axes"][-1].set_xlim(x[0], x[-1])

    plot_state["fig"].canvas.draw_idle()
    plot_state["fig"].canvas.flush_events()
    plot_state["last_plot_time"] = now