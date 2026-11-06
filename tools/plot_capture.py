#!/usr/bin/env python3
"""Plot a capture saved by pi_receiver.py: I(t), Q(t), FFT spectrum and spectrogram."""

import argparse
import json
import pathlib
import sys

import numpy as np

FULL_SCALE = 512  # Signed 10-bit samples
MIN_SPECTROGRAM_SAMPLES = 1024
DEFAULT_RATE_MSPS = 80

SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
MUTED = "#898781"
BASELINE = "#c3c2b7"
GRID = "#e8e7e1"
SERIES_I = "#2a78d6"
SERIES_Q = "#eb6834"
RAW_TRACE = "#c3c2b7"
WATERFALL_RAMP = ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"]


def load_capture(path, rate_msps):
    """Return (complex samples, metadata). Metadata comes from the .json sidecar if any."""
    iq = np.load(path)
    if iq.ndim != 1 or not np.iscomplexobj(iq):
        sys.exit(f"{path}: expected a 1-D complex array, got {iq.dtype} {iq.shape}")

    sidecar = path.with_suffix(".json")
    metadata = json.loads(sidecar.read_text()) if sidecar.exists() else {}
    if rate_msps is not None:
        metadata["sample_rate_hz"] = rate_msps * 1_000_000
    elif "sample_rate_hz" not in metadata:
        print(f"warning: no {sidecar.name} and no --rate, assuming {DEFAULT_RATE_MSPS} MS/s")
        metadata["sample_rate_hz"] = DEFAULT_RATE_MSPS * 1_000_000
    return iq.astype(np.complex128), metadata


def select_burst(iq, metadata, burst):
    """Bursts in a file have gaps between them, so only one can be analyzed at a time."""
    starts = metadata.get("burst_starts")
    if not starts:
        return iq
    if not 0 <= burst < len(starts):
        sys.exit(f"--burst must be 0-{len(starts) - 1}")
    end = starts[burst + 1] if burst + 1 < len(starts) else len(iq)
    metadata["burst_index"] = burst
    channel = metadata["burst_channels"][burst]
    if channel:
        metadata["channel"] = channel
        metadata["center_mhz"] = 2484 if channel == 14 else 2407 + 5 * channel
    return iq[starts[burst]:end]


def dbfs(power):
    return 10 * np.log10(np.maximum(power, 1e-20))


def windowed_spectrum(iq):
    """Return power relative to full scale: a full-scale complex tone reads 0 dBFS."""
    window = np.hanning(len(iq))
    spectrum = np.fft.fftshift(np.fft.fft(iq * window))
    return np.abs(spectrum) ** 2 / (FULL_SCALE * window.sum()) ** 2


def offset_axis(samples, rate_hz):
    return np.fft.fftshift(np.fft.fftfreq(samples, 1 / rate_hz))


def dc_mask(freqs, notch_hz):
    """Select the bins hidden around DC: at least the Hann main lobe, where removing the
    mean leaves a hole rather than signal. Captures show no DC skirt beyond that."""
    return np.abs(freqs) <= max(notch_hz, 2.5 * abs(freqs[1] - freqs[0]))


def smooth(power, bins):
    return np.convolve(power, np.ones(bins) / bins, mode="same")


def spectrogram(iq, segment, hop):
    """Return power per (time, frequency). Each segment loses its own mean, so a DC
    step from the AGC changing gain mid-capture can't smear across the band."""
    starts = np.arange(0, len(iq) - segment + 1, hop)
    window = np.hanning(segment)
    lines = np.empty((len(starts), segment))
    for row, start in enumerate(starts):
        chunk = iq[start:start + segment]
        lines[row] = np.abs(np.fft.fftshift(np.fft.fft((chunk - chunk.mean()) * window))) ** 2
    return lines / (FULL_SCALE * window.sum()) ** 2, starts


def spectrogram_segment(samples):
    """Pick a power-of-two segment giving at least 16 non-overlapping time slices."""
    return min(512, max(64, 2 ** int(np.log2(samples / 16))))


def analyze(iq, rate_hz, notch_hz, keep_dc):
    """Compute everything the plot and the printed summary need."""
    mean = iq.mean()
    conditioned = iq if keep_dc else iq - mean
    freqs = offset_axis(len(iq), rate_hz)
    power = windowed_spectrum(conditioned)
    hidden = np.zeros_like(freqs, dtype=bool) if keep_dc else dc_mask(freqs, notch_hz)
    visible = np.where(~hidden)[0]
    peak = visible[np.argmax(power[visible])]
    return {
        "mean": mean,
        "iq": conditioned,
        "freqs": freqs,
        "power": power,
        "hidden": hidden,
        "hidden_hz": np.abs(freqs[hidden]).max() if hidden.any() else 0,
        "peak_hz": freqs[peak],
        "peak_dbfs": dbfs(power[peak]),
        "floor_dbfs": dbfs(np.median(power[visible])),
    }


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=9, length=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE)


def set_title(ax, text):
    ax.set_title(text, loc="left", color=TEXT, fontsize=11)


def describe(metadata, samples):
    rate = metadata["sample_rate_hz"] / 1e6
    parts = []
    if "channel" in metadata:
        parts.append(f"Channel {metadata['channel']} ({metadata['center_mhz']} MHz)")
    elif "center_mhz" in metadata:
        parts.append(f"{metadata['center_mhz']} MHz")
    if metadata.get("bursts", 1) > 1:
        parts.append(f"Burst {metadata['burst_index']} of {metadata['bursts']}")
    parts.append(f"{samples} samples at {rate:g} MS/s ({samples / rate:.1f} µs)")
    if "gain_requested" in metadata:
        low, high = metadata["gain_min"], metadata["gain_max"]
        if metadata["gain_requested"] == "agc":
            changed = "" if low == high else ", changed during capture"
            parts.append(f"AGC, gain index {low}" + (f"-{high}" if low != high else "") + changed)
        else:
            parts.append(f"manual gain index {metadata['gain_requested']}")
    return " · ".join(parts)


def plot_trace(ax, time_us, values, color, name, limit):
    style_axes(ax)
    ax.plot(time_us, values, color=color, linewidth=0.7)
    ax.set_ylim(-limit, limit)
    ax.set_xlim(time_us[0], time_us[-1])
    ax.set_ylabel("Counts", color=TEXT_SECONDARY, fontsize=9)
    set_title(ax, name)


def plot_spectrum(ax, result, keep_dc):
    style_axes(ax)
    freqs_mhz = result["freqs"] / 1e6
    power = result["power"]
    bins = max(1, len(power) // 128)

    raw = dbfs(power)
    smoothed = dbfs(smooth(power, bins))
    raw[result["hidden"]] = np.nan
    smoothed[result["hidden"]] = np.nan
    ax.plot(freqs_mhz, raw, color=RAW_TRACE, linewidth=0.5, label="FFT")
    resolution_khz = abs(result["freqs"][1] - result["freqs"][0]) * bins / 1e3
    ax.plot(freqs_mhz, smoothed, color=SERIES_I, linewidth=1.5,
            label=f"Smoothed over {resolution_khz:.0f} kHz")

    if not keep_dc:
        edge = np.abs(freqs_mhz[result["hidden"]]).max()
        ax.axvspan(-edge, edge, color=GRID, zorder=0)

    ax.set_xlim(freqs_mhz[0], freqs_mhz[-1])
    floor = np.nanpercentile(raw, 1)
    ax.set_ylim(floor - 3, max(np.nanmax(raw) + 5, floor + 30))
    ax.set_xlabel("Offset from tuned frequency (MHz)", color=TEXT_SECONDARY, fontsize=9)
    ax.set_ylabel("dBFS", color=TEXT_SECONDARY, fontsize=9)
    legend = ax.legend(loc="upper right", frameon=False, fontsize=9)
    for text in legend.get_texts():
        text.set_color(TEXT_SECONDARY)
    note = "DC shown" if keep_dc else f"DC removed, ±{result['hidden_hz'] / 1e3:.0f} kHz hidden"
    set_title(ax, f"Spectrum ({note})")


def plot_spectrogram(fig, ax, result, rate_hz, notch_hz, keep_dc):
    from matplotlib.colors import LinearSegmentedColormap

    style_axes(ax)
    ax.grid(False)
    iq = result["iq"]
    segment = spectrogram_segment(len(iq))
    hop = segment // 4
    lines, starts = spectrogram(iq, segment, hop)

    freqs = offset_axis(segment, rate_hz)
    image = dbfs(lines)
    if not keep_dc:
        image[:, dc_mask(freqs, notch_hz)] = np.nan

    colormap = LinearSegmentedColormap.from_list("power", WATERFALL_RAMP)
    colormap.set_bad(GRID)
    step_us = hop / rate_hz * 1e6
    extent = [freqs[0] / 1e6, freqs[-1] / 1e6,
              (starts[-1] + segment) / rate_hz * 1e6, starts[0] / rate_hz * 1e6]
    low, high = np.nanpercentile(image, [5, 99.9])
    mesh = ax.imshow(image, aspect="auto", extent=extent, cmap=colormap, vmin=low, vmax=high,
                     interpolation="nearest")
    colorbar = fig.colorbar(mesh, ax=ax, pad=0.01)
    colorbar.set_label("dBFS", color=TEXT_SECONDARY, fontsize=9)
    colorbar.ax.tick_params(colors=TEXT_SECONDARY, labelsize=9, length=0)
    colorbar.outline.set_visible(False)
    ax.set_xlabel("Offset from tuned frequency (MHz)", color=TEXT_SECONDARY, fontsize=9)
    ax.set_ylabel("Time (µs)", color=TEXT_SECONDARY, fontsize=9)
    set_title(ax, f"Spectrogram ({segment}-point FFT, a new line every {step_us:.1f} µs)")


def make_figure(result, metadata, rate_hz, notch_hz, keep_dc):
    import matplotlib.pyplot as plt

    iq = result["iq"]
    show_spectrogram = len(iq) >= MIN_SPECTROGRAM_SAMPLES
    ratios = [1, 1, 2] + ([3] if show_spectrogram else [])
    fig, axes = plt.subplots(len(ratios), 1, figsize=(11, 3 + 2 * sum(ratios) * 0.6),
                             height_ratios=ratios, constrained_layout=True, facecolor=SURFACE)

    time_us = np.arange(len(iq)) / rate_hz * 1e6
    limit = max(np.abs(iq.real).max(), np.abs(iq.imag).max(), 1) * 1.1
    suffix = "" if keep_dc else " (mean removed)"
    plot_trace(axes[0], time_us, iq.real, SERIES_I, f"I(t){suffix}", limit)
    plot_trace(axes[1], time_us, iq.imag, SERIES_Q, f"Q(t){suffix}", limit)
    axes[1].set_xlabel("Time (µs)", color=TEXT_SECONDARY, fontsize=9)
    plot_spectrum(axes[2], result, keep_dc)
    if show_spectrogram:
        plot_spectrogram(fig, axes[3], result, rate_hz, notch_hz, keep_dc)
    fig.suptitle(describe(metadata, len(iq)), x=0.01, ha="left", color=TEXT, fontsize=12)
    return fig


def print_summary(result, metadata, keep_dc):
    mean = result["mean"]
    print(describe(metadata, len(result["iq"])))
    if metadata.get("dc_removed"):
        print(f"  DC at capture: I={metadata['mean_i']:.2f} Q={metadata['mean_q']:.2f} counts "
              "(already removed from this file)")
    else:
        print(f"  DC: I={mean.real:.2f} Q={mean.imag:.2f} counts"
              f"{' (kept)' if keep_dc else ' (removed for plotting)'}")
    print(f"  strongest bin: {result['peak_hz'] / 1e6:+.3f} MHz at {result['peak_dbfs']:.1f} dBFS"
          f"{'' if keep_dc else f', ignoring ±{result["hidden_hz"] / 1e3:.0f} kHz around DC'}")
    print(f"  median bin: {result['floor_dbfs']:.1f} dBFS")
    if len(result["iq"]) < MIN_SPECTROGRAM_SAMPLES:
        print(f"  spectrogram skipped: needs at least {MIN_SPECTROGRAM_SAMPLES} samples")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=pathlib.Path, help=".npy file from pi_receiver.py --out")
    parser.add_argument("--save", type=pathlib.Path, help="write an image instead of opening a window")
    parser.add_argument("--rate", type=float, help="sample rate in MS/s, if the .json sidecar is missing")
    parser.add_argument("--burst", type=int, default=0, help="burst to plot (default: %(default)s)")
    parser.add_argument("--dc-notch-khz", type=float, default=0,
                        help="extra width to hide either side of DC (default: only the window's main lobe)")
    parser.add_argument("--keep-dc", action="store_true",
                        help="plot the data as saved, with no DC removal or hidden bins")
    args = parser.parse_args()

    iq, metadata = load_capture(args.capture, args.rate)
    iq = select_burst(iq, metadata, args.burst)
    rate_hz = metadata["sample_rate_hz"]
    notch_hz = args.dc_notch_khz * 1e3
    result = analyze(iq, rate_hz, notch_hz, args.keep_dc)
    print_summary(result, metadata, args.keep_dc)

    if args.save:
        import matplotlib
        matplotlib.use("Agg")
    fig = make_figure(result, metadata, rate_hz, notch_hz, args.keep_dc)
    if args.save:
        fig.savefig(args.save, dpi=130, facecolor=SURFACE)
        print(f"saved {args.save}")
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
