#!/usr/bin/env python3
"""Show the FFT spectrum, power over time and a spectrogram of a record_mat.py recording.

    plot_mat.py single.mat
    plot_mat.py multi.mat --save multi.png
    plot_mat.py single.mat --burst 12

The spectrum is computed the way MATLAB's fft would be, burst by burst:

    X = fftshift(fft(x_burst));          % one burst of real samples
    P = abs(X).^2 / N^2;                 % power per bin, relative to full scale
    S = 10*log10(mean(P_bursts));        % averaged over all bursts, in dBFS

Only the real samples are used. The zeros that record_mat.py puts between bursts would
otherwise pull every level down, by as much as the share of time that wasn't captured.
"""

import argparse
import pathlib
import sys

import numpy as np
from scipy import io, signal

import plot_capture as style

POWER_WINDOW_US = 10  # Averaging for the power-over-time trace
SPECTROGRAM_SEGMENT = 64


def load(path):
    """Return complex samples scaled so the ADC's full scale is 1.0, and the config."""
    contents = io.loadmat(path, squeeze_me=True, simplify_cells=True)
    if "iq" not in contents or "config" not in contents:
        sys.exit(f"{path} isn't a record_mat.py recording: it needs 'iq' and 'config'")
    iq, config = contents["iq"], contents["config"]
    x = (iq[0::2].astype(np.float64) + 1j * iq[1::2].astype(np.float64)) / config["adc_full_scale"]
    starts = np.atleast_1d(config["burst_start"]).astype(int) - 1  # MATLAB indices are 1-based
    lengths = np.atleast_1d(config["burst_length"]).astype(int)
    return x, config, list(zip(starts, lengths))


def fft_spectrum(bursts):
    """MATLAB's fftshift(fft(x)), as power relative to full scale, averaged over bursts.

    Bursts can differ in length by a sample or two, so each is cut to the shortest."""
    n = min(len(burst) for burst in bursts)
    power = np.zeros(n)
    for burst in bursts:
        spectrum = np.fft.fftshift(np.fft.fft(burst[:n]))
        power += np.abs(spectrum) ** 2 / n ** 2
    return 10 * np.log10(power / len(bursts) + 1e-20), n


def frequency_axis(n, sample_rate):
    """The same as MATLAB's ((0:n-1) - floor(n/2)) * fs/n, which matches fftshift."""
    return np.fft.fftshift(np.fft.fftfreq(n, 1 / sample_rate))


def plot(x, config, bursts, chosen):
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap

    fs = float(config["sample_rate_hz"])
    center_mhz = float(config["center_frequency_hz"]) / 1e6
    segments = [x[start:start + length] for start, length in bursts]
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), constrained_layout=True,
                             facecolor=style.SURFACE)
    for ax in axes:
        style.style_axes(ax)

    # Power over time, real samples only: gaps aren't drawn.
    window = max(1, int(POWER_WINDOW_US * 1e-6 * fs))
    for (start, length), segment in zip(bursts, segments):
        count = length // window
        if count == 0:
            continue
        power = np.mean(np.abs(segment[:count * window].reshape(count, window)) ** 2, axis=1)
        times = (start + np.arange(count) * window) / fs * 1e3
        axes[0].plot(times, 10 * np.log10(power + 1e-20), color=style.SERIES_I, linewidth=0.8)
    axes[0].set_xlim(0, len(x) / fs * 1e3)
    axes[0].set_xlabel("Time (ms)", color=style.TEXT_SECONDARY)
    axes[0].set_ylabel("Power (dBFS)", color=style.TEXT_SECONDARY)
    style.set_title(axes[0], f"Power over time ({len(bursts)} bursts, "
                             f"{float(config['captured_fraction']) * 100:.1f}% of the time; gaps not drawn)")

    # FFT spectrum, of one burst or averaged over all of them.
    chosen_segments = [segments[chosen]] if chosen is not None else segments
    spectrum, n = fft_spectrum(chosen_segments)
    freqs_mhz = frequency_axis(n, fs) / 1e6
    axes[1].plot(freqs_mhz, spectrum, color=style.SERIES_I, linewidth=0.9)
    axes[1].set_xlim(freqs_mhz[0], freqs_mhz[-1])
    axes[1].set_xlabel(f"Frequency relative to {center_mhz:g} MHz (MHz)", color=style.TEXT_SECONDARY)
    axes[1].set_ylabel("Power per bin (dBFS)", color=style.TEXT_SECONDARY)
    which = f"burst {chosen}" if chosen is not None else f"averaged over {len(segments)} bursts"
    style.set_title(axes[1], f"FFT spectrum, fftshift(fft(x)) of {n} samples, {which}")

    # Spectrogram of the chosen burst, or the one with the most energy.
    index = chosen if chosen is not None else int(np.argmax([np.mean(np.abs(s) ** 2) for s in segments]))
    start, length = bursts[index]
    segment_length = min(SPECTROGRAM_SEGMENT, length)
    freqs, times, power = signal.spectrogram(segments[index], fs=fs, nperseg=segment_length,
                                             noverlap=segment_length * 3 // 4,
                                             return_onesided=False, detrend=False, mode="psd")
    colormap = LinearSegmentedColormap.from_list("power", style.WATERFALL_RAMP)
    mesh = axes[2].pcolormesh(times * 1e3, np.fft.fftshift(freqs) / 1e6,
                              10 * np.log10(np.fft.fftshift(power, axes=0) + 1e-20),
                              cmap=colormap, shading="auto")
    colorbar = fig.colorbar(mesh, ax=axes[2], pad=0.01)
    colorbar.set_label("dB/Hz", color=style.TEXT_SECONDARY)
    axes[2].set_xlabel("Time within the burst (ms)", color=style.TEXT_SECONDARY)
    axes[2].set_ylabel("Baseband frequency (MHz)", color=style.TEXT_SECONDARY)
    style.set_title(axes[2], f"Spectrogram of burst {index} ({length / fs * 1e3:.2f} ms at "
                             f"{start / fs * 1e3:.0f} ms)")

    fig.suptitle(f"{center_mhz:g} MHz · {fs / 1e6:g} MS/s · gain index {config['gain_index']} · "
                 f"{len(x) / fs:.2f} s recorded", x=0.01, ha="left", color=style.TEXT, fontsize=12)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", type=pathlib.Path, help=".mat file from record_mat.py")
    parser.add_argument("--burst", type=int, help="show this burst instead of averaging all of them")
    parser.add_argument("--save", type=pathlib.Path, help="write an image instead of opening a window")
    args = parser.parse_args()

    x, config, bursts = load(args.recording)
    if not bursts:
        sys.exit("the recording has no bursts")
    if args.burst is not None and not 0 <= args.burst < len(bursts):
        parser.error(f"--burst must be 0-{len(bursts) - 1}")

    if args.save:
        import matplotlib
        matplotlib.use("Agg")
    fig = plot(x, config, bursts, args.burst)
    if args.save:
        fig.savefig(args.save, dpi=110, facecolor=style.SURFACE)
        print(f"saved {args.save}")
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
