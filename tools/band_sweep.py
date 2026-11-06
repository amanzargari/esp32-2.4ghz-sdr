#!/usr/bin/env python3
"""Stitch a channel sweep from pi_receiver.py into a panorama of the 2.4 GHz band.

    band_sweep.py sweep.npy --save panorama.png
    band_sweep.py --connect 192.168.77.2:5555 --passes 20 --save panorama.png
    band_sweep.py --connect 192.168.77.2:5555 --live

Each frequency is seen from about four tunings, at different offsets from the tuned
frequency. A real signal stays at the same frequency in all of them, while an internal
spur stays at the same offset and so moves with the tuning. That lets the average trace
take a median across tunings, and lets spurs be found and removed.
"""

import argparse
import json
import pathlib
import sys

import numpy as np

import pi_receiver
import plot_capture as style

BAND_MHZ = (2400.0, 2485.0)
FFT_SIZE = 1024
FULL_SENSITIVITY_MHZ = 10.0  # Inside this the filter is close to flat
ROLLOFF_LIMIT_MHZ = 12.5  # Only band edges use this far out, where correcting for the
                          # filter's rolloff inflates noise. Beyond it is mostly ADC noise.
DC_BINS = 2  # The window's main lobe, which removing the mean leaves as a hole
MIN_CHANNELS_FOR_RESPONSE = 5
RESPONSE_SMOOTHING_MHZ = 1.0
SPUR_THRESHOLD_DB = 6.0  # Above the smoothed response, at the same offset on every tuning
MAX_WATERFALL_PASSES = 200
FULL_SCALE = pi_receiver.FULL_SCALE


def to_db(power):
    return 10 * np.log10(np.maximum(power, 1e-20))


def burst_psd(iq, rate_hz):
    """Welch PSD, normalized so it integrates to the signal's power relative to a
    full-scale tone. Returns linear power per MHz, with DC at the center."""
    window = np.hanning(FFT_SIZE)
    iq = iq - iq.mean()
    starts = range(0, len(iq) - FFT_SIZE + 1, FFT_SIZE // 2)
    power = np.zeros(FFT_SIZE)
    for start in starts:
        power += np.abs(np.fft.fft(iq[start:start + FFT_SIZE] * window)) ** 2
    scale = len(starts) * FULL_SCALE ** 2 * rate_hz * np.sum(window ** 2)
    return np.fft.fftshift(power) / scale * 1e6


def moving_average(values, bins):
    return np.convolve(np.pad(values, bins // 2, mode="edge"), np.ones(bins) / bins, mode="valid")


class Panorama:
    """Accumulates bursts by channel, then stitches them onto one frequency axis."""

    def __init__(self, channels, rate_hz, burst_samples):
        if burst_samples < FFT_SIZE:
            sys.exit(f"bursts must have at least {FFT_SIZE} samples for a sweep")
        self.channels = list(channels)
        self.rate_hz = rate_hz
        self.offsets_mhz = np.fft.fftshift(np.fft.fftfreq(FFT_SIZE, 1e6 / rate_hz))
        self.bin_mhz = rate_hz / FFT_SIZE / 1e6
        self.sums = {channel: np.zeros(FFT_SIZE) for channel in self.channels}
        self.peaks = {channel: np.zeros(FFT_SIZE) for channel in self.channels}
        self.counts = {channel: 0 for channel in self.channels}
        self.passes = {}

        self.full_mhz = min(FULL_SENSITIVITY_MHZ, rate_hz / 1e6 * 0.45)
        self.rolloff_mhz = min(ROLLOFF_LIMIT_MHZ, rate_hz / 1e6 * 0.45)
        # Anchored to channel 1, so channels 1-13 land exactly on the grid's bins.
        anchor = pi_receiver.channel_mhz(1)
        low = np.ceil((BAND_MHZ[0] - anchor) / self.bin_mhz)
        high = np.floor((BAND_MHZ[1] - anchor) / self.bin_mhz)
        self.grid_mhz = anchor + np.arange(low, high + 1) * self.bin_mhz

    def add(self, number, channel, words):
        i, q, _ = pi_receiver.unpack_iq(words)
        self.add_iq(number, channel, (i + 1j * q).astype(np.complex128))

    def add_iq(self, number, channel, iq):
        psd = burst_psd(iq, self.rate_hz)
        self.sums[channel] += psd
        self.peaks[channel] = np.maximum(self.peaks[channel], psd)
        self.counts[channel] += 1

        index = number // len(self.channels)
        if index not in self.passes and len(self.passes) >= MAX_WATERFALL_PASSES:
            del self.passes[min(self.passes)]
        self.passes.setdefault(index, {})[channel] = psd

    def measured_channels(self):
        return [channel for channel in self.channels if self.counts[channel]]

    def filter_response(self):
        """Estimate the receiver's shape from the sweep itself: a real signal is at a
        different offset on each tuning, so the median across tunings leaves the shape
        plus anything fixed to the tuning, which is what a spur is."""
        measured = self.measured_channels()
        flat = np.ones(FFT_SIZE)
        no_spurs = np.zeros(FFT_SIZE, dtype=bool)
        if len(measured) < MIN_CHANNELS_FOR_RESPONSE:
            return flat, no_spurs

        averages = np.array([self.sums[c] / self.counts[c] for c in measured])
        median_db = to_db(np.median(averages, axis=0))
        bins = max(3, int(RESPONSE_SMOOTHING_MHZ / self.bin_mhz)) | 1

        dc = np.abs(self.offsets_mhz) <= DC_BINS * self.bin_mhz
        filled = median_db.copy()
        filled[dc] = np.interp(self.offsets_mhz[dc], self.offsets_mhz[~dc], median_db[~dc])
        # Taking the median again first keeps the spurs out of the smoothed shape.
        half = bins // 2
        pad = np.pad(filled, half, mode="edge")
        rolling = np.median(np.lib.stride_tricks.sliding_window_view(pad, bins), axis=1)
        smooth_db = moving_average(rolling, bins)

        usable = np.abs(self.offsets_mhz) <= self.rolloff_mhz
        spurs = usable & ~dc & (filled - smooth_db > SPUR_THRESHOLD_DB)
        center = np.abs(self.offsets_mhz) <= FULL_SENSITIVITY_MHZ / 2
        response_db = smooth_db - np.median(smooth_db[center])
        return 10 ** (response_db / 10), spurs

    def _to_grid(self, psd, channel, response, excluded):
        center = pi_receiver.channel_mhz(channel)
        corrected = psd / response
        corrected[excluded] = np.nan
        in_range = (self.grid_mhz >= center + self.offsets_mhz[0]) & \
                   (self.grid_mhz <= center + self.offsets_mhz[-1])
        values = np.full(len(self.grid_mhz), np.nan)
        positions = (self.grid_mhz[in_range] - center - self.offsets_mhz[0]) / self.bin_mhz
        lower = np.clip(np.floor(positions).astype(int), 0, FFT_SIZE - 2)
        fraction = positions - lower
        values[in_range] = corrected[lower] * (1 - fraction) + corrected[lower + 1] * fraction
        return values

    def _stack(self, psds, response, unusable):
        """Put each tuning's PSD on the grid. A frequency only uses a tuning's rolloff
        region if no tuning covers it at full sensitivity."""
        full = np.abs(self.offsets_mhz) > self.full_mhz
        rolloff = np.abs(self.offsets_mhz) > self.rolloff_mhz
        main = np.array([self._to_grid(psd, c, response, unusable | full) for c, psd in psds])
        edge = np.array([self._to_grid(psd, c, response, unusable | rolloff) for c, psd in psds])
        use_edge = np.all(np.isnan(main), axis=0)
        main[:, use_edge] = edge[:, use_edge]
        return main

    @staticmethod
    def _combine(stack, reducer):
        combined = np.full(stack.shape[1], np.nan)
        seen = np.any(~np.isnan(stack), axis=0)
        combined[seen] = reducer(stack[:, seen], axis=0)
        return combined

    def stitch(self):
        response, spurs = self.filter_response()
        dc = np.abs(self.offsets_mhz) <= DC_BINS * self.bin_mhz
        unusable = dc | spurs

        measured = self.measured_channels()
        averages = self._stack([(c, self.sums[c] / self.counts[c]) for c in measured],
                               response, unusable)
        peaks = self._stack([(c, self.peaks[c]) for c in measured], response, unusable)
        coverage = np.sum(~np.isnan(averages), axis=0)
        average = self._combine(averages, np.nanmedian)
        peak = self._combine(peaks, np.nanmax)

        rows = [self._combine(self._stack(list(self.passes[index].items()), response, unusable),
                              np.nanmax)
                for index in sorted(self.passes)]

        return {
            "freq_mhz": self.grid_mhz,
            "average": average,
            "peak": peak,
            "coverage": coverage,
            "waterfall": np.array(rows),
            "response_offsets_mhz": self.offsets_mhz,
            "response": response,
            "spur_offsets_mhz": self.offsets_mhz[spurs],
            "bin_mhz": self.bin_mhz,
        }


def full_sensitivity_range(channels):
    return (pi_receiver.channel_mhz(channels[0]) - FULL_SENSITIVITY_MHZ,
            pi_receiver.channel_mhz(channels[-1]) + FULL_SENSITIVITY_MHZ)


def group_spurs(offsets, bin_mhz):
    """Merge neighbouring bins into one spur each, returning their center offsets."""
    groups = []
    for offset in offsets:
        if groups and offset - groups[-1][-1] <= 2 * bin_mhz:
            groups[-1].append(offset)
        else:
            groups.append([offset])
    return [float(np.mean(group)) for group in groups]


def print_summary(panorama, result):
    measured = panorama.measured_channels()
    low, high = full_sensitivity_range(measured)
    print(f"channels {measured[0]}-{measured[-1]}, "
          f"{min(panorama.counts[c] for c in measured)} or more bursts each, "
          f"{result['bin_mhz'] * 1e3:.0f} kHz resolution")
    print(f"full sensitivity: {low:.0f}-{high:.0f} MHz. Outside that the filter's rolloff "
          "leaves the trace unreliable.")

    print("channel  center MHz  average dBFS/MHz  peak dBFS/MHz (over each 20 MHz)")
    average_db = to_db(result["average"])
    peak_db = to_db(result["peak"])
    for channel in measured:
        center = pi_receiver.channel_mhz(channel)
        band = (np.abs(result["freq_mhz"] - center) <= FULL_SENSITIVITY_MHZ) & \
               ~np.isnan(result["average"])
        if band.any():
            print(f"{channel:7d}  {center:10d}  {np.mean(average_db[band]):16.1f}  "
                  f"{np.max(peak_db[band]):13.1f}")

    spurs = group_spurs(result["spur_offsets_mhz"], result["bin_mhz"])
    if spurs:
        print("removed as internal spurs, at these offsets from the tuned frequency (MHz): " +
              ", ".join(f"{offset:+.2f}" for offset in spurs))
    else:
        print("no internal spurs found")


def channel_axis(ax, channels):
    top = ax.secondary_xaxis("top")
    ticks = [channel for channel in channels if channel in (1, 6, 11, 13, 14)]
    top.set_xticks([pi_receiver.channel_mhz(channel) for channel in ticks])
    top.set_xticklabels([f"Ch {channel}" for channel in ticks])
    top.tick_params(colors=style.TEXT_SECONDARY, labelsize=9, length=0)
    top.spines["top"].set_visible(False)


def shade_edges(ax, result, measured):
    low, high = full_sensitivity_range(measured)
    freq = result["freq_mhz"]
    if freq[0] < low:
        ax.axvspan(BAND_MHZ[0], low, color=style.GRID, zorder=0)
    if freq[-1] > high:
        ax.axvspan(high, BAND_MHZ[1], color=style.GRID, zorder=0)


def draw_traces(ax, result, measured):
    freq = result["freq_mhz"]
    ax.plot(freq, to_db(result["peak"]), color=style.SERIES_Q, linewidth=0.8, label="Peak hold")
    ax.plot(freq, to_db(result["average"]), color=style.SERIES_I, linewidth=1.5, label="Average")
    shade_edges(ax, result, measured)
    ax.set_xlim(*BAND_MHZ)
    ax.set_ylabel("dBFS/MHz", color=style.TEXT_SECONDARY, fontsize=9)
    legend = ax.legend(loc="upper right", frameon=False, fontsize=9)
    for text in legend.get_texts():
        text.set_color(style.TEXT_SECONDARY)


def plot_panorama(panorama, result, title):
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap

    measured = panorama.measured_channels()
    rows = result["waterfall"]
    has_waterfall = len(rows) >= 4
    fig, axes = plt.subplots(2 if has_waterfall else 1, 1, figsize=(12, 8 if has_waterfall else 4.5),
                             height_ratios=[1, 1] if has_waterfall else [1],
                             sharex=True, constrained_layout=True, facecolor=style.SURFACE,
                             squeeze=False)
    axes = axes[:, 0]

    style.style_axes(axes[0])
    draw_traces(axes[0], result, measured)
    channel_axis(axes[0], measured)
    style.set_title(axes[0], "Spectrum (DC removed, shaded edges unreliable)")

    if has_waterfall:
        ax = axes[1]
        style.style_axes(ax)
        ax.grid(False)
        image = to_db(rows)
        colormap = LinearSegmentedColormap.from_list("power", style.WATERFALL_RAMP)
        colormap.set_bad(style.GRID)
        freq = result["freq_mhz"]
        low, high = np.nanpercentile(image, [5, 99.8])
        mesh = ax.imshow(image, aspect="auto", cmap=colormap, vmin=low, vmax=high,
                         interpolation="nearest", extent=[freq[0], freq[-1], len(rows), 0])
        colorbar = fig.colorbar(mesh, ax=ax, pad=0.01)
        colorbar.set_label("dBFS/MHz", color=style.TEXT_SECONDARY, fontsize=9)
        colorbar.ax.tick_params(colors=style.TEXT_SECONDARY, labelsize=9, length=0)
        colorbar.outline.set_visible(False)
        ax.set_ylabel("Pass", color=style.TEXT_SECONDARY, fontsize=9)
        style.set_title(ax, "Waterfall (strongest tuning at each frequency, per pass)")

    axes[-1].set_xlabel("Frequency (MHz)", color=style.TEXT_SECONDARY, fontsize=9)
    fig.suptitle(title, x=0.01, ha="left", color=style.TEXT, fontsize=12)
    return fig


def describe(metadata, panorama):
    channels = panorama.channels
    return (f"2.4 GHz band · channels {channels[0]}-{channels[-1]} · "
            f"{metadata['burst_samples']} samples per burst at "
            f"{metadata['sample_rate_hz'] / 1e6:g} MS/s · manual gain index "
            f"{metadata['gain_requested']}")


def save_outputs(args, panorama, result, metadata):
    print_summary(panorama, result)
    if args.npz:
        np.savez(args.npz, **result)
        print(f"saved {args.npz}")
    if args.save:
        import matplotlib
        matplotlib.use("Agg")
        plot_panorama(panorama, result, describe(metadata, panorama)).savefig(
            args.save, dpi=130, facecolor=style.SURFACE)
        print(f"saved {args.save}")


def check_metadata(metadata):
    if metadata["gain_requested"] == "agc":
        sys.exit("This capture used AGC, which scales each channel differently, so it "
                 "can't be stitched. Capture it with a manual --gain.")


def from_file(args):
    iq = np.load(args.capture)
    sidecar = args.capture.with_suffix(".json")
    if not sidecar.exists():
        sys.exit(f"{sidecar} is missing, so the channels of the bursts are unknown")
    metadata = json.loads(sidecar.read_text())
    check_metadata(metadata)
    if len(metadata["channels"]) < 2:
        sys.exit("That capture isn't a sweep. Capture it with pi_receiver.py --sweep.")
    if metadata.get("dc_removed"):
        print("note: this capture was saved with --remove-dc, which doesn't affect the panorama")

    panorama = Panorama(metadata["channels"], metadata["sample_rate_hz"], metadata["burst_samples"])
    samples = metadata["burst_samples"]
    for number, channel, start in zip(metadata["burst_numbers"], metadata["burst_channels"],
                                      metadata["burst_starts"]):
        panorama.add_iq(number, channel, iq[start:start + samples].astype(np.complex128))
    save_outputs(args, panorama, panorama.stitch(), metadata)


def from_stream(args):
    stats = pi_receiver.Stats()
    bursts = pi_receiver.pull_bursts(args.connect, stats)
    metadata = next(bursts)
    check_metadata(metadata)
    channels = metadata["channels"]
    if len(channels) < 2:
        sys.exit("That stream isn't a sweep. Start the Pi with --sweep.")
    print(f"connected: {pi_receiver.describe_channels(metadata)}")

    panorama = Panorama(channels, metadata["sample_rate_hz"], metadata["burst_samples"])
    live = LivePanorama(panorama, metadata) if args.live else None
    finished_pass = None
    first_pass = None
    try:
        for burst in bursts:
            panorama.add(burst.number, burst.channel, burst.words)
            index = burst.number // len(channels)
            first_pass = index if first_pass is None else first_pass
            if burst.channel != channels[-1] or index == finished_pass:
                continue
            finished_pass = index
            if live and not live.update():
                break
            if args.passes and index - first_pass + 1 >= args.passes:
                break
    except KeyboardInterrupt:
        pass
    if not panorama.measured_channels():
        sys.exit("FAIL: no bursts received")
    save_outputs(args, panorama, panorama.stitch(), metadata)
    if stats.crc_errors:
        sys.exit(f"FAIL: {stats.crc_errors} bursts were corrupted")


class LivePanorama:
    def __init__(self, panorama, metadata):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.panorama = panorama
        self.lines = None
        self.figure, self.axes = plt.subplots(figsize=(12, 5), constrained_layout=True,
                                              facecolor=style.SURFACE)
        style.style_axes(self.axes)
        self.figure.suptitle(describe(metadata, panorama), x=0.01, ha="left", fontsize=12)
        plt.ion()
        plt.show()

    def update(self):
        if len(self.panorama.measured_channels()) < len(self.panorama.channels):
            return True
        result = self.panorama.stitch()
        average, peak = to_db(result["average"]), to_db(result["peak"])
        if self.lines is None:
            measured = self.panorama.measured_channels()
            draw_traces(self.axes, result, measured)
            channel_axis(self.axes, measured)
            self.axes.set_xlabel("Frequency (MHz)", color=style.TEXT_SECONDARY, fontsize=9)
            self.lines = self.axes.get_lines()
        else:
            self.lines[0].set_ydata(peak)
            self.lines[1].set_ydata(average)
            self.axes.relim()
            self.axes.autoscale_view(scalex=False)
        self.plt.pause(0.001)
        return bool(self.plt.get_fignums())


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("capture", type=pathlib.Path, nargs="?",
                        help=".npy sweep saved by pi_receiver.py --sweep")
    parser.add_argument("--connect", metavar="HOST:PORT", help="pull a sweep from a running Pi")
    parser.add_argument("--passes", type=int, default=0, help="stop after this many passes")
    parser.add_argument("--live", action="store_true", help="show the panorama as it builds")
    parser.add_argument("--save", type=pathlib.Path, help="write an image of the panorama")
    parser.add_argument("--npz", type=pathlib.Path, help="write the panorama's data")
    args = parser.parse_args()

    if bool(args.capture) == bool(args.connect):
        parser.error("give either a capture file or --connect")
    if args.capture:
        from_file(args)
    else:
        from_stream(args)


if __name__ == "__main__":
    main()
