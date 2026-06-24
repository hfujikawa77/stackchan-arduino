#!/usr/bin/env python3
"""Generate a beat map JSON for Stack-chan dance sync from an audio file.

The output is a list of beat timestamps (seconds from song start) plus an
"accent" flag (down-beats) and a "motion" hint (lr / ud). The browser player
(player.html) reads this map and POSTs each beat to Stack-chan's /beat endpoint,
phase-locked to Web Audio's playback clock.

Usage:
    python generate_beatmap.py song.mp3 -o billie_jean.json
    python generate_beatmap.py song.mp3 --bar 4 --auto-motion

See Issue #24 for the overall design.
"""

import argparse
import json
import os
import sys

import numpy as np


def assign_phrase_motions(beat_energy, beats_per_bar, phrase_bars):
    """Return a per-beat motion list that is constant within each phrase.

    Beats are grouped into phrases of `phrase_bars` bars. Each phrase gets a
    single motion based on its mean energy (louder phrases nod / 'ud', quieter
    ones side-swing / 'lr'), so the motion only changes at section-sized
    boundaries instead of flipping every beat.
    """
    n = len(beat_energy)
    phrase_len = max(1, beats_per_bar * phrase_bars)
    phrase_means = []
    for start in range(0, n, phrase_len):
        chunk = beat_energy[start:start + phrase_len]
        phrase_means.append(float(np.mean(chunk)) if len(chunk) else 0.0)
    threshold = float(np.median(phrase_means)) if phrase_means else 0.0

    motions = []
    for i in range(n):
        phrase_idx = i // phrase_len
        motions.append("ud" if phrase_means[phrase_idx] >= threshold else "lr")
    return motions


def build_beatmap(path, beats_per_bar, auto_motion, downbeat_offset, phrase_bars):
    try:
        import librosa
    except ImportError:
        sys.exit("librosa not installed. Run: pip install -r requirements.txt")

    # Load mono; librosa resamples to 22050 by default which is plenty for beats.
    y, sr = librosa.load(path, mono=True)
    duration = float(librosa.get_duration(y=y, sr=sr))

    tempo, beat_times = librosa.beat.beat_track(y=y, sr=sr, units="time")
    tempo = float(np.atleast_1d(tempo)[0])
    beat_times = np.asarray(beat_times, dtype=float)
    if beat_times.size == 0:
        sys.exit("No beats detected. Try a different track or check the file.")

    # Per-beat energy (RMS), then smoothed to phrase-level motion assignment.
    rms = librosa.feature.rms(y=y)[0]
    rms_times = librosa.times_like(rms, sr=sr)
    beat_rms = np.interp(beat_times, rms_times, rms)
    if auto_motion:
        motions = assign_phrase_motions(beat_rms, beats_per_bar, phrase_bars)
    else:
        motions = ["lr"] * len(beat_times)

    beats = []
    for i, t in enumerate(beat_times):
        # Down-beat approximation: every `beats_per_bar`-th beat is an accent.
        is_accent = ((i - downbeat_offset) % beats_per_bar) == 0
        beats.append({
            "t": round(float(t), 3),
            "accent": bool(is_accent),
            "motion": motions[i],
        })

    return {
        "title": os.path.splitext(os.path.basename(path))[0],
        "bpm": int(round(tempo)),
        "duration": round(duration, 2),
        "beats_per_bar": beats_per_bar,
        "phrase_bars": phrase_bars,
        "beats": beats,
    }


def main():
    ap = argparse.ArgumentParser(description="Generate Stack-chan dance beat map from audio.")
    ap.add_argument("audio", help="input audio file (mp3/wav/m4a/...)")
    ap.add_argument("-o", "--out", help="output JSON path (default: <audio>.json)")
    ap.add_argument("--bar", type=int, default=4, help="beats per bar for accent (default 4)")
    ap.add_argument("--downbeat-offset", type=int, default=0,
                    help="index of the first down-beat (default 0)")
    ap.add_argument("--auto-motion", action="store_true",
                    help="nod (ud) on high-energy phrases, side-swing (lr) otherwise")
    ap.add_argument("--phrase-bars", type=int, default=8,
                    help="phrase length in bars; motion is constant within a phrase (default 8)")
    args = ap.parse_args()

    if not os.path.isfile(args.audio):
        sys.exit(f"File not found: {args.audio}")

    bmap = build_beatmap(args.audio, args.bar, args.auto_motion,
                         args.downbeat_offset, args.phrase_bars)
    out_path = args.out or (os.path.splitext(args.audio)[0] + ".json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(bmap, f, ensure_ascii=False, indent=0)

    print(f"Wrote {out_path}")
    print(f"  title: {bmap['title']}")
    print(f"  bpm:   {bmap['bpm']}")
    print(f"  beats: {len(bmap['beats'])}  duration: {bmap['duration']}s")


if __name__ == "__main__":
    main()
