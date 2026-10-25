"""
sam2_track_positions.py
==========================
STAGE 1 of 2 -- this is the only script that needs SAM2/GPU/torch. It does
tracking only: no theta, no time_sec, no mm conversion. Those are derived
afterwards by finalize_theta_and_table.py, which needs nothing but plain
numpy and can run on any machine (see that file's docstring for why).

Also writes an annotated MP4 (each robot's mask + ID overlaid in a
distinct color) so the tracking can be verified by eye before trusting
the numbers. Its filename is derived from RUN_NUM, same as every
other output here.

WHAT IT NEEDS ALONGSIDE IT:
    - This script
    - A frames/<RUN_NUM>/ folder of extracted JPEGs for the video to process
      (ffmpeg -i <video>.mkv -q:v 2 -start_number 0 frames/<RUN_NUM>/%05d.jpg)
No trained model, no other project file needed -- this is the whole GPU side.

SET UP THE ENVIRONMENT:
    python3 -m venv sam2kb
    source sam2kb/bin/activate 

WHAT TO SET PER RUN:
    N       -- number of robots in this video
    RUN_NUM -- which recording, 1-10 -- selects frames/<RUN_NUM> as input
Everything else (input folder, output filenames) is derived from those two
automatically.

RUN THE SCRIPT:
    python sam2_track_positions.py

WHAT IT DOES:
    1. Opens frame 0 and asks you to click each robot's centre (in any
       order -- they get object IDs 1..N in click order). Saves the clicks
       to a small log file for the record.
    2. Runs SAM2 across the ENTIRE video in one pass.
    3. Writes three files:
         - the annotated MP4 (visual check)
         - the raw per-frame CSV of pixel positions (x_px, y_px; NaN on
           any frame SAM2 lost the mask)
         - a one-line metadata file (frame size, frame count) that stage 2
           needs but doesn't have to re-derive by opening the video again
    4. Prints a chassis-diameter scale sanity check against MM_PER_PX up
       front -- catches a changed camera setup before a long run -- and a
       per-robot tracked-frame summary at the end, so a robot that lost
       its mask partway through is obvious without scrubbing the video.

OUTPUT (raw CSV, pixel space only):
    obj_id, frame_idx, x_px, y_px
"""

import csv
import math
import os
import time

import colorsys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

import torch

from sam2.sam2_video_predictor import SAM2VideoPredictor

# ═══════════════════════════════════════════════════════════════════════
# SET THESE TWO PER RUN
# ═══════════════════════════════════════════════════════════════════════
N = 25            # number of robots in this video
RUN_NUM = 1        # 1-10 -- selects frames/<RUN_NUM> as input

# ── Derived automatically from N and RUN_NUM above -- no need to edit ───
VIDEO_DIR = f"frames/{RUN_NUM}"
POINTS_LOG = f"points_n{N}_run{RUN_NUM}.txt"
OUT_RAW_CSV = f"raw_positions_n{N}_run{RUN_NUM}.csv"
OUT_META = f"meta_n{N}_run{RUN_NUM}.txt"
OUT_VIDEO = f"tracked_n{N}_run{RUN_NUM}.mp4"

# ── Fixed settings -- check once for your camera/arena setup, then leave ─
CKPT_NAME = "facebook/sam2.1-hiera-large"  # GPU can afford the best checkpoint for
                                             # quality; drop to "-small" if too slow
FPS = 30                   # match the frame rate frames/<RUN_NUM> was extracted at --
                           # used here only for the output video's playback speed
MM_PER_PX = 1 / 1.1960     # only used for the printed scale sanity-check below --
                           # the actual mm conversion happens in stage 2, so this
                           # doesn't need to be perfectly right to run this script
ROBOT_DIAM_MM = 33.0
MASK_ALPHA = 0.45          # video overlay blend strength
# ═══════════════════════════════════════════════════════════════════════


def pick_points(frame0_img, n):
    """Click n robot centres on frame 0. Returns {obj_id: (x, y)}, obj_id
    starting at 1 in click order."""
    points = {}

    def onclick(event):
        if event.xdata is None or len(points) >= n:
            return
        obj_id = len(points) + 1
        points[obj_id] = (event.xdata, event.ydata)
        plt.plot(event.xdata, event.ydata, "r+", markersize=14, mew=2)
        plt.annotate(str(obj_id), (event.xdata, event.ydata), color="yellow",
                     xytext=(6, 6), textcoords="offset points")
        plt.draw()
        print(f"  robot {obj_id}: ({event.xdata:.0f}, {event.ydata:.0f})")
        if len(points) == n:
            print("All points clicked -- close the window to continue.")

    fig, ax = plt.subplots(figsize=(12, 8))
    ax.imshow(frame0_img)
    ax.set_title(f"Click all {n} robot centres, in any order. Close when done.")
    fig.canvas.mpl_connect("button_press_event", onclick)
    plt.show()

    if len(points) != n:
        raise SystemExit(f"Expected {n} clicks, got {len(points)} -- rerun.")
    return points


def generate_colors(n):
    """n evenly-spaced hues around the color wheel, as BGR tuples for cv2."""
    colors = []
    for i in range(n):
        r, g, b = colorsys.hsv_to_rgb(i / n, 0.85, 0.95)
        colors.append((int(b * 255), int(g * 255), int(r * 255)))
    return colors


def main():
    frame_files = sorted(os.listdir(VIDEO_DIR))
    total_frames = len(frame_files)
    frame0 = np.array(Image.open(os.path.join(VIDEO_DIR, frame_files[0])).convert("RGB"))
    video_h, video_w = frame0.shape[:2]

    print(f"N={N} robots, run {RUN_NUM}, video: {VIDEO_DIR} "
          f"({total_frames} frames, {video_w}x{video_h})")

    device = "cuda" if torch.cuda.is_available() else "cpu"
    if device == "cpu":
        print("WARNING: no GPU detected. This script processes the whole video in "
              "one pass with no memory safeguards -- on CPU that risks the same "
              "out-of-memory crash a long single-pass run hit earlier. Use a "
              "GPU machine, or a chunked CPU-safe version instead.")

    points = pick_points(frame0, N)
    with open(POINTS_LOG, "w") as f:
        for obj_id, (x, y) in points.items():
            f.write(f"{obj_id}: ({x:.0f}, {y:.0f})\n")
    print(f"Saved click points to {POINTS_LOG}")

    colors = generate_colors(N)
    video_writer = cv2.VideoWriter(
        OUT_VIDEO, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (video_w, video_h))

    predictor = SAM2VideoPredictor.from_pretrained(CKPT_NAME, device=device)
    state = predictor.init_state(video_path=VIDEO_DIR)

    autocast_dtype = torch.bfloat16 if device == "cuda" else torch.float32
    raw_rows = []  # (obj_id, frame_idx, x_px, y_px)
    t_start = time.time()

    with torch.inference_mode(), torch.autocast(device, dtype=autocast_dtype):
        frame0_radii = {}
        for obj_id, (x, y) in points.items():
            _, out_obj_ids, out_mask_logits = predictor.add_new_points_or_box(
                state, frame_idx=0, obj_id=obj_id,
                points=np.array([[x, y]], dtype=np.float32),
                labels=np.array([1], dtype=np.int32),
            )
            idx = list(out_obj_ids).index(obj_id)
            mask0 = (out_mask_logits[idx][0] > 0.0).cpu().numpy()
            if mask0.sum() > 10:
                frame0_radii[obj_id] = math.sqrt(mask0.sum() / math.pi)

        # Scale sanity check -- catches a changed camera setup (zoom, distance)
        # before it wastes a long run. Purely informational: this script
        # itself never converts anything to mm.
        if frame0_radii:
            median_diam_px = 2 * np.median(list(frame0_radii.values()))
            implied_mm = median_diam_px * MM_PER_PX
            err = abs(implied_mm - ROBOT_DIAM_MM) / ROBOT_DIAM_MM
            print(f"Scale check: measured chassis diameter {median_diam_px:.1f}px "
                  f"-> {implied_mm:.1f}mm (expected {ROBOT_DIAM_MM}mm, {100*err:.1f}% off)")
            if err > 0.10:
                print(f"WARNING: more than 10% off -- MM_PER_PX may be wrong for this "
                      f"video. Chassis alone implies MM_PER_PX = "
                      f"{ROBOT_DIAM_MM/median_diam_px:.4f}")

        print("Tracking...")
        for frame_idx, obj_ids, mask_logits in predictor.propagate_in_video(
                state, max_frame_num_to_track=total_frames):

            frame_bgr = cv2.imread(os.path.join(VIDEO_DIR, frame_files[frame_idx]))

            for obj_id, logits in zip(obj_ids, mask_logits):
                mask = (logits[0] > 0.0).cpu().numpy()
                ys, xs = np.where(mask)
                if len(xs) == 0:
                    raw_rows.append((obj_id, frame_idx, float("nan"), float("nan")))
                    continue
                cx, cy = float(xs.mean()), float(ys.mean())
                raw_rows.append((obj_id, frame_idx, cx, cy))

                color = colors[(obj_id - 1) % len(colors)]
                frame_bgr[mask] = (
                    (1 - MASK_ALPHA) * frame_bgr[mask].astype(float)
                    + MASK_ALPHA * np.array(color)
                ).astype(np.uint8)
                cv2.circle(frame_bgr, (int(round(cx)), int(round(cy))), 3, color, -1)
                cv2.putText(frame_bgr, str(obj_id), (int(cx) + 6, int(cy) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

            video_writer.write(frame_bgr)

            if frame_idx > 0 and frame_idx % 200 == 0:
                elapsed = time.time() - t_start
                rate = elapsed / frame_idx
                remaining_min = rate * (total_frames - frame_idx) / 60
                print(f"  frame {frame_idx}/{total_frames}  "
                      f"({rate:.2f}s/frame, ~{remaining_min:.1f} min left)")

    video_writer.release()
    print(f"\nSaved {OUT_VIDEO}")

    with open(OUT_RAW_CSV, "w", newline="") as f:
        w_ = csv.writer(f)
        w_.writerow(["obj_id", "frame_idx", "x_px", "y_px"])
        w_.writerows(raw_rows)
    print(f"Saved {OUT_RAW_CSV}")

    with open(OUT_META, "w") as f:
        f.write(f"video_w={video_w}\n")
        f.write(f"video_h={video_h}\n")
        f.write(f"total_frames={total_frames}\n")
        f.write(f"n_robots={N}\n")
        f.write(f"run_num={RUN_NUM}\n")
    print(f"Saved {OUT_META}")

    # ── Diagnostics: per-robot tracked-frame summary ────────────────────────
    print("\nPer-robot summary:")
    by_obj = {}
    for obj_id, frame_idx, x, y in raw_rows:
        by_obj.setdefault(obj_id, []).append(x)
    for obj_id, xs in by_obj.items():
        xs = np.array(xs)
        n_valid, n_total = int((~np.isnan(xs)).sum()), len(xs)
        if n_valid == 0:
            print(f"  robot {obj_id}: NO valid mask in any frame -- check its "
                  f"frame-0 click point")
        else:
            pct = 100 * n_valid / n_total
            flag = "" if pct > 95 else "  <-- lost the mask partway, check the video"
            print(f"  robot {obj_id}: {n_valid}/{n_total} frames tracked ({pct:.0f}%){flag}")

    print(f"\nNext: run finalize_theta_and_table.py (no GPU needed) on {OUT_RAW_CSV} "
          f"and {OUT_META} to get theta, time_sec, and the final Kilombo-format table.")


if __name__ == "__main__":
    main()
