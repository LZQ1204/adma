#!/usr/bin/env python3
import argparse
import bisect
import csv
import math
import sys

import rosbag


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare ADMA truth vs XCP values from a single rosbag."
    )
    parser.add_argument("--bag", required=True, help="Path to rosbag")
    parser.add_argument("--adma-topic", required=True, help="ADMA topic name")
    parser.add_argument(
        "--adma-field",
        required=True,
        help="ADMA field path(s), comma-separated (e.g., actual_spd,yaw_rate)",
    )
    parser.add_argument("--xcp-topic", required=True, help="XCP topic name")
    parser.add_argument(
        "--xcp-field",
        required=True,
        help="XCP field path(s), comma-separated (e.g., speed,yaw_rate)",
    )
    parser.add_argument(
        "--time-source",
        choices=["header", "bag", "header_or_bag"],
        default="header_or_bag",
        help="Timestamp source for messages",
    )
    parser.add_argument(
        "--method",
        choices=["nearest", "linear"],
        default="nearest",
        help="Time alignment method",
    )
    parser.add_argument(
        "--max-dt",
        type=float,
        default=None,
        help="Max allowed time difference (sec). Skip pairs larger than this.",
    )
    parser.add_argument(
        "--adma-scale",
        default="1.0",
        help="Scale for ADMA value(s), single or comma-separated",
    )
    parser.add_argument(
        "--adma-offset",
        default="0.0",
        help="Offset for ADMA value(s), single or comma-separated",
    )
    parser.add_argument(
        "--xcp-scale",
        default="1.0",
        help="Scale for XCP value(s), single or comma-separated",
    )
    parser.add_argument(
        "--xcp-offset",
        default="0.0",
        help="Offset for XCP value(s), single or comma-separated",
    )
    parser.add_argument(
        "--output-csv",
        default="",
        help="Optional CSV output path",
    )
    return parser.parse_args()


def split_list(value):
    items = [item.strip() for item in str(value).split(",")]
    return [item for item in items if item]


def parse_float_list(value, count, name):
    items = split_list(value)
    if not items:
        raise ValueError("%s must not be empty" % name)
    if len(items) == 1:
        return [float(items[0])] * count
    if len(items) != count:
        raise ValueError("%s count (%d) does not match fields (%d)" % (name, len(items), count))
    return [float(item) for item in items]


def get_msg_time(msg, bag_time, source):
    if source == "bag":
        return bag_time.to_sec()
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        stamp = msg.header.stamp
        if stamp is not None and (stamp.secs != 0 or stamp.nsecs != 0):
            return stamp.to_sec()
    if source == "header":
        return None
    return bag_time.to_sec()


def get_field(msg, path):
    cur = msg
    for part in path.split("."):
        if "[" in part and part.endswith("]"):
            name, index = part[:-1].split("[", 1)
            cur = getattr(cur, name)
            cur = cur[int(index)]
        else:
            cur = getattr(cur, part)
    return cur


def apply_scale(value, scale, offset):
    return value * scale + offset


def align_nearest(adma_list, xcp_list, max_dt):
    adma_times = [t for t, _ in adma_list]
    for t_xcp, v_xcp in xcp_list:
        idx = bisect.bisect_left(adma_times, t_xcp)
        candidates = []
        if idx > 0:
            candidates.append(adma_list[idx - 1])
        if idx < len(adma_list):
            candidates.append(adma_list[idx])
        if not candidates:
            continue
        t_adma, v_adma = min(candidates, key=lambda item: abs(item[0] - t_xcp))
        dt = abs(t_xcp - t_adma)
        if max_dt is not None and dt > max_dt:
            continue
        yield t_xcp, t_adma, v_adma, v_xcp, dt


def align_linear(adma_list, xcp_list, max_dt):
    adma_times = [t for t, _ in adma_list]
    for t_xcp, v_xcp in xcp_list:
        idx = bisect.bisect_right(adma_times, t_xcp)
        if idx == 0 or idx >= len(adma_list):
            continue
        t0, v0 = adma_list[idx - 1]
        t1, v1 = adma_list[idx]
        if t1 == t0:
            continue
        if max_dt is not None and min(abs(t_xcp - t0), abs(t_xcp - t1)) > max_dt:
            continue
        ratio = (t_xcp - t0) / (t1 - t0)
        v_adma = v0 + (v1 - v0) * ratio
        dt = abs(t_xcp - t0) if ratio < 0.5 else abs(t_xcp - t1)
        yield t_xcp, t_xcp, v_adma, v_xcp, dt


def main():
    args = parse_args()

    adma_fields = split_list(args.adma_field)
    xcp_fields = split_list(args.xcp_field)
    if len(adma_fields) != len(xcp_fields):
        print("Field count mismatch: adma=%d xcp=%d" % (len(adma_fields), len(xcp_fields)))
        return 1

    try:
        adma_scales = parse_float_list(args.adma_scale, len(adma_fields), "adma-scale")
        adma_offsets = parse_float_list(args.adma_offset, len(adma_fields), "adma-offset")
        xcp_scales = parse_float_list(args.xcp_scale, len(xcp_fields), "xcp-scale")
        xcp_offsets = parse_float_list(args.xcp_offset, len(xcp_fields), "xcp-offset")
    except ValueError as exc:
        print(str(exc))
        return 1

    adma_vals = {field: [] for field in adma_fields}
    xcp_vals = {field: [] for field in xcp_fields}

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[args.adma_topic, args.xcp_topic]):
            ts = get_msg_time(msg, t, args.time_source)
            if ts is None:
                continue
            if topic == args.adma_topic:
                for i, field in enumerate(adma_fields):
                    try:
                        val = float(get_field(msg, field))
                    except Exception:
                        continue
                    val = apply_scale(val, adma_scales[i], adma_offsets[i])
                    adma_vals[field].append((ts, val))
            else:
                for i, field in enumerate(xcp_fields):
                    try:
                        val = float(get_field(msg, field))
                    except Exception:
                        continue
                    val = apply_scale(val, xcp_scales[i], xcp_offsets[i])
                    xcp_vals[field].append((ts, val))

    any_pairs = 0
    csv_rows = []
    for idx, (adma_field, xcp_field) in enumerate(zip(adma_fields, xcp_fields)):
        adma_list = adma_vals.get(adma_field, [])
        xcp_list = xcp_vals.get(xcp_field, [])
        if not adma_list or not xcp_list:
            print("No data for field pair: %s vs %s" % (adma_field, xcp_field))
            continue

        adma_list.sort(key=lambda x: x[0])
        xcp_list.sort(key=lambda x: x[0])

        if args.method == "nearest":
            pairs = list(align_nearest(adma_list, xcp_list, args.max_dt))
        else:
            pairs = list(align_linear(adma_list, xcp_list, args.max_dt))

        if not pairs:
            print("No matched pairs after alignment for: %s vs %s" % (adma_field, xcp_field))
            continue

        any_pairs += len(pairs)
        errors = [xcp - adma for _, _, adma, xcp, _ in pairs]
        abs_errors = [abs(e) for e in errors]
        mae = sum(abs_errors) / len(abs_errors)
        rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
        mean_err = sum(errors) / len(errors)
        max_err = max(abs_errors)

        print("Field: %s vs %s" % (adma_field, xcp_field))
        print("  ADMA samples: %d" % len(adma_list))
        print("  XCP samples: %d" % len(xcp_list))
        print("  Matched pairs: %d" % len(pairs))
        print("  Mean error: %.6f" % mean_err)
        print("  MAE: %.6f" % mae)
        print("  RMSE: %.6f" % rmse)
        print("  Max abs error: %.6f" % max_err)

        for t_xcp, t_adma, v_adma, v_xcp, dt in pairs:
            csv_rows.append([adma_field, xcp_field, t_xcp, t_adma, v_adma, v_xcp, v_xcp - v_adma, dt])

    if any_pairs == 0:
        print("No matched pairs for any field.")
        return 1

    if args.output_csv:
        with open(args.output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["adma_field", "xcp_field", "t_xcp", "t_adma", "adma", "xcp", "error", "dt"])
            for row in csv_rows:
                writer.writerow(row)

        print("Wrote CSV: %s" % args.output_csv)

    return 0


if __name__ == "__main__":
    sys.exit(main())
