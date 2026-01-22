#!/usr/bin/env python3
import math
import threading
import time

import rospy
import cantools
import canlib.canlib as canlib

from adma_ros_driver_msgs.msg import AdmaTruth


def _to_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "on")
    return bool(value)


class CanTruthPublisher:
    def __init__(self):
        rospy.init_node("adma_truth_can", anonymous=False)

        dbc_file = rospy.get_param("~dbc_file", "")
        if not dbc_file:
            rospy.logfatal("Parameter ~dbc_file is required")
            raise RuntimeError("dbc_file not set")
        self.db = cantools.database.load_file(dbc_file)

        self.channel = int(rospy.get_param("~channel", 0))
        self.use_can_fd = bool(rospy.get_param("~can_fd", True))
        self.bit_rate = rospy.get_param("~bitrate", canlib.canBITRATE_500K)
        self.bit_rate_fd = rospy.get_param("~bitrate_fd", canlib.canFD_BITRATE_2M_80P)

        # Assumed CAN IDs and signal names (replace to match your DBC)
        self.speed_id = int(rospy.get_param("~speed_can_id", 0x100))
        self.yaw_rate_id = int(rospy.get_param("~yaw_rate_can_id", 0x101))
        self.lat_accel_id = int(rospy.get_param("~lat_accel_can_id", 0x102))
        self.long_accel_id = int(rospy.get_param("~long_accel_can_id", 0x103))

        self.speed_signal = rospy.get_param("~speed_signal", "veh_speed")
        self.yaw_rate_signal = rospy.get_param("~yaw_rate_signal", "yaw_rate")
        self.lat_accel_signal = rospy.get_param("~lat_accel_signal", "lat_accel")
        self.long_accel_signal = rospy.get_param("~long_accel_signal", "long_accel")

        self.publish_on_frame = _to_bool(rospy.get_param("~publish_on_frame", False))
        self.publish_hz = float(rospy.get_param("~publish_hz", 20.0))
        self.max_age = float(rospy.get_param("~max_age_sec", 0.2))
        self.require_all = bool(rospy.get_param("~require_all", True))
        self.use_freshness = _to_bool(rospy.get_param("~use_freshness", True))
        self.truth_log = _to_bool(rospy.get_param("~truth_log", False))
        self.truth_log_hz = float(rospy.get_param("~truth_log_hz", 1.0))
        if self.truth_log_hz <= 0.0:
            self.truth_log_hz = 1.0
        self.truth_log_period = 1.0 / self.truth_log_hz

        self.pub = rospy.Publisher("adma/truth", AdmaTruth, queue_size=10)

        self._lock = threading.Lock()
        self._last = {}
        self._last_stamp = {}

        self._open_can()
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        if self.publish_on_frame:
            rospy.loginfo("Publishing truth on each matching CAN frame")
        elif self.publish_hz > 0.0:
            period = 1.0 / self.publish_hz
            rospy.Timer(rospy.Duration(period), self._publish_cb)
        else:
            rospy.logwarn("publish_hz <= 0, will not publish truth topic")

    def _open_can(self):
        flags = canlib.canOPEN_CAN_FD if self.use_can_fd else 0
        self._can = canlib.openChannel(self.channel, flags)
        self._can.setBusParams(self.bit_rate)
        if self.use_can_fd:
            self._can.setBusParamsFd(self.bit_rate_fd)
        self._can.busOn()
        rospy.loginfo("CAN channel %d opened", self.channel)

    def _read_loop(self):
        need_ids = {self.speed_id, self.yaw_rate_id, self.lat_accel_id, self.long_accel_id}
        while not rospy.is_shutdown() and self._running:
            try:
                frame = self._can.read(timeout=100)#ms
            except canlib.CanNoMsg:
                continue
            except canlib.canError as exc:
                rospy.logerr("CAN error: %s", exc)
                time.sleep(0.5)
                continue

            msg_id = frame.id
            if msg_id not in need_ids:
                continue

            try:
                decoded = self.db.decode_message(
                    msg_id, frame.data, decode_choices=True, decode_containers=True
                )#false false
            except Exception:
                continue

            now = rospy.Time.now()
            with self._lock:
                if msg_id == self.speed_id and self.speed_signal in decoded:
                    self._last["actual_spd"] = float(decoded[self.speed_signal])
                    self._last_stamp["actual_spd"] = now
                elif msg_id == self.yaw_rate_id and self.yaw_rate_signal in decoded:
                    self._last["yaw_rate"] = float(decoded[self.yaw_rate_signal])
                    self._last_stamp["yaw_rate"] = now
                elif msg_id == self.lat_accel_id and self.lat_accel_signal in decoded:
                    self._last["lat_accel"] = float(decoded[self.lat_accel_signal])
                    self._last_stamp["lat_accel"] = now
                elif msg_id == self.long_accel_id and self.long_accel_signal in decoded:
                    self._last["long_accel"] = float(decoded[self.long_accel_signal])
                    self._last_stamp["long_accel"] = now

            if self.publish_on_frame:
                self._publish_now(now)

    def _publish_cb(self, _event):
        self._publish_now(rospy.Time.now())

    def _publish_now(self, now):
        with self._lock:
            def _fresh(key):
                if key not in self._last_stamp:
                    return False
                return (now - self._last_stamp[key]).toSec() <= self.max_age

            keys = ["actual_spd", "yaw_rate", "lat_accel", "long_accel"]
            fill = math.nan
            msg = AdmaTruth()
            msg.header.stamp = now
            if self.use_freshness:
                if self.require_all and not all(_fresh(k) for k in keys):
                    return
                msg.actual_spd = self._last.get("actual_spd", fill) if _fresh("actual_spd") else fill
                msg.yaw_rate = self._last.get("yaw_rate", fill) if _fresh("yaw_rate") else fill
                msg.lat_accel = self._last.get("lat_accel", fill) if _fresh("lat_accel") else fill
                msg.long_accel = self._last.get("long_accel", fill) if _fresh("long_accel") else fill
            else:
                msg.actual_spd = self._last.get("actual_spd", fill)
                msg.yaw_rate = self._last.get("yaw_rate", fill)
                msg.lat_accel = self._last.get("lat_accel", fill)
                msg.long_accel = self._last.get("long_accel", fill)
            msg.yaw_rate_sign = 1 if msg.yaw_rate < 0.0 else 0

        self.pub.publish(msg)
        if self.truth_log:
            rospy.loginfo_throttle(
                self.truth_log_period,
                "adma/truth spd_mps=%.3f yaw_rate_dps=%.3f lat_g=%.3f long_g=%.3f yaw_sign=%d"
                % (msg.actual_spd, msg.yaw_rate, msg.lat_accel, msg.long_accel, msg.yaw_rate_sign),
            )

    def shutdown(self):
        self._running = False
        self._thread.join(timeout=1.0)
        try:
            self._can.busOff()
            self._can.close()
        except Exception:
            pass


def main():
    node = None
    try:
        node = CanTruthPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.shutdown()


if __name__ == "__main__":
    main()
