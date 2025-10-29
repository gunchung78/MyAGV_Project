#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import fcntl
import struct
import time
import rospy
from geometry_msgs.msg import Twist
# ===== Linux joystick: struct js_event =====
# struct js_event {
#   __u32 time;   /* event timestamp in ms */
#   __s16 value;  /* value */
#   __u8  type;   /* event type */
#   __u8  number; /* axis/button number */
# };
# format => I h B B  (little-endian)
JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
# event types (plus 0x80 init flag)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x80
# ===== PS2 mapping (C 占쌘듸옙 占쌓댐옙占? =====
PS2_BUTTON_A       = 0x00
PS2_BUTTON_B       = 0x01
PS2_BUTTON_X       = 0x02
PS2_BUTTON_Y       = 0x03
PS2_BUTTON_L1      = 0x04
PS2_BUTTON_R1      = 0x05
PS2_BUTTON_SELECT  = 0x06
PS2_BUTTON_START   = 0x07
PS2_BUTTON_MODE    = 0x08
PS2_BUTTON_LO      = 0x09
PS2_BUTTON_RO      = 0x0a
PS2_AXIS_LX = 0x00
PS2_AXIS_LY = 0x01
PS2_AXIS_RX = 0x03
PS2_AXIS_RY = 0x04
PS2_AXIS_L2 = 0x02
PS2_AXIS_R2 = 0x05
PS2_AXIS_XX = 0x06
PS2_AXIS_YY = 0x07
PS2_AXIS_VAL_UP    = -32767
PS2_AXIS_VAL_DOWN  =  32767
PS2_AXIS_VAL_LEFT  = -32767
PS2_AXIS_VAL_RIGHT =  32767
PS2_AXIS_VAL_MID   =  0
# ===== ps2_map_t 占쏙옙 占쌔댐옙占싹댐옙 占쏙옙占쏙옙 占쏙옙占쏙옙占싱놂옙 =====
class PS2Map:
    __slots__ = (
        "time","a","b","x","y","l1","r1","select","start","mode","lo","ro",
        "lx","ly","rx","ry","l2","r2","xx","yy"
    )
    def __init__(self):
        self.time   = 0
        self.a      = 0
        self.b      = 0
        self.x      = 0
        self.y      = 0
        self.l1     = 0
        self.r1     = 0
        self.select = 0
        self.start  = 0
        self.mode   = 0
        self.lo     = 0
        self.ro     = 0
        self.lx     = 0
        self.ly     = 0
        self.rx     = 0
        self.ry     = 0
        self.l2     = 0
        self.r2     = 0
        self.xx     = 0
        self.yy     = 0
# ===== /dev/input/js0 reader =====
class JSDevice:
    def __init__(self, dev_path="/dev/input/js0"):
        self.dev_path = dev_path
        self.fd = None
    def open(self):
        self.fd = os.open(self.dev_path, os.O_RDONLY | os.O_NONBLOCK)
        # non-blocking read
    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
    def read_events(self):
        """占쏙옙占?占쏙옙占쏙옙占?占싱븝옙트占쏙옙 占싻억옙 [(time, type, number, value), ...] 占쏙옙 占쏙옙환"""
        events = []
        if self.fd is None:
            return events
        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE)
                if len(data) < JS_EVENT_SIZE:
                    break
                (t, value, etype, number) = struct.unpack(JS_EVENT_FORMAT, data)
                # init 占시뤄옙占쏙옙 占쏙옙占쏙옙
                etype_clean = etype & ~JS_EVENT_INIT
                events.append((t, etype_clean, number, value))
            except BlockingIOError:
                break
        return events
def main():
    rospy.init_node("ps2_joy_twist")
    dev_path = rospy.get_param("~device", "/dev/input/js0")
    pub_topic = rospy.get_param("~cmd_vel", "/cmd_vel")
    rate_hz = rospy.get_param("~rate", 10)
    pub = rospy.Publisher(pub_topic, Twist, queue_size=10)
    rate = rospy.Rate(rate_hz)
    js = JSDevice(dev_path)
    try:
        js.open()
    except OSError as e:
        rospy.logerr("Failed to open %s: %s", dev_path, e)
        return
    ps2 = PS2Map()
    # C 占쌘듸옙占?占쏙옙占쏙옙占쏙옙 占쏙옙칼占쏙옙 占쏙옙
    lin_step = 0.5
    ang_step = 0.5
    rospy.loginfo("PS2 joystick -> %s, reading %s", pub_topic, dev_path)
    try:
        while not rospy.is_shutdown():
            # 1) 占싱븝옙트 占쌥울옙
            for t, etype, number, value in js.read_events():
                ps2.time = t
                if etype == JS_EVENT_BUTTON:
                    if   number == PS2_BUTTON_A:      ps2.a = value
                    elif number == PS2_BUTTON_B:      ps2.b = value
                    elif number == PS2_BUTTON_X:      ps2.x = value
                    elif number == PS2_BUTTON_Y:      ps2.y = value
                    elif number == PS2_BUTTON_L1:     ps2.l1 = value
                    elif number == PS2_BUTTON_R1:     ps2.r1 = value
                    elif number == PS2_BUTTON_SELECT: ps2.select = value
                    elif number == PS2_BUTTON_START:  ps2.start = value
                    elif number == PS2_BUTTON_MODE:   ps2.mode = value
                    elif number == PS2_BUTTON_LO:     ps2.lo = value
                    elif number == PS2_BUTTON_RO:     ps2.ro = value
                elif etype == JS_EVENT_AXIS:
                    if   number == PS2_AXIS_LX: ps2.lx = value
                    elif number == PS2_AXIS_LY: ps2.ly = value
                    elif number == PS2_AXIS_RX: ps2.rx = value
                    elif number == PS2_AXIS_RY: ps2.ry = value
                    elif number == PS2_AXIS_L2: ps2.l2 = value
                    elif number == PS2_AXIS_R2: ps2.r2 = value
                    elif number == PS2_AXIS_XX: ps2.xx = value
                    elif number == PS2_AXIS_YY: ps2.yy = value
            # 2) C 占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙占쏙옙 占쏙옙칙占쏙옙占쏙옙 x,y,theta 占쏙옙占쏙옙
            x = 0.0
            y = 0.0
            theta = 0.0
            if ps2.x == 1:
                x = 0.0; y = 0.0; theta = 0.0
            else:
                if (ps2.xx == 0 and ps2.yy == 0 and ps2.l1 == 0 and ps2.r1 == 0):
                    x = 0.0; y = 0.0; theta = 0.0
                # YY: 占쏙옙/占쏙옙
                if ps2.yy == PS2_AXIS_VAL_UP:
                    x = lin_step
                if ps2.yy == PS2_AXIS_VAL_DOWN:
                    x = -lin_step
                if ps2.yy == PS2_AXIS_VAL_MID:
                    x = 0.0
                # XX: 占쏙옙/占쏙옙(占쏙옙占쏙옙 占싱듸옙) ? 占쏙옙占쏙옙 C占쏙옙 y占쏙옙 占쏙옙占?                if ps2.xx == PS2_AXIS_VAL_LEFT:
                    y = lin_step
                if ps2.xx == PS2_AXIS_VAL_RIGHT:
                    y = -lin_step
                if ps2.xx == PS2_AXIS_VAL_MID:
                    y = 0.0
                # L1/R1: 회占쏙옙
                if ps2.l1 == 1:
                    theta = ang_step
                if ps2.r1 == 1:
                    theta = -ang_step
                if (ps2.r1 == 0 and ps2.l1 == 0):
                    theta = 0.0
            # 3) 占쌜븝옙占쏙옙占쏙옙
            tw = Twist()
            tw.linear.x = x
            tw.linear.y = y
            tw.linear.z = 0.0
            tw.angular.x = 0.0
            tw.angular.y = 0.0
            tw.angular.z = theta
            pub.publish(tw)
            rate.sleep()
    finally:
        js.close()
if __name__ == "__main__":
    main()