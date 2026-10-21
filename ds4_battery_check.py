#!/usr/bin/env python3
"""Read the battery level of a DualShock 4 controller from /dev/hidraw*.

Works without the hid-sony kernel driver. Run with sudo:
    sudo python3 ds4_battery_check.py
"""
import fcntl
import glob
import os
import select
import sys

SONY_VID = 0x054C
DS4_PIDS = (0x05C4, 0x09CC)  # DualShock 4 v1 and v2
BUS_USB, BUS_BT = 0x03, 0x05


def find_ds4():
    for path in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        try:
            with open(os.path.join(path, "device", "uevent")) as f:
                lines = f.read().splitlines()
        except OSError:
            continue
        for line in lines:
            if line.startswith("HID_ID="):
                bus, vid, pid = (int(x, 16) for x in line[7:].split(":"))
                if vid == SONY_VID and pid in DS4_PIDS:
                    return "/dev/" + os.path.basename(path), bus
    return None, None


def hidiocgfeature(length):
    # _IOC(_IOC_READ | _IOC_WRITE, 'H', 0x07, length)
    return 0xC0000000 | (length << 16) | (ord("H") << 8) | 0x07


def parse_battery(byte):
    cable = (byte >> 4) & 0x01
    level = byte & 0x0F
    if not cable:
        return min(level * 10 + 5, 100), "Discharging"
    if level < 10:
        return level * 10 + 5, "Charging"
    if level == 10:
        return 100, "Charging"
    if level == 11:
        return 100, "Full"
    return None, "Unknown (charging error)"


def main():
    dev, bus = find_ds4()
    if dev is None:
        sys.exit("No DualShock 4 found. Is it connected?")

    try:
        fd = os.open(dev, os.O_RDWR)
    except PermissionError:
        sys.exit(f"Permission denied on {dev}. Run with sudo.")

    bluetooth = bus == BUS_BT
    if bluetooth:
        # Over Bluetooth the DS4 sends short reports with no battery info
        # until feature report 0x02 is requested; then it sends full 0x11 reports.
        buf = bytearray(37)
        buf[0] = 0x02
        try:
            fcntl.ioctl(fd, hidiocgfeature(len(buf)), buf)
        except OSError as e:
            print(f"Warning: could not switch to full reports: {e}")

    report_id, offset = (0x11, 32) if bluetooth else (0x01, 30)

    for _ in range(500):
        ready, _, _ = select.select([fd], [], [], 2.0)
        if not ready:
            sys.exit("Timed out waiting for data from the controller.")
        data = os.read(fd, 128)
        if data and data[0] == report_id and len(data) > offset:
            percent, status = parse_battery(data[offset])
            conn = "Bluetooth" if bluetooth else "USB"
            if percent is None:
                print(f"Battery: {status}  ({conn}, {dev})")
            else:
                print(f"Battery: {percent}%  {status}  ({conn}, {dev})")
            os.close(fd)
            return

    os.close(fd)
    sys.exit("Got data from the controller but no battery report.")


if __name__ == "__main__":
    main()