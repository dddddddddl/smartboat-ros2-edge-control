import argparse
import multiprocessing as mp
import threading
from ctypes import *
import time
import math
import os
import platform
import errno
import socket
import struct
import subprocess
import sys


DEV_TYPE_OPEN = 4
DEV_TYPE_ACTIVE = 4
DEFAULT_BAUD_RATE = 0x0012000B  # BOT_500K in target/iTekon-usb.h
ITEK_USB_VENDOR = "0471"
ITEK_USB_PRODUCT = "1200"
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1fffffff
CAN_SFF_MASK = 0x000007ff
RADAR_CAN_ID = 0x60B
DEFAULT_POINT_CLOUD_SIZE = 720
DEFAULT_POINT_CLOUD_MAX_DISTANCE = 20.0
DEFAULT_POINT_CLOUD_MIN_DISTANCE = 1.0
DEFAULT_RADAR_FOV_DEG = 112.0
DEFAULT_CAN_BITRATE = 500000


class RadarConnectionError(RuntimeError):
    pass


def parse_radar_object_general(frame_id, data):
    if frame_id != RADAR_CAN_ID or len(data) < 8:
        return None

    # SR73F Obj_1_General(0x60B), same layout as common ARS-style object
    # CAN messages: longitudinal/lateral position, relative velocity, dynamic
    # property and RCS. Units and offsets come from the vendor manual.
    object_id = data[0]
    dist_long_raw = (data[1] << 5) | ((data[2] >> 3) & 0x1f)
    dist_lat_raw = ((data[2] & 0x07) << 8) | data[3]
    vrel_long_raw = (data[4] << 2) | ((data[5] >> 6) & 0x03)
    vrel_lat_raw = ((data[5] & 0x3f) << 3) | ((data[6] >> 5) & 0x07)
    dyn_prop = data[6] & 0x07
    rcs = data[7] * 0.5 - 64.0

    dist_long = dist_long_raw * 0.2 - 500.0
    dist_lat = dist_lat_raw * 0.2 - 204.6
    vrel_long = vrel_long_raw * 0.25 - 128.0
    vrel_lat = vrel_lat_raw * 0.25 - 64.0
    distance = math.sqrt(dist_long * dist_long + dist_lat * dist_lat)
    bearing = math.degrees(math.atan2(dist_lat, dist_long))
    angle = int(round(bearing))
    if angle < 0:
        angle += 360
    return {
        "id": int(object_id),
        "dist_long": float(dist_long),
        "dist_lat": float(dist_lat),
        "vrel_long": float(vrel_long),
        "vrel_lat": float(vrel_lat),
        "dyn_prop": int(dyn_prop),
        "rcs": float(rcs),
        "distance": float(distance),
        "angle_deg": int(angle),
        "bearing_deg": float(bearing),
        "raw": bytes(data[:8]).hex(" "),
    }


def _object_passes_filter(obj, min_distance=0.0, front_only=False, min_rcs=-64.0, fov_deg=None):
    if obj is None:
        return False
    if obj["distance"] < min_distance:
        return False
    if front_only and obj["dist_long"] <= 0:
        return False
    if fov_deg is not None and abs(obj["bearing_deg"]) > fov_deg * 0.5:
        return False
    if obj["rcs"] < min_rcs:
        return False
    return True


def _update_radar_data_from_object(shared_radar_data, obj):
    angle = obj["angle_deg"]
    distance = round(obj["distance"], 2)
    if shared_radar_data[angle] == 0 or shared_radar_data[angle] > distance:
        shared_radar_data[angle] = distance


def _update_radar_data_from_frame(
    shared_radar_data,
    shared_radar_objects,
    frame_id,
    data,
    min_distance=0.0,
    front_only=False,
    min_rcs=-64.0,
    fov_deg=None,
):
    obj = parse_radar_object_general(frame_id, data)
    if not _object_passes_filter(
        obj,
        min_distance=min_distance,
        front_only=front_only,
        min_rcs=min_rcs,
        fov_deg=fov_deg,
    ):
        return False
    shared_radar_objects.append(obj)
    _update_radar_data_from_object(shared_radar_data, obj)
    return True


def dedupe_radar_objects(radar_objects):
    latest_by_id = {}
    for obj in radar_objects:
        latest_by_id[obj["id"]] = obj
    return list(latest_by_id.values())


def radar_points_from_objects(radar_objects):
    radar_objects = dedupe_radar_objects(radar_objects)
    return [
        {
            "angle_deg": obj["angle_deg"],
            "distance": obj["distance"],
            "x": obj["dist_lat"],
            "y": obj["dist_long"],
            "z": 0.0,
            "vrel_long": obj["vrel_long"],
            "vrel_lat": obj["vrel_lat"],
            "dyn_prop": obj["dyn_prop"],
            "rcs": obj["rcs"],
            "id": obj["id"],
        }
        for obj in radar_objects
    ]


def radar_points_from_scan(radar_data):
    points = []
    for angle, distance in enumerate(radar_data):
        if distance <= 0:
            continue
        theta = math.radians(angle)
        x = distance * math.sin(theta)
        y = distance * math.cos(theta)
        points.append({
            "angle_deg": angle,
            "distance": float(distance),
            "x": float(x),
            "y": float(y),
            "z": 0.0,
        })
    return points


def _draw_colorbar(canvas, cv2, np, x, y, width, height, max_distance):
    gradient = np.linspace(255, 0, height, dtype=np.uint8).reshape(height, 1)
    colorbar = cv2.applyColorMap(gradient, cv2.COLORMAP_TURBO)
    canvas[y:y + height, x:x + width] = colorbar
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (210, 210, 210), 1, cv2.LINE_AA)
    for value in (0, max_distance * 0.25, max_distance * 0.5, max_distance * 0.75, max_distance):
        norm = 1.0 - value / max_distance
        tick_y = int(round(y + norm * height))
        cv2.line(canvas, (x + width, tick_y), (x + width + 7, tick_y), (235, 235, 235), 1, cv2.LINE_AA)
        cv2.putText(
            canvas,
            f"{value:.0f}m",
            (x + width + 10, tick_y + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )


def render_radar_object_cloud(
    radar_objects,
    size=DEFAULT_POINT_CLOUD_SIZE,
    max_distance=DEFAULT_POINT_CLOUD_MAX_DISTANCE,
    fov_deg=DEFAULT_RADAR_FOV_DEG,
):
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("radar visualization requires cv2 and numpy") from exc

    radar_objects = dedupe_radar_objects(radar_objects)
    points = radar_points_from_objects(radar_objects)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    canvas[:] = (18, 20, 24)
    max_distance = max(1.0, float(max_distance))
    fov_deg = max(10.0, min(170.0, float(fov_deg)))
    half_fov = fov_deg * 0.5
    margin = 54
    colorbar_left = size - 82
    origin = (size // 2 - 36, size - 62)
    radius_px = min(origin[0] - margin, colorbar_left - origin[0] - margin, size - 126)
    radius_px = max(80, radius_px)
    max_lat = max_distance * math.sin(math.radians(half_fov))

    def color_for_distance(distance):
        norm = np.uint8([[max(0, min(255, int(255 - distance / max_distance * 255)))]])
        return tuple(int(v) for v in cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)[0, 0])

    def project(x, y, z=0.0):
        px = int(round(origin[0] + x / max_lat * radius_px))
        py = int(round(origin[1] - y / max_distance * radius_px))
        return px, py

    def polar_project(distance, angle_deg):
        angle_rad = math.radians(angle_deg)
        return project(distance * math.sin(angle_rad), distance * math.cos(angle_rad))

    grid_color = (58, 66, 76)
    axis_color = (125, 150, 176)
    sector_color = (82, 96, 110)
    tick_step = max(1, int(max_distance // 5))
    for value in range(tick_step, int(max_distance) + 1, tick_step):
        arc = []
        for angle in range(int(-half_fov), int(half_fov) + 1, 2):
            arc.append(polar_project(value, angle))
        if len(arc) >= 2:
            cv2.polylines(canvas, [np.array(arc, dtype=np.int32)], False, grid_color, 1, cv2.LINE_AA)
        label = polar_project(value, half_fov)
        cv2.putText(canvas, f"{value}m", (label[0] + 6, label[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 178, 188), 1, cv2.LINE_AA)

    left_edge = polar_project(max_distance, -half_fov)
    right_edge = polar_project(max_distance, half_fov)
    cv2.line(canvas, origin, left_edge, sector_color, 1, cv2.LINE_AA)
    cv2.line(canvas, origin, right_edge, sector_color, 1, cv2.LINE_AA)
    cv2.line(canvas, origin, polar_project(max_distance, 0), axis_color, 2, cv2.LINE_AA)
    cv2.circle(canvas, origin, 5, (210, 220, 230), -1, cv2.LINE_AA)
    cv2.putText(canvas, f"-{half_fov:.0f}deg", (left_edge[0] - 42, left_edge[1] + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 170, 180), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"+{half_fov:.0f}deg", (right_edge[0] + 4, right_edge[1] + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 170, 180), 1, cv2.LINE_AA)

    for point in sorted(points, key=lambda item: item["distance"], reverse=True):
        x = point["x"]
        y = point["y"]
        if y < 0 or abs(point["angle_deg"] if point["angle_deg"] <= 180 else point["angle_deg"] - 360) > half_fov or y > max_distance:
            continue
        px, py = project(x, y)
        color = color_for_distance(point["distance"])
        radius = max(4, int(round(8 - min(point["distance"], max_distance) / max_distance * 4)))
        cv2.circle(canvas, (px, py), radius + 2, (12, 12, 12), -1, cv2.LINE_AA)
        cv2.circle(canvas, (px, py), radius, color, -1, cv2.LINE_AA)
        cv2.putText(canvas, f"ID{point['id']} {point['distance']:.1f}m", (px + 8, py - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (235, 235, 235), 1, cv2.LINE_AA)

    cv2.putText(canvas, "Radar sector view", (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.82, (245, 245, 245), 2, cv2.LINE_AA)
    cv2.putText(canvas, f"objects={len(points)} range={max_distance:.0f}m fov={fov_deg:.0f}deg", (18, size - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (210, 216, 224), 1, cv2.LINE_AA)
    cv2.putText(canvas, "front", polar_project(max_distance, 0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (205, 214, 224), 1, cv2.LINE_AA)
    _draw_colorbar(canvas, cv2, np, size - 82, 70, 24, size - 150, max_distance)
    return canvas, points


def render_radar_point_cloud(
    radar_data,
    size=DEFAULT_POINT_CLOUD_SIZE,
    max_distance=DEFAULT_POINT_CLOUD_MAX_DISTANCE,
):
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("radar visualization requires cv2 and numpy") from exc

    points = radar_points_from_scan(radar_data)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    center = (size // 2, size // 2)
    margin = 44
    radius_px = max(1, size // 2 - margin)

    if max_distance <= 0 and points:
        max_distance = max(point["distance"] for point in points)
    max_distance = max(1.0, float(max_distance))

    grid_color = (55, 55, 55)
    axis_color = (95, 95, 95)
    for fraction in (0.25, 0.5, 0.75, 1.0):
        cv2.circle(canvas, center, int(radius_px * fraction), grid_color, 1, cv2.LINE_AA)
    cv2.line(canvas, (center[0], margin), (center[0], size - margin), axis_color, 1, cv2.LINE_AA)
    cv2.line(canvas, (margin, center[1]), (size - margin, center[1]), axis_color, 1, cv2.LINE_AA)

    if points:
        distances = np.array([point["distance"] for point in points], dtype=np.float32)
        norm = np.clip(distances / max_distance, 0.0, 1.0)
        color_values = (255 - norm * 255).astype(np.uint8).reshape(-1, 1)
        colors = cv2.applyColorMap(color_values, cv2.COLORMAP_TURBO).reshape(-1, 3)

        for point, color in zip(points, colors):
            px = int(round(center[0] + point["x"] / max_distance * radius_px))
            py = int(round(center[1] - point["y"] / max_distance * radius_px))
            if 0 <= px < size and 0 <= py < size:
                cv2.circle(canvas, (px, py), 5, tuple(int(c) for c in color), -1, cv2.LINE_AA)

    cv2.putText(canvas, "Radar point cloud", (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (240, 240, 240), 2, cv2.LINE_AA)
    cv2.putText(canvas, "forward", (center[0] + 10, margin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 210, 210), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"points={len(points)} max={max_distance:.1f}", (18, size - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (210, 210, 210), 1, cv2.LINE_AA)

    bar_w = 24
    bar_h = size - 2 * margin
    gradient = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(bar_h, 1)
    colorbar = cv2.applyColorMap(gradient, cv2.COLORMAP_TURBO)
    canvas[margin:margin + bar_h, size - margin:size - margin + bar_w] = colorbar
    cv2.putText(canvas, "near", (size - margin - 12, margin - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(canvas, "far", (size - margin - 6, margin + bar_h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
    return canvas, points

class _FILTER_DATA(Structure):
    _fields_ = [("FilterEN", c_ubyte),
                ("FilterMode", c_ubyte),
                ("FilterId", c_uint),
                ("FilterMask", c_uint)
                ]


class _INIT_CONFIG(Structure):
    _fields_ = [("filter_info", _FILTER_DATA * 14),
                ("filter_num", c_uint),
                ("Reserved", c_ulong),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte),
                ("BotRate", c_uint)
                ]


class _VCI_BOARD_INFO(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),
        ("fw_Version", c_ushort),
        ("dr_Version", c_ushort),
        ("in_Version", c_ushort),
        ("irq_Num", c_ushort),
        ("can_Num", c_byte),
        ("str_Serial_Num", c_char * 20),
        ("str_hw_Type", c_char * 40),
        ("Reserved", c_ushort * 4)
    ]

class _VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_byte),
        ("SendType", c_byte),
        ("RemoteFlag", c_byte),
        ("ExternFlag", c_byte),
        ("DataLen", c_byte),
        ("Data", c_byte * 8),
        ("Reserved", c_byte * 3)
    ]



def _script_dir():
    return os.path.dirname(os.path.abspath(__file__))


def _default_library_path():
    candidates = [
        os.path.join(_script_dir(), "usbcan驱动64位linux", "iTekon-usb-210420", "lib", "libiTekon-usb.so"),
        os.path.join(_script_dir(), "target", "libiTekon-usb.so"),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[-1]


def _elf_class(path):
    try:
        with open(path, "rb") as fp:
            header = fp.read(5)
    except OSError:
        return None
    if len(header) < 5 or header[:4] != b"\x7fELF":
        return None
    if header[4] == 1:
        return 32
    if header[4] == 2:
        return 64
    return None


def find_itek_usb_devices():
    devices = []
    sys_usb = "/sys/bus/usb/devices"
    if not os.path.isdir(sys_usb):
        return devices

    for name in sorted(os.listdir(sys_usb)):
        path = os.path.join(sys_usb, name)
        try:
            with open(os.path.join(path, "idVendor"), "r", encoding="ascii") as fp:
                vendor = fp.read().strip().lower()
            with open(os.path.join(path, "idProduct"), "r", encoding="ascii") as fp:
                product = fp.read().strip().lower()
        except OSError:
            continue

        if vendor != ITEK_USB_VENDOR or product != ITEK_USB_PRODUCT:
            continue

        info = {"sysfs": name, "vendor_id": vendor, "product_id": product}
        for key in ("manufacturer", "product", "serial"):
            try:
                with open(os.path.join(path, key), "r", encoding="utf-8", errors="replace") as fp:
                    info[key] = fp.read().strip()
            except OSError:
                info[key] = ""
        devices.append(info)
    return devices


def _safe_send_message(message, code, text):
    if hasattr(message, "sendMessage"):
        message.sendMessage(code, text)


class Radar():
    def __init__(self, msg=None, library_path=None, baud_rate=None):
        self.dll = None
        self.ready = False
        self.last_receive_num = 0
        self.radarData = [0 for i in range(360)]
        self.sharedRadarData = [0 for i in range(360)]
        self.sharedRadarObjects = []
        self.min_distance = DEFAULT_POINT_CLOUD_MIN_DISTANCE
        self.front_only = True
        self.min_rcs = -64.0
        self.fov_deg = DEFAULT_RADAR_FOV_DEG
        self.count = 0
        self.library_path = os.environ.get("RADAR_LIB_PATH") or library_path or _default_library_path()
        self.baud_rate = baud_rate if baud_rate is not None else _parse_int(os.environ.get("RADAR_BAUD_RAW"), DEFAULT_BAUD_RATE)
        try:
            Radar.message = msg
            self._load_library()
            self._configure_symbols()
            y = self.dll.VCI_UsbInit()
            self._require_success(y, "VCI_UsbInit")
            y = self.dll.VCI_OpenDevice(DEV_TYPE_OPEN, 0, 0)
            self._require_success(y, "VCI_OpenDevice")
            conf = _INIT_CONFIG()
            conf.filter_num = 1
            conf.filter_info[0].FilterEN = 1
            conf.filter_info[0].FilterMode = 0x01
            conf.filter_info[0].FilterId = 0
            conf.filter_info[0].FilterMask = 0
            conf.Filter = 0
            conf.Mode = 0
            conf.BotRate = self.baud_rate
            y = self.dll.VCI_InitCan(DEV_TYPE_ACTIVE, 0, 0, byref(conf))
            self._require_success(y, "VCI_InitCan")
            self.ready = True
        except Exception as e:
            print("radar connection error: " + str(e))
            _safe_send_message(Radar.message, -1, '打开Radar串口失败！')
        #print("initcan: ",y)
        
        #board_info = _VCI_BOARD_INFO()
        #y = self.dll.VCI_ReadBoardInfo(254, 0, byref(board_info))


    def _load_library(self):
        if not os.path.exists(self.library_path):
            raise RadarConnectionError("library not found: " + self.library_path)

        lib_bits = _elf_class(self.library_path)
        python_bits = 64 if sys.maxsize > 2 ** 32 else 32
        if lib_bits and lib_bits != python_bits:
            raise RadarConnectionError(
                "library architecture mismatch: %s is ELF%d, Python is %d-bit %s"
                % (self.library_path, lib_bits, python_bits, platform.machine())
            )

        target_dir = os.path.dirname(self.library_path)
        libusb_path = os.path.join(target_dir, "libusb-1.0.so")
        if os.path.exists(libusb_path):
            cdll.LoadLibrary(libusb_path)
        self.dll = cdll.LoadLibrary(self.library_path)

    def _configure_symbols(self):
        self.dll.VCI_UsbInit.restype = c_ulong
        self.dll.VCI_UsbExit.restype = c_ulong
        self.dll.VCI_OpenDevice.argtypes = [c_ulong, c_ulong, c_ulong]
        self.dll.VCI_OpenDevice.restype = c_ulong
        self.dll.VCI_CloseDevice.argtypes = [c_ulong, c_ulong]
        self.dll.VCI_CloseDevice.restype = c_ulong
        self.dll.VCI_InitCan.argtypes = [c_ulong, c_ulong, c_ulong, POINTER(_INIT_CONFIG)]
        self.dll.VCI_InitCan.restype = c_ulong
        self.dll.VCI_GetReceiveNum.argtypes = [c_ulong, c_ulong, c_ulong]
        self.dll.VCI_GetReceiveNum.restype = c_ulong
        self.dll.VCI_StartCAN.argtypes = [c_ulong, c_ulong, c_ulong]
        self.dll.VCI_StartCAN.restype = c_ulong
        self.dll.VCI_ResetCAN.argtypes = [c_ulong, c_ulong, c_ulong]
        self.dll.VCI_ResetCAN.restype = c_ulong
        self.dll.VCI_Receive.argtypes = [c_ulong, c_ulong, c_ulong, POINTER(_VCI_CAN_OBJ), c_ulong, c_int]
        self.dll.VCI_Receive.restype = c_ulong

    def _require_success(self, result, name):
        if result == 0:
            raise RadarConnectionError("%s failed with return value 0" % name)

    def start(self):
        if not self.ready:
            return False
        try:
            time.sleep(0.1)
            y = self.dll.VCI_StartCAN(DEV_TYPE_ACTIVE, 0, 0)
            self._require_success(y, "VCI_StartCAN")
            time.sleep(0.1)
            return True
        except Exception as e:
            print('start radar error, continue', e)
            _safe_send_message(Radar.message, -1, '打开Radar串口失败！')
            return False
    #def getRadarData(self):
    #    th = threading.Thread(target=self.getData)
    #    th.start()
    #    print("thread is started")
    #def returnRadarData(self):
    #    return self.sharedRadarData

    def close(self):
        try:
            if self.dll:
                self.dll.VCI_ResetCAN(DEV_TYPE_ACTIVE, 0, 0)
                self.dll.VCI_CloseDevice(DEV_TYPE_ACTIVE, 0)
                self.dll.VCI_UsbExit()
        except Exception as e:
            print('close radar error,continue:',e)
            _safe_send_message(Radar.message, -1, '关闭Radar失败！')
    def stop(self):
        try:
            if self.dll:
                self.dll.VCI_ResetCAN(DEV_TYPE_ACTIVE, 0, 0)
            #self.dll.VCI_CloseDevice(254,0)
            #self.dll.VCI_UsbExit()
        except Exception as e:
            print('stop radar error,continue:',e)
            _safe_send_message(Radar.message, -1, '关闭Radar失败！')

    def getRadarData(self):
        self.count = self.count + 1
        self.sharedRadarData = [0 for i in range(360)]
        self.sharedRadarObjects = []
        if not self.ready:
            return self.sharedRadarData
        try:
            msg_num = self.dll.VCI_GetReceiveNum(DEV_TYPE_ACTIVE, 0, 0)
            self.last_receive_num = int(msg_num)
            #with open('num11.txt','a') as file_handle1:   
            #.txt可以不自己新建,代码会自动新建
            #file_handle1.write(str(msg_num)+':'+str(self.count)+' ')
            #print('data num:',msg_num)
            recv_msg2 = _VCI_CAN_OBJ * msg_num
            recv_msg = recv_msg2()
            self.dll.VCI_Receive(DEV_TYPE_ACTIVE, 0, 0, recv_msg, msg_num, 0)
            #with open('num22.txt','a') as file_handle2:   # .txt可以不自己新建,代码会自动新建
            #    file_handle2.write("Received:"+str(self.count))
            #print('msg_num:',msg_num)
            for i in range(msg_num):
                data = [recv_msg[i].Data[j] & 0xff for j in range(8)]
                _update_radar_data_from_frame(
                    self.sharedRadarData,
                    self.sharedRadarObjects,
                    recv_msg[i].ID,
                    data,
                    min_distance=self.min_distance,
                    front_only=self.front_only,
                    min_rcs=self.min_rcs,
                    fov_deg=self.fov_deg,
                )
        except Exception as e:
            print("read data error")
        #print(self.sharedRadarData)
        return self.sharedRadarData


class SocketCanRadar:
    def __init__(self, interface):
        self.interface = interface
        self.ready = False
        self.last_receive_num = 0
        self.sharedRadarData = [0 for i in range(360)]
        self.sharedRadarObjects = []
        self.min_distance = DEFAULT_POINT_CLOUD_MIN_DISTANCE
        self.front_only = True
        self.min_rcs = -64.0
        self.fov_deg = DEFAULT_RADAR_FOV_DEG
        self.sock = None
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((interface,))
            self.sock.settimeout(0.05)
            self.ready = True
        except Exception as e:
            print("socketcan connection error: " + str(e))

    def start(self):
        return self.ready

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None
        self.ready = False

    def stop(self):
        self.close()

    def getRadarData(self, read_window=0.2):
        self.sharedRadarData = [0 for i in range(360)]
        self.sharedRadarObjects = []
        self.last_receive_num = 0
        if not self.ready:
            return self.sharedRadarData

        deadline = time.monotonic() + read_window
        while time.monotonic() < deadline:
            try:
                frame = self.sock.recv(16)
            except socket.timeout:
                continue
            except OSError as e:
                if e.errno == errno.ENETDOWN:
                    print("socketcan interface is down: %s. Run: sudo bash scripts/bringup_peak_can.sh 500000" % self.interface)
                    self.ready = False
                    break
                raise
            if len(frame) < 16:
                continue
            can_id_raw, can_dlc, data = struct.unpack("=IB3x8s", frame)
            if can_id_raw & CAN_EFF_FLAG:
                frame_id = can_id_raw & CAN_EFF_MASK
            else:
                frame_id = can_id_raw & CAN_SFF_MASK
            payload = list(data[:can_dlc])
            self.last_receive_num += 1
            _update_radar_data_from_frame(
                self.sharedRadarData,
                self.sharedRadarObjects,
                frame_id,
                payload,
                min_distance=self.min_distance,
                front_only=self.front_only,
                min_rcs=self.min_rcs,
                fov_deg=self.fov_deg,
            )
        return self.sharedRadarData


class _ConsoleMessage:
    def sendMessage(self, code, text):
        print("message[%s]: %s" % (code, text))


def print_probe(library_path):
    print("python:", platform.machine(), 64 if sys.maxsize > 2 ** 32 else 32, "bit")
    print("library:", library_path)
    print("library_elf_bits:", _elf_class(library_path))
    devices = find_itek_usb_devices()
    if devices:
        for dev in devices:
            print(
                "usb: {vendor_id}:{product_id} {manufacturer} {product_name} sysfs={sysfs}".format(
                    vendor_id=dev["vendor_id"],
                    product_id=dev["product_id"],
                    manufacturer=dev.get("manufacturer", ""),
                    product_name=dev.get("product", ""),
                    sysfs=dev["sysfs"],
                )
            )
    else:
        print("usb: iTEK USBCAN 0471:1200 not found")


def _parse_int(value, default=None):
    if value is None or value == "":
        return default
    return int(str(value), 0)


def configure_display(display_arg=""):
    if display_arg:
        os.environ["DISPLAY"] = display_arg
        return display_arg
    if os.environ.get("DISPLAY"):
        return os.environ["DISPLAY"]
    if os.path.exists("/tmp/.X11-unix/X0"):
        os.environ["DISPLAY"] = ":0"
        return ":0"
    return ""


def find_socketcan_interfaces():
    interfaces = []
    sys_net = "/sys/class/net"
    if not os.path.isdir(sys_net):
        return interfaces
    for name in sorted(os.listdir(sys_net)):
        if not name.startswith("can"):
            continue
        path = os.path.join(sys_net, name)
        if not os.path.exists(path):
            continue
        try:
            target = os.path.realpath(path)
        except OSError:
            target = ""
        score = 0
        if "/virtual/net/" in target:
            score += 20
        if name == "can2":
            score += 10
        if read_net_operstate(name) == "up":
            score += 5
        interfaces.append((score, name))
    return [name for _, name in sorted(interfaces, reverse=True)]


def read_net_operstate(interface):
    try:
        with open(os.path.join("/sys/class/net", interface, "operstate"), "r", encoding="ascii") as fp:
            return fp.read().strip().lower()
    except OSError:
        return ""


def run_ip_command(args):
    command = ["ip"] + args
    if os.geteuid() != 0:
        # Configuring a CAN interface requires CAP_NET_ADMIN. The maintained
        # container grants passwordless sudo for these short ip-link commands.
        command = ["sudo", "-n"] + command
    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
    except OSError as exc:
        return False, str(exc)
    if result.returncode == 0:
        return True, result.stdout.strip()
    return False, result.stderr.strip() or result.stdout.strip()


def bring_up_socketcan(interface, bitrate=DEFAULT_CAN_BITRATE):
    if read_net_operstate(interface) == "up":
        return True

    run_ip_command(["link", "set", interface, "down"])
    ok, msg = run_ip_command(["link", "set", interface, "type", "can", "bitrate", str(int(bitrate))])
    if not ok:
        print("socketcan bringup warning: failed to set bitrate on %s: %s" % (interface, msg))
    ok, msg = run_ip_command(["link", "set", interface, "up"])
    if not ok:
        print("socketcan bringup warning: failed to bring up %s: %s" % (interface, msg))
        return False
    return True


def select_socketcan_interface(requested=""):
    if requested:
        return requested
    interfaces = find_socketcan_interfaces()
    if not interfaces:
        raise RadarConnectionError("no SocketCAN interface found")
    return interfaces[0]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the SR73F millimeter-wave radar sector display.")
    parser.add_argument("--socketcan", default=os.environ.get("RADAR_CAN_IFACE", ""), help="SocketCAN interface. Default: auto-detect.")
    parser.add_argument("--headless", action="store_true", help="Do not open the OpenCV window.")
    parser.add_argument("--save", default="", help="Save the latest rendered sector image to this path.")
    parser.add_argument("--once", action="store_true", help="Read and render once, then exit.")
    parser.add_argument("--debug", action="store_true", help="Print parsed object fields.")
    parser.add_argument("--display", default="", help=argparse.SUPPRESS)
    args = parser.parse_args()

    show_window = not args.headless
    if show_window:
        display = configure_display(args.display)
        if display:
            print("display:", display)
        else:
            print("display: not available; use --headless or --save if OpenCV cannot open a window")

    try:
        interface = select_socketcan_interface(args.socketcan)
    except RadarConnectionError as exc:
        print("radar connection error:", exc)
        sys.exit(1)

    print("socketcan:", interface)
    if not bring_up_socketcan(interface, DEFAULT_CAN_BITRATE):
        print(
            "radar connection error: failed to configure %s; "
            "verify passwordless sudo/CAP_NET_ADMIN" % interface
        )
        sys.exit(1)
    radar = SocketCanRadar(interface)
    radar.min_distance = DEFAULT_POINT_CLOUD_MIN_DISTANCE
    radar.front_only = True
    radar.min_rcs = -64.0
    radar.fov_deg = DEFAULT_RADAR_FOV_DEG
    if not radar.ready or not radar.start():
        sys.exit(1)

    try:
        window_name = "Radar Sector View"
        if show_window:
            import cv2
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, DEFAULT_POINT_CLOUD_SIZE, DEFAULT_POINT_CLOUD_SIZE)
            print("viz: window opened, press Ctrl-C to quit")

        while True:
            time.sleep(0.05)
            radar.getRadarData()
            display_objects = dedupe_radar_objects(radar.sharedRadarObjects)
            print("receive_num:", radar.last_receive_num, "objects:", len(display_objects), "iface:", interface)
            image, points = render_radar_object_cloud(
                display_objects,
                size=DEFAULT_POINT_CLOUD_SIZE,
                max_distance=DEFAULT_POINT_CLOUD_MAX_DISTANCE,
                fov_deg=DEFAULT_RADAR_FOV_DEG,
            )
            if args.debug:
                for obj in sorted(display_objects, key=lambda item: item["distance"]):
                    print(
                        " object id={id:3d} long={dist_long:7.2f}m lat={dist_lat:7.2f}m "
                        "range={distance:6.2f}m bearing={bearing_deg:6.1f}deg "
                        "vlong={vrel_long:7.2f}m/s vlat={vrel_lat:7.2f}m/s "
                        "dyn={dyn_prop} rcs={rcs:6.1f} raw={raw}".format(**obj)
                    )
            if args.save:
                import cv2
                cv2.imwrite(args.save, image)
            if show_window:
                import cv2
                try:
                    cv2.imshow(window_name, image)
                    key = cv2.waitKey(30) & 0xFF
                except cv2.error as exc:
                    print("OpenCV display failed: %s" % exc)
                    print("Try: export DISPLAY=:0, or run with --display :0, or use --viz-save")
                    break
                if key in (27, ord("q")):
                    break
            if args.once:
                break
    except KeyboardInterrupt:
        print("\nradar stopped by Ctrl-C")
    finally:
        radar.close()
        if show_window:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass
