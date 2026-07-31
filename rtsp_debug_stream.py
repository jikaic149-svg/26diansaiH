"""可选 RTSP 调试图传封装。

本模块只管理 Wi-Fi 与 RTSP 服务，不参与钢球位置闭环。默认的 EXISTING 模式
不会调用 ``start_ap()``、``stop_ap()`` 或 ``connect()``，因此程序退出不会改变
板端当前 Wi-Fi 配置。只有显式选择 AP 模式时，程序才会创建热点。

为了兼容 MaixCAM2，第一版 RTSP 发送摄像头 NV21 主通道的原始画面；YOLO、水管
中轴与文字仍只画在本地调试屏幕上。不要在这里把软件绘制后的 RGB 帧重新编码，
否则会给控制主循环增加明显开销。
"""


VALID_WIFI_MODES = ("EXISTING", "AP")


def normalize_wifi_mode(value):
    """规范化并校验 Wi-Fi 模式，供板端和 PC 纯逻辑测试复用。"""
    mode = str(value or "EXISTING").strip().upper()
    if mode not in VALID_WIFI_MODES:
        raise ValueError(
            "RTSP_WIFI_MODE must be EXISTING or AP, got %r" % value)
    return mode


def should_restore_sta(mode, restore_enabled, ssid):
    """仅 AP 模式且明确填写恢复热点时，才允许退出时调用 connect。"""
    return (
        normalize_wifi_mode(mode) == "AP" and
        bool(restore_enabled) and
        bool(str(ssid or "").strip())
    )


class RtspDebugStream:
    """RTSP 调试服务生命周期管理器。

    ``open()`` 和 ``close()`` 可重复调用。硬件模块在 ``open()`` 内延迟导入，
    导入本文件本身不会初始化 Wi-Fi、相机或 RTSP。
    """

    def __init__(self, config):
        self.config = config
        self.enabled = bool(getattr(config, "RTSP_DEBUG_ENABLED", False))
        self.required = bool(getattr(config, "RTSP_REQUIRED", False))
        self.wifi_mode = normalize_wifi_mode(
            getattr(config, "RTSP_WIFI_MODE", "EXISTING"))
        self.wifi = None
        self.server = None
        self.server_started = False
        self.opened = False
        self.url = ""
        self._err = None
        self._time = None

    def _wait_ip(self):
        wait_ms = max(0, int(getattr(
            self.config, "RTSP_WAIT_IP_MS", 5000)))
        step_ms = 100
        attempts = max(1, wait_ms // step_ms + 1)
        ip = ""
        for _ in range(attempts):
            ip = str(self.wifi.get_ip() or "")
            if ip:
                return ip
            self._time.sleep_ms(step_ms)
        return ""

    def _prepare_wifi(self):
        from maix import network

        self.wifi = network.wifi.Wifi()
        if self.wifi_mode == "AP":
            already_ap = bool(self.wifi.is_ap_mode())
            if not already_ap:
                result = self.wifi.start_ap(
                    str(getattr(
                        self.config, "RTSP_AP_SSID", "H_BALL_MAIXCAM")),
                    str(getattr(
                        self.config, "RTSP_AP_PASSWORD", "88888888")),
                )
                self._err.check_raise(result, "start wifi ap failed")
            print(
                "RTSP_WIFI|mode=AP|already_ap=%d|restore_on_exit=%d"
                % (
                    1 if already_ap else 0,
                    1 if bool(getattr(
                        self.config,
                        "RTSP_RESTORE_STA_ON_EXIT", False)) else 0,
                )
            )
        else:
            # 安全默认值：完全复用系统设置页/二维码已经建立的网络。
            # 即使当前残留在 AP 模式，这里也只报告，不擅自切网。
            print(
                "RTSP_WIFI|mode=EXISTING|ap_mode=%d|network_control=0"
                % (1 if bool(self.wifi.is_ap_mode()) else 0)
            )

        ip = self._wait_ip()
        if not ip:
            raise RuntimeError(
                "active Wi-Fi has no IP; connect it in system settings first")
        print("RTSP_WIFI_READY|mode=%s|ip=%s" % (self.wifi_mode, ip))

    def open(self, stream_camera):
        """绑定 NV21 主通道并启动 RTSP；可选模式失败时返回 False。"""
        if not self.enabled:
            return False
        if self.opened:
            return True
        if stream_camera is None:
            raise ValueError("stream_camera is required when RTSP is enabled")

        try:
            from maix import err, rtsp, time

            self._err = err
            self._time = time
            self._prepare_wifi()
            self.server = rtsp.Rtsp(
                fps=max(1, int(getattr(
                    self.config, "RTSP_FPS", 20))),
                bitrate=max(100000, int(getattr(
                    self.config, "RTSP_BITRATE", 1500000))),
            )
            self._err.check_raise(
                self.server.bind_camera(stream_camera),
                "bind rtsp camera failed",
            )
            self._err.check_raise(
                self.server.start(), "start rtsp server failed")
            self.server_started = True
            self.opened = True
            self.url = str(self.server.get_url() or "")
            print(
                "RTSP_READY|url=%s|fps=%d|bitrate=%d|overlay=0"
                % (
                    self.url,
                    max(1, int(getattr(
                        self.config, "RTSP_FPS", 20))),
                    max(100000, int(getattr(
                        self.config, "RTSP_BITRATE", 1500000))),
                )
            )
            return True
        except Exception as exc:
            print("RTSP_OPEN_ERROR|%s" % exc)
            self.close()
            if self.required:
                raise
            print("RTSP_DISABLED_AFTER_ERROR|vision_continues=1")
            return False

    def _restore_sta_if_requested(self):
        if self.wifi_mode != "AP" or self.wifi is None:
            return

        restore_enabled = bool(getattr(
            self.config, "RTSP_RESTORE_STA_ON_EXIT", False))
        ssid = str(getattr(
            self.config, "RTSP_RESTORE_STA_SSID", "") or "").strip()
        password = str(getattr(
            self.config, "RTSP_RESTORE_STA_PASSWORD", "") or "")
        if not should_restore_sta(self.wifi_mode, restore_enabled, ssid):
            # stop_ap() 只停服务，不会删除持久 AP 配置，也不会恢复原热点。
            # 没有 STA 凭据时保持当前 AP 可连接，并给出明确告警。
            print(
                "WIFI_AP_LEFT_ACTIVE|reason=NO_STA_RESTORE|"
                "action=USE_SYSTEM_QR_TO_SWITCH_STA"
            )
            return

        timeout_s = max(1, int(getattr(
            self.config, "RTSP_RESTORE_STA_TIMEOUT_SEC", 20)))
        try:
            result = self.wifi.connect(
                ssid, password, True, timeout_s)
            self._err.check_raise(result, "restore wifi sta failed")
            ip = str(self.wifi.get_ip() or "")
            print("WIFI_STA_RESTORED|ssid=%s|ip=%s" % (ssid, ip))
        except Exception as exc:
            print(
                "WIFI_STA_RESTORE_ERROR|ssid=%s|error=%s|"
                "action=USE_SYSTEM_QR_TO_SWITCH_STA"
                % (ssid, exc)
            )

    def close(self):
        """先停 RTSP，再按显式配置恢复 STA；多次调用不会重复切网。"""
        if self.server is not None and self.server_started:
            try:
                self.server.stop()
                print("RTSP_STOPPED")
            except Exception as exc:
                print("RTSP_STOP_ERROR|%s" % exc)
        self.server_started = False
        self.server = None

        if self.wifi is not None:
            self._restore_sta_if_requested()
        self.wifi = None
        self.opened = False
        self.url = ""
