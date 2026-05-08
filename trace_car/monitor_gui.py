#!/usr/bin/env python3
# coding: utf-8

#!/usr/bin/env python3
import tkinter as tk
import math
import threading
import argparse

try:
    import rclpy
    import time
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParameters
    from trace_car.msg import Detection, CarInfo, GuiControl
    from gazebo_msgs.srv import SetEntityState
    from gazebo_msgs.msg import EntityState
    from std_srvs.srv import Empty
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False



# ══════════════════════════════════════════════════════════════════════════════
#  COLOR PALETTE  — matches Car Control reference (light, blue-accent style)
# ══════════════════════════════════════════════════════════════════════════════
BG          = "#e1e9f1"   # overall window – light blue-grey
PANEL       = "#ffffff"   # panel / frame backgrounds
BORDER      = "#c0d8f0"   # borders / separators
ACCENT      = "#1976d2"   # primary blue (titles, active slider, value labels)
ACCENT2     = "#2196f3"   # lighter blue (secondary highlights)
WARN        = "#e53935"   # red – stop / warning
TEXT        = "#1a3a5c"   # main text (near-black)
TEXT_DIM    = "#546e7a"   # dimmed text
SLIDER_BG   = "#c5d0da"   # slider trough
SLIDER_FG   = "#1976d2"   # slider thumb / active fill

STRAIGHT    = "#1abb1a"   #
TURN        = "#1976d2"   #
CROSS       = "#89970A"   #
STOP        = "#e53935"   #
BLACK       = "#1a3a5c"   #

TRACK    = "#D0E4F5"   # slider track bg
FILL     = "#2196F3"   # slider fill (bright blue)
THUMB    = "#FFFFFF"   # slider thumb
THUMB_BD = "#1976D2"   # thumb border

BTN_STOP    = "#e53935"   # stop button
BTN_START   = "#43a047"   # start (green)
BTN_RESET   = "#4593e2"   # reset

BTN_VIEW    = "#5f7e91"   # view buttons

CCD_FAR_CLR = ACCENT
CCD_NEAR_CLR= "#1565c0"



# ══════════════════════════════════════════════════════════════════════════════
#  MoNITOR NODE  — subscribes to ROS2 topics and forwards data to GUI callbacks
# ══════════════════════════════════════════════════════════════════════════════
class MonitorNode(Node):
    def __init__(self):
        super().__init__("monitor_node")
        self.ccd_detection_msg = None
        self.car_info_msg = None
        self.running = threading.Event()   # set=running, clear=paused
        self.running.set()                 # init unlock

        # subscribe
        self.create_subscription(
            Detection,
            "/ccd_detection",
            self.ccd_detection_callback,
            10
        )


        self.create_subscription(
            CarInfo,
            "/car_info",
            self.car_info_callback,
            10
        )


        self.gui_control_pub = self.create_publisher(GuiControl, "/gui_control", 10)


        # parameter client for dynamic reconfigure
        self.sens_param_client = self.create_client(SetParameters, '/ccd_node/set_parameters')
        self.ctrl_param_client = self.create_client(SetParameters, '/controller_node/set_parameters')

        # gazebo control client
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')

        # pause and unpause service
        self.pause_client = self.create_client(Empty, '/pause_physics')
        self.unpause_client = self.create_client(Empty, '/unpause_physics')

        # view pub
        self.view_pub = self.create_publisher(String, '/camera_view_cmd', 10)

        self.get_logger().info("Monitor Node started [ROS2 mode]")

    def on_reset(self):
        self.ccd_detection_msg = None
        self.car_info_msg = None

    def ccd_detection_callback(self, msg):
        if not self.running.is_set():
            self.car_info_msg = None
            return

        self.ccd_detection_msg = msg

    def car_info_callback(self, msg):
        if not self.running.is_set():
            self.car_info_msg = None
            return

        self.car_info_msg = msg

    def set_parameter(self, client, name, value, value_type="float"):
        if not client.service_is_ready():
            self.get_logger().warn(f"Parameter service not ready: {name}")
            return
        req = SetParameters.Request()
        if value_type == "float":
            p = Parameter(name, Parameter.Type.DOUBLE, float(value)).to_parameter_msg()
        else:
            p = Parameter(name, Parameter.Type.INTEGER, int(value)).to_parameter_msg()
        req.parameters = [p]
        client.call_async(req)

    def set_pose(self,
                    model_name: str   = "car",
                    x:          float = 1.175,
                    y:          float = -2.985,
                    z:          float = 0.007,
                    roll:       float = 0.0,
                    pitch:      float = 0.0,
                    yaw:        float = 3.14159):
        if not self.set_state_client.service_is_ready():
            self.get_logger().warn("SetModelState service not ready")
            return

        req = SetEntityState.Request()
        state = EntityState()

        state.name = model_name

        # position
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z

        # orientation
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        state.pose.orientation.w = cr * cp * cy + sr * sp * sy
        state.pose.orientation.x = sr * cp * cy - cr * sp * sy
        state.pose.orientation.y = cr * sp * cy + sr * cp * sy
        state.pose.orientation.z = cr * cp * sy - sr * sp * cy

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0

        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        state.reference_frame = "world"
        req.state = state


        rclpy.spin_until_future_complete(
            self,
            self.set_state_client.call_async(req),
            timeout_sec=2.0
            )


    def pause_physics(self):
        if not self.pause_client.service_is_ready():
            if not self.pause_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Pause service not ready")
                return
        req = Empty.Request()

        rclpy.spin_until_future_complete(self, self.pause_client.call_async(req), timeout_sec=2.0)


    def unpause_physics(self):
        if not self.unpause_client.service_is_ready():
            self.get_logger().warn("Unpause service not ready")
            return
        req = Empty.Request()
        self.unpause_client.call_async(req)

    def send_view_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.view_pub.publish(msg)

    def send_gui_control(self, command: int):
        msg = GuiControl()
        msg.command = command
        self.gui_control_pub.publish(msg)

# ══════════════════════════════════════════════════════════════════════════════
#  SPEEDOMETER (Canvas widget)
# ══════════════════════════════════════════════════════════════════════════════
class Speedometer(tk.Canvas):
    def __init__(self, parent, size=160, max_val=4000, **kw):
        super().__init__(parent,
                         width=size, height=size,
                         bg=BG,
                         highlightthickness=0,
                         **kw)
        self.size = size
        self.max_val = max_val
        self._value = 0
        self._draw(0)

    def set(self, val, road_type):
        # self._value = max(0, min(val, self.max_val))
        # self._draw(self._value, road_type)
        self._draw(val, road_type)


    def lerp_color(self, c1, c2, t):
        """c1, c2: '#rrggbb', t: 0~1"""
        t = max(0, min(t, 1))
        r1, g1, b1 = int(c1[1:3], 16), int(c1[3:5], 16), int(c1[5:7], 16)
        r2, g2, b2 = int(c2[1:3], 16), int(c2[3:5], 16), int(c2[5:7], 16)

        r = int(r1 + (r2 - r1) * t)
        g = int(g1 + (g2 - g1) * t)
        b = int(b1 + (b2 - b1) * t)

        return f"#{r:02x}{g:02x}{b:02x}"

    def _draw(self, val, road_type="----"):
        self.delete("all")
        cx = cy = self.size / 2
        r  = self.size / 2 - 8

        # Outer ring
        self.create_oval(cx-r, cy-r, cx+r, cy+r,
                         outline=BORDER, width=5)
        # Arc track
        self.create_arc(cx-r, cy-r, cx+r, cy+r,
                        start=220, extent=-260,
                        outline=SLIDER_BG, width=10, style="arc")
        # Value arc
        pct = val / self.max_val
        pct = max(0, min(pct, 1))

        sweep = pct * 260

        if pct < 0.25:
            color = self.lerp_color("#1976d2", "#2bb832", pct / 0.25)
        elif pct < 0.55:
            color = self.lerp_color("#2bb832", "#fdd835", (pct-0.25)/0.3)
        elif pct < 0.68:
            color = self.lerp_color("#fdd835", "#e53935", (pct-0.55)/0.13)
        else:
            color = "#e53935"


        if sweep > 0.5:
            self.create_arc(cx-r, cy-r, cx+r, cy+r,
                            start=220, extent=-sweep,
                            outline=color, width=10, style="arc")
        # Needle
        angle_deg = 220 - pct * 260
        angle_rad = math.radians(angle_deg)
        nx = cx + (r - 18) * math.cos(angle_rad)
        ny = cy - (r - 18) * math.sin(angle_rad)
        self.create_line(cx, cy, nx, ny, fill=WARN, width=3)
        self.create_oval(cx-5, cy-5, cx+5, cy+5, fill=WARN, outline="")

        # Value text
        self.create_text(cx - 14, cy + 25, text=f"{int(val)}",
                         fill=TEXT, font=("Arial", 14, "bold"))
        self.create_text(cx + 20, cy + 27, text="mm/s",
                         fill=TEXT_DIM, font=("Arial", 9))
        # road type
        if road_type:
            match road_type:
                case "STRAIGHT":
                    color = STRAIGHT
                case "TURN":
                    color = TURN
                case "CROSS":
                    color = CROSS
                case "STOP":
                    color = STOP
                case "BLACK":
                    color = BLACK
                case _:
                    color = ACCENT
            self.create_text(cx, cy + 48, text=str(road_type),
                            fill=color, font=("Arial", 12, "bold"))
# ══════════════════════════════════════════════════════════════════════════════
#  CCD BAR (128-pixel line sensor visualisation)
# ══════════════════════════════════════════════════════════════════════════════
class CCDBar(tk.Canvas):
    def __init__(self, parent, label="CCD", **kw):
        super().__init__(parent, height=50, bg=BG,
                         highlightthickness=1, highlightbackground=BORDER, **kw)
        self.label      = label
        self._data      = [0] * 128
        self._edge_l    = 0
        self._center    = 63
        self._edge_r    = 127
        self.bind("<Configure>", lambda e: self._draw())

    def update_data(self, data, edge_l, center, edge_r):
        self._data   = data
        self._edge_l = edge_l
        self._center = center
        self._edge_r = edge_r
        self._draw()

    def _draw(self):
        self.delete("all")
        W = self.winfo_width() or 600
        H = 50
        n = len(self._data)
        if n == 0:
            return
        bar_w = W / n

        for i, gray in enumerate(self._data):
            if i == self._edge_l:
                color = "#4dabf7"
            elif i == self._edge_r:
                color = "#40c057"
            elif i == self._center:
                color = "#f03e3e"
            else:
                color = f"#{gray:02x}{gray:02x}{gray:02x}"

            x0 = i * bar_w
            x1 = x0 + bar_w
            self.create_rectangle(x0, 8, x1, H - 4,
                                  fill=color, outline="")



# ══════════════════════════════════════════════════════════════════════════════
#  CUSTOM SLIDER WIDGET
# ══════════════════════════════════════════════════════════════════════════════
class FancySlider(tk.Canvas):
    HEIGHT   = 18
    TRACK_H  = 6
    THUMB_W  = 12
    THUMB_H  = 17

    def __init__(self,
                 parent,
                 from_,
                 to,
                 var: tk.DoubleVar,
                 resolution,
                 on_change=None, **kw):
        kw.setdefault("bg", PANEL)
        kw.setdefault("highlightthickness", 0)
        super().__init__(parent, height=self.HEIGHT, **kw)
        self.from_  = from_
        self.to     = to
        self.var    = var
        self.resolution    = resolution
        self.on_change = on_change
        self._dragging = False

        # window change redraw
        self.bind("<Configure>",     self._redraw)
        # mouse events
        self.bind("<ButtonPress-1>", self._press)
        # drag only if initially pressed on the thumb
        self.bind("<B1-Motion>",     self._drag)
        # release anywhere cancels dragging
        self.bind("<ButtonRelease-1>", self._release)
        var.trace_add("write", lambda *_: self._redraw())

    def _val_to_x(self, val, w):
        ratio = (val - self.from_) / (self.to - self.from_)
        pad = self.THUMB_W // 2 + 2
        return pad + ratio * (w - 2 * pad)

    def _x_to_val(self, x, w):
        pad = self.THUMB_W // 2 + 2
        ratio = (x - pad) / max(1, w - 2 * pad)
        return self.from_ + max(0, min(1, ratio)) * (self.to - self.from_)

    def _redraw(self, *_):
        self.delete("all")
        w = self.winfo_width()
        if w < 4:
            return
        h   = self.HEIGHT
        cy  = h // 2
        pad = self.THUMB_W // 2

        # track bg
        self.create_rectangle(pad, cy - self.TRACK_H//2,
                               w - pad, cy + self.TRACK_H//2,
                               fill=TRACK, outline=BORDER, width=1)
        val  = self.var.get()

        # thumb
        tx = self._val_to_x(val, w)
        self.create_rectangle(tx - self.THUMB_W//2, cy - self.THUMB_H//2,
                               tx + self.THUMB_W//2, cy + self.THUMB_H//2,
                               fill=THUMB, outline=THUMB_BD, width=2)
        # center groove on thumb
        self.create_line(tx, cy - 5, tx, cy + 5, fill=THUMB_BD, width=2)

    def _press(self, e):
        self._dragging = True
        self._set(e.x)

    def _drag(self, e):
        if self._dragging:
            self._set(e.x)

    def _release(self, e):
        self._dragging = False

    def _set(self, x):
        raw = self._x_to_val(x, self.winfo_width())

        if isinstance(self.var, tk.IntVar):
            v = int(round(raw))
        else:
            v = round(raw, self.resolution)

        self.var.set(v)
        if self.on_change:
            self.on_change(v)

    def _rescale(self, new_from, new_to):
        self.from_  = new_from
        self.to     = new_to
        self._redraw()

# ══════════════════════════════════════════════════════════════════════════════
#  LABELED SLIDER with editable min/max
# ══════════════════════════════════════════════════════════════════════════════
class ParamSlider(tk.Frame):
    def __init__(self,
                 parent,
                 label,
                 from_=0,
                 to=100,
                 default=50,
                 resolution=2,
                 value_type="float",
                 callback=None,
                 **kw):
        super().__init__(parent, bg=PANEL, **kw)
        self._cb   = callback
        self.value_type = value_type
        if value_type == "int":
            self._from = tk.IntVar(value=from_)
            self._to   = tk.IntVar(value=to)
            self._var  = tk.IntVar(value=default)
        else:
            self._from = tk.DoubleVar(value=from_)
            self._to   = tk.DoubleVar(value=to)
            self._var  = tk.DoubleVar(value=default)

        self._last_f = from_
        self._last_t = to

        self.grid_columnconfigure(1, weight=1)

        # Label
        tk.Label(self, text=label, bg=PANEL, fg=TEXT,
                 font=("Arial", 9, "bold"),
                 anchor="w",
                 width=26
                 ).grid(
                     row=0,
                     column=1,
                     columnspan=3,
                     sticky="w",
                     padx=2)

        # Min entry
        self._min_e = tk.Entry(self,
                               textvariable=self._from,
                               width=5,
                               bg="white",
                               fg=ACCENT,
                               insertbackground=ACCENT,
                               relief="sunken",
                               bd=1,
                               font=("Arial", 8))

        self._min_e.grid(row=1, column=0, padx=(0,4))
        self._min_e.bind("<Return>", self._rebuild_scale)    # enter key

        # Scale — blue trough like reference image
        self.sl = FancySlider(self,
                              from_=float(self._from.get()),
                              to=float(self._to.get()),
                              var=self._var,
                              resolution=resolution,
                              on_change=self._on_change)

        self.sl.grid(row=1, column=1, sticky="ew")

        # Max entry
        self._max_e = tk.Entry(self,
                               textvariable=self._to,
                               width=5,
                               bg="white",
                               fg=ACCENT,
                               insertbackground=ACCENT,
                               relief="sunken",
                               bd=1,
                               font=("Arial", 8))
        self._max_e.grid(row=1, column=2, padx=(4, 0))
        self._max_e.bind("<Return>", self._rebuild_scale)

        # Current value label
        self._val_lbl = tk.Label(self,
                                 textvariable=self._var,
                                 bg=PANEL,
                                 fg=ACCENT,
                                 anchor="w",
                                 font=("Arial", 9, "bold"),
                                 width=6)
        self.grid_columnconfigure(3, weight=0)
        self._val_lbl.grid(row=1, column=3, sticky="w", padx=4)

    def _rebuild_scale(self, _=None):
        try:
            f = self._from.get()
            t = self._to.get()
            v = self._var.get()


            self.focus_set()  # remove focus from entry after update
            if f > v or f > t or t < v:
                # set the min and max back
                self._from.set(self._last_f)
                self._to.set(self._last_t)
                return  # invalid range, ignore

            self._from.set(f)
            self._to.set(t)

            self.sl._rescale(f, t)

            self._last_f = f
            self._last_t = t
        except ValueError:
            pass

    def _on_change(self, _=None):
        if self._cb:
            self._cb(self._var.get())

    def get(self):
        return self._var.get()

# ══════════════════════════════════════════════════════════════════════════════
#  MAIN APPLICATION
# ══════════════════════════════════════════════════════════════════════════════
class CarDashboard(tk.Tk):
    def __init__(self, node: MonitorNode = None):
        super().__init__()
        self.node = node

        if self.node:
            self.system_paused = True
        else:
            self.system_paused = False
        self.refresh_rate = 50  # ms
        self.title("ROS2 Car Monitor")
        self.configure(bg=BG)
        self.resizable(True, True)

        # ── State vars ────────────────────────────────────────────────────────
        self.steer_l   = tk.DoubleVar(value=0.0)
        self.steer_r   = tk.DoubleVar(value=0.0)
        self.speed_l   = tk.DoubleVar(value=0.0)
        self.speed_r   = tk.DoubleVar(value=0.0)
        self.output_speed = tk.DoubleVar(value=0.0)
        self.road_type = tk.StringVar(value="----")
        self.tgt_speed = tk.DoubleVar(value=0.0)
        self.tgt_steer = tk.DoubleVar(value=0.0)
        self.status_msg= tk.StringVar(value="● connect ROS2 to run")

        self._build_ui()
        self._start_demo_loop()   # replace with ROS2 spin in real use

    # ─────────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Top title bar ─────────────────────────────────────────────────────
        title_bar = tk.Frame(self, bg=BG, pady=0,
                              highlightbackground=BORDER)
        title_bar.pack(fill="x", padx=0, pady=(0,1))
        tk.Label(title_bar, text="CAR  CONTROL",
                 bg=BG, fg=ACCENT,
                 font=("Arial", 16, "bold")).pack(side="left", padx=16, pady=(4, 0))
        tk.Label(title_bar, text="ROS2 Gazebo Controller",
                 bg=BG, fg=TEXT_DIM,
                 font=("Arial", 9)).pack(side="left", padx=4)
        tk.Label(title_bar, textvariable=self.status_msg,
                 bg=BG, fg=TEXT_DIM,
                 font=("Arial", 9)).pack(side="right", padx=16)

        # ── Main body ─────────────────────────────────────────────────────────
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=10, pady=4)

        # ── sensor ccd ─────────────────
        self._build_ccd_panel(body)

        # ── middle speed_odometer car_info  control_button ─────────────────
        middle = tk.Frame(body, bg=BG)
        middle.pack(fill="x", padx=0, pady=0)

        car_info_tk  = tk.Frame(middle, bg=BG)
        car_info_tk.pack(side="left", padx=0)
        self._build_car_info(car_info_tk)

        view_tk = tk.Frame(middle, bg=BG)
        view_tk.pack(side="right", padx=(12,0))
        self._build_view_buttons(view_tk)

        control_tk = tk.Frame(middle, bg=BG)
        control_tk.pack(side="right", padx=(3,0))
        self._build_control_buttons(control_tk)


        spacer_l = tk.Frame(middle, bg=BG)
        spacer_l.pack(side="left", expand=True, fill="x")

        self.speedo = Speedometer(middle, size=185, max_val=4000)
        self.speedo.pack(side="left",  padx=0)

        spacer_r = tk.Frame(middle, bg=BG)
        spacer_r.pack(side="left", expand=True, fill="x")

        # ── BOTTOM: Parameter sliders ──────────────────────────────────────────
        bottom = tk.Frame(body, bg=PANEL)
        bottom.columnconfigure(0, weight=1)
        bottom.columnconfigure(1, weight=1)
        self._build_param_sliders(bottom)
        bottom.pack(fill="x", padx=0, pady=(6,0))

    # ── CCD sensor panels ────────────────────────────────────────────────────
    def _build_ccd_panel(self, parent):
        for label, attr in [("CCD FAR", "ccd_far"), ("CCD CLOSE", "ccd_close")]:
            frame = tk.LabelFrame(parent, text=f"  {label}  ",
                                  bg=PANEL, fg=ACCENT,
                                  font=("Arial", 9, "bold"),
                                  relief="flat",
                                  highlightthickness=1,
                                  highlightbackground=BORDER,
                                  bd=1)
            frame.pack(fill="x", pady=(0,6))

            # Info row
            info = tk.Frame(frame, bg=PANEL)
            info.pack(fill="x", padx=6, pady=(2,0))

            # road width
            tk.Label(info, text="Road Width:",
                     bg=PANEL, fg=TEXT_DIM, font=("Arial", 8)
                     ).pack(side="left")
            road_width = tk.StringVar(value="---")
            tk.Label(info, textvariable=road_width,
                     bg=PANEL, fg=ACCENT, font=("Arial", 8, "bold"),width = 5, anchor="w"
                     ).pack(side="left", padx=2)

            # average brightness
            tk.Label(info, text="Avg Brightness:",
                     bg=PANEL, fg=TEXT_DIM, font=("Arial", 8)
                     ).pack(side="left")
            avg_var = tk.StringVar(value="---")
            tk.Label(info, textvariable=avg_var,
                     bg=PANEL, fg=ACCENT, font=("Arial", 8, "bold"),width = 6, anchor="w"
                     ).pack(side="left", padx=2)




            for tag, text, color in [("edge_L",     "Edge L:",  "#1565c0"),
                                     ("center",     "Center:",  "#c62828"),
                                     ("edge_R",     "Edge R:",  "#2e7d32")]:
                tk.Label(info, text=text,
                         bg=PANEL, fg=TEXT_DIM, font=("Arial", 8)
                         ).pack(side="left", padx=1)

                v = tk.StringVar(value="-")
                tk.Label(info, textvariable=v,
                         bg=PANEL, fg=color, font=("Arial", 8, "bold"),
                         width = 5, anchor="w"
                         ).pack(side="left")

                # store refs
                setattr(self, f"{attr}_{tag}", v)

            # CCD bar
            bar = CCDBar(frame, label=label)
            bar.pack(fill="x", padx=6, pady=(2,6))
            setattr(self, attr + "_bar", bar)
            setattr(self, attr + "_avg", avg_var)
            setattr(self, attr + "_road_width", road_width)

 # ── car info ────────────────────────────────
    def _build_car_info(self, parent):
        car_panel = parent
        car_panel.pack()

        # Target steer (top)
        tk.Label(car_panel, text="Target Steer(°)",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).grid(
                 row=0, column=1, pady=0, columnspan= 2)
        tk.Label(car_panel, textvariable=self.tgt_steer,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").grid(
                 row=1, column=1, pady=0, padx=(8, 0), columnspan=2)

        # Steer L
        steer_l_frame = tk.Frame(car_panel, bg=BG)
        steer_l_frame.grid(row=2, column=0, padx=0)
        tk.Label(steer_l_frame, text="Steer L",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).pack(anchor="w")
        tk.Label(steer_l_frame, textvariable=self.steer_l,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").pack(anchor="w")

        # Car box
        car_box = tk.Canvas(car_panel, width=100, height=120,
                            bg=PANEL, highlightthickness=0,
                            highlightbackground=BORDER)
        car_box.grid(row=2, column=1, columnspan=2, rowspan=2)
        self._draw_car_icon(car_box)

        # Steer R
        steer_r_frame = tk.Frame(car_panel, bg=BG)
        steer_r_frame.grid(row=2, column=3, padx=0)
        tk.Label(steer_r_frame, text="Steer R",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).pack(anchor="w")
        tk.Label(steer_r_frame, textvariable=self.steer_r,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").pack(anchor="w")

        # Speed L
        speed_l_frame = tk.Frame(car_panel, bg=BG)
        speed_l_frame.grid(row=3, column=0, rowspan=2, sticky="n")
        tk.Label(speed_l_frame, text="Speed L",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).pack(anchor="w")
        tk.Label(speed_l_frame, textvariable=self.speed_l,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").pack(anchor="w")

        # Speed R
        speed_r_frame = tk.Frame(car_panel, bg=BG)
        speed_r_frame.grid(row=3, column=3, rowspan=2, sticky="n")
        tk.Label(speed_r_frame, text="Speed R",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).pack(anchor="w")
        tk.Label(speed_r_frame, textvariable=self.speed_r,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").pack(anchor="w")

        # Target Speed
        tk.Label(car_panel, text="Target Speed(mm/s)",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).grid(
                 row=4, column=1, pady=0 , columnspan=2)
        tk.Label(car_panel, textvariable=self.tgt_speed,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=7, anchor="w").grid(
                 row=5, column=1, pady=0, padx=(8, 0), columnspan=2)

    # ── Speedometer + car image + speed labels ────────────────────────────────
    def _build_speedometer_row(self, parent):
        row = tk.Frame(parent, bg=BG)
        row.pack(fill="x", pady=4)

        # Car image panel with surrounding labels


        car_info_tk = tk.Frame(row, bg=BG)
        car_info_tk.pack(side="left", fill="y", padx=5)




        car_panel = tk.Frame(car_info_tk, bg=BG)
        car_panel.pack()



        # Target steer (top)
        tk.Label(car_panel, text="Target Steer(°):",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).grid(
                 row=0, column=1, pady=(0,2))
        tk.Label(car_panel, textvariable=self.tgt_steer,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").grid(
                 row=0, column=2, pady=(0,2))

        # Steer L
        steer_l_frame = tk.Frame(car_panel, bg=BG)
        steer_l_frame.grid(row=1, column=0, padx=0)
        tk.Label(steer_l_frame, text="Steer L",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).pack(anchor="w")
        tk.Label(steer_l_frame, textvariable=self.steer_l,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").pack(anchor="w")

        # Car box
        car_box = tk.Canvas(car_panel, width=100, height=120,
                            bg=PANEL, highlightthickness=0,
                            highlightbackground=BORDER)
        car_box.grid(row=1, column=1, columnspan=2, rowspan=2)
        self._draw_car_icon(car_box)

        # Steer R
        steer_r_frame = tk.Frame(car_panel, bg=BG)
        steer_r_frame.grid(row=1, column=3, padx=0)
        tk.Label(steer_r_frame, text="Steer R",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).pack(anchor="w")
        tk.Label(steer_r_frame, textvariable=self.steer_r,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").pack(anchor="w")

        # Speed L
        speed_l_frame = tk.Frame(car_panel, bg=BG)
        speed_l_frame.grid(row=2, column=0, rowspan=2, padx=6)
        tk.Label(speed_l_frame, text="Speed L",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).pack(anchor="w")
        tk.Label(speed_l_frame, textvariable=self.speed_l,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").pack(anchor="w")

        # Speed R
        speed_r_frame = tk.Frame(car_panel, bg=BG)
        speed_r_frame.grid(row=2, column=3, rowspan=2, padx=6)
        tk.Label(speed_r_frame, text="Speed R",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).pack(anchor="w")
        tk.Label(speed_r_frame, textvariable=self.speed_r,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").pack(anchor="w")

        # Target Speed
        tk.Label(car_panel, text="Target Speed:",
                 bg=BG, fg=TEXT_DIM, font=("Arial", 8)).grid(
                 row=3, column=1, pady=(4,0))
        tk.Label(car_panel, textvariable=self.tgt_speed,
                 bg=BG, fg=ACCENT, font=("Arial", 9,"bold"),
                 width=6, anchor="w").grid(
                 row=3, column=2, pady=(4,0))

    def _draw_car_icon(self, canvas):
        canvas.delete("all")
        # Simple top-down car silhouette
        W, H = 100, 120
        # Body
        canvas.create_rounded_rect = lambda *a, **kw: None  # helper
        canvas.create_rectangle(20, 15, 80, 105, fill=ACCENT, outline=BORDER, width=1)
        canvas.create_rectangle(25, 25, 75, 50, fill="#bbdefb", outline=BORDER)
        canvas.create_rectangle(25, 70, 75, 95, fill="#bbdefb", outline=BORDER)
        for x, y in [(12,22), (72,22), (12,82), (72,82)]:
            canvas.create_rectangle(x, y, x+16, y+20,
                                    fill="#455a64", outline=BORDER)
        canvas.create_polygon(50,8, 44,18, 56,18, fill=BTN_START, outline="")

    # ── Control buttons ───────────────────────────────────────────────────────
    def _build_control_buttons(self, parent):
        frame = tk.LabelFrame(parent,
                                text="  CONTROL",
                                bg=BG, fg=ACCENT,
                                font=("Arial", 9, "bold"),
                                relief="flat", bd=1,
                                highlightbackground=BORDER, highlightthickness=0)
        frame.pack(fill="x")

        btns = [
            ("START  ▶",  BTN_START, "white", self._on_start),
            ("PAUSE ❚❚",   BTN_STOP,  "white", self._on_pause),
            ("RESET  ↺ ", BTN_RESET, "white", self._on_reset),
            ("SHOW RAW",   BTN_VIEW,  "white", self._on_show_raw),
            ("CLOSE RAW",  BTN_VIEW,  "white", self._on_close_raw)
        ]
        for txt, bg_col, fg_col, cmd in btns:
            b = tk.Button(frame, text=txt, bg=bg_col, fg=fg_col,
                          activebackground=ACCENT, activeforeground=fg_col,
                          font=("Arial", 9, "bold"),
                          relief="raised", cursor="hand2",
                          pady=5, width=12, command=cmd)
            b.pack(fill="x", padx=0, pady=2)

    # ── View buttons ──────────────────────────────────────────────────────────
    def _build_view_buttons(self, parent):
        frame = tk.LabelFrame(parent,
                                text="  VIEW",
                                bg=BG, fg=ACCENT,
                                font=("Arial", 9, "bold"),
                                relief="flat", bd=1,
                                highlightbackground=BORDER, highlightthickness=0)

        frame.pack(fill="x")

        views = [
            ("GLOBAL_S",  self._view_global_static),
            ("GLOBAL_T",  self._view_global_track),
            ("FPV",     self._view_first),
            ("TPV",     self._view_third),
        ]
        for txt, cmd in views:
            b = tk.Button(frame, text=txt, bg=BTN_VIEW, fg="white",
                          activebackground=ACCENT, activeforeground="white",
                          font=("Arial", 10, "bold"),
                          relief="raised", cursor="hand2",
                          pady=7, width=12, command=cmd)
            b.pack(fill="x", padx=(0, 0), pady=2)

    # ── Parameter sliders ─────────────────────────────────────────────────────
    def _build_param_sliders(self, parent):
        # ── Control parameters ────────────────────────────────────────────────
        ctrl_frame = tk.LabelFrame(parent,
                                   text="  CONTROL PARAMETERS",
                                   bg=PANEL, fg=ACCENT,
                                   font=("Arial", 9, "bold"),
                                   relief="flat", bd=1,
                                   highlightbackground=BORDER, highlightthickness=1)

        ctrl_frame.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)
        ctrl_frame.columnconfigure(0, weight=1)


        ctrl_params = [
            ("STEER_KP",                    "float",    "Steer Kp",                   0,    0.05,     0.0074, 4),
            ("STEER_KD",                    "float",    "Steer Kd",                   0,    0.05,     0.005,  4),
            ("CROSS_LOCK_ERROR_DECAY_EXP",  "float",    "Cross Lock Error Decay Exp", 0,    3,        0.5,    2),
            ("MAX_STEER_RATE_DEGREE",       "float",    "Max Steer Rate (°/s)",       0,    720,      360,    2),
            ("MAX_SPEED_UP_RATE_MM",        "float",    "Max Speed Up Rate (mm/s^2)", 0,    5000,     2000,   0),
            ("MAX_SPEED_DOWN_RATE_MM",      "float",    "Max Brake Rate (mm/s^2)",    0,    5000,     3500,   0),
            ("STRAIGHT_SPEED_MM",           "float",    "Straight Speed (mm/s)",      0,    4000,     2500,   0),
            ("TURN_SPEED_MM",               "float",    "Turn Speed     (mm/s)",      0,    4000,     1600,   0),
            ("CROSS_SPEED_MM",              "float",    "Cross Speed    (mm/s)",      0,    4000,     1600,   0),
            ("STOP_DISTANCE_MM",            "float",    "Stop Distance  (mm)",        0,    600,      250,    0),
        ]


        sens_params = [
            ("BLACK_THRESHOLD",             "int",      "Black Threshold",             0, 255, 15,     0),
            ("WHITE_THRESHOLD",             "int",      "White Threshold",             0, 255, 225,    0),
            ("STOP_LIGHT_THRESHOLD",        "int",      "Stop Light Threshold",        0, 255, 50,     0),
            ("STRAIGHT_CENTER_ERROR_THRESHOLD",   "int", "Straight center error Threshold",     0, 255, 16,     0),
            ("MIN_LIGHT_WIDTH_RATIO",       "float",    "Min Light Width Ratio",       0, 1,   0.08,   2),
            ("STOP_LINE_WIDTH_RATIO",       "float",    "Stop Line Width Ratio",       0, 1,   0.35,   2),
            ("EDGE_LIGHT_THRESHOLD_RATIO",  "float",    "Edge Light Threshold Ratio",  0, 1,   0.25,   2),
        ]

        def make_ctrl_callback(ros_name, ros_type):
            def callback(v):
                self.node.set_parameter(self.node.ctrl_param_client, ros_name, v, ros_type)
            return callback

        self.ctrl_sliders = {}
        for i, (para_name, value_type, show_name, lo, hi, default, resolution) in enumerate(ctrl_params):
            callback = make_ctrl_callback(para_name, value_type)
            s = ParamSlider(ctrl_frame, show_name, lo, hi, default, resolution, value_type, callback)
            s.grid(row=i, column=0, sticky="ew", padx=0, pady=0)
            self.ctrl_sliders[para_name] = s

        # ── Sensor parameters ─────────────────────────────────────────────────
        sens_frame = tk.LabelFrame(parent,
                                   text="  SENSOR PARAMETERS",
                                   bg=PANEL, fg=ACCENT,
                                   font=("Arial", 9, "bold"),
                                   relief="flat", bd=1,
                                   highlightbackground=BORDER, highlightthickness=1)
        sens_frame.grid(row=0, column=1, sticky="nsew", pady=0)

        sens_frame.columnconfigure(0, weight=1)


        def make_sens_callback(ros_name, ros_type):
            def callback(v):
                self.node.set_parameter(self.node.sens_param_client, ros_name, v, ros_type)
            return callback

        self.sens_sliders = {}
        for i, (para_name, value_type, show_name, lo, hi, default, resolution) in enumerate(sens_params):
            callback = make_sens_callback(para_name, value_type)
            s = ParamSlider(sens_frame, show_name, lo, hi, default, resolution, value_type, callback)
            s.grid(row=i, column=0, sticky="ew", padx=0, pady=0)
            self.sens_sliders[para_name] = s

    # ── Button callbacks ──────────────────────────────────────────────────────
    def _on_show_raw(self):
        self.status_msg.set("● RAW CCD VIEW OPEN")
        self.node.unpause_physics()

        # ros control
        if not self.node:
            return
        self.node.send_gui_control(GuiControl.SHOW_IMAGE)

    def _on_close_raw(self):
        self.status_msg.set("● RAW CCD VIEW CLOSED")

        # ros control
        if not self.node:
            return
        self.node.send_gui_control(GuiControl.HIDE_IMAGE)


    def _on_start(self):
        self.status_msg.set("● RUNNING – autonomous mode active")

        # ros control
        if not self.node:
            return
        self.system_paused = False
        self.node.running.set()
        self.node.send_gui_control(GuiControl.START)
        self.node.unpause_physics()

    def _on_pause(self):
        self.status_msg.set("❚❚ PAUSED")

        # ros control
        if not self.node:
            return
        self.system_paused = True
        self.node.running.clear()
        self.node.send_gui_control(GuiControl.PAUSE)
        self.node.pause_physics()

    def _on_reset(self):
        self.status_msg.set("↺ RESETTING – car returning to spawn …")

        # ros control
        if not self.node:
            return
        self.system_paused = True
        self.node.running.clear()

        self.node.on_reset()  # reset internal state first

        self.speed_l.set(0.0)
        self.speed_r.set(0.0)
        self.steer_l.set(0.0)
        self.steer_r.set(0.0)
        self.output_speed.set(0.0)
        self.road_type.set("----")
        self.tgt_speed.set(0.0)
        self.tgt_steer.set(0.0)
        self.speedo.set(0, self.road_type.get())

        init_ccd_bar = [0]*128

        self.ccd_far_bar.update_data(init_ccd_bar, 0, 63, 127)
        self.ccd_far_road_width.set("---")
        self.ccd_far_avg.set("---")
        self.ccd_far_edge_L.set("-")
        self.ccd_far_center.set("-")
        self.ccd_far_edge_R.set("-")

        self.ccd_close_bar.update_data(init_ccd_bar, 0, 63, 127)
        self.ccd_close_road_width.set("---")
        self.ccd_close_avg.set("---")
        self.ccd_close_edge_L.set("-")
        self.ccd_close_center.set("-")
        self.ccd_close_edge_R.set("-")


        self.node.send_gui_control(GuiControl.RESET)
        self.node.set_pose()
        time.sleep(0.5)  # wait a moment for the reset to take effect
        self.node.pause_physics()  # pause after reset to prevent car from moving immediately


    def _view_global_static(self):
        self.status_msg.set("⬡ View: GLOBAL_S")
        #print("[VIEW] global")

        # ros control
        if self.node:
            self.node.send_view_cmd("global")

    def _view_global_track(self):
        self.status_msg.set("⬢ View: GLOBAL_T")
        #print("[VIEW] global track")

        # ros control
        if self.node:
            self.node.send_view_cmd("global_track")

    def _view_first(self):
        self.status_msg.set("◎ View: FIRST PERSON")
        #print("[VIEW] first-person")

        # ros control
        if self.node:
            self.node.send_view_cmd("first")

    def _view_third(self):
        self.status_msg.set("◈ View: THIRD PERSON")
        #print("[VIEW] third-person")

        # ros control
        if self.node:
            self.node.send_view_cmd("third")



    # ── Demo data loop (replace with ROS2 callbacks) ─────────────────────────
    def _start_demo_loop(self):
        self._t = 0.0
        if self.node:
            self._ros_tick()   # start ROS2 data loop
        else:
            self._demo_tick()

    def _demo_tick(self):
        self._t += 0.05
        t = self._t

        # Simulate sensor data
        sl = 800 + 200 * math.sin(t * 0.7)
        sr = 800 + 200 * math.sin(t * 0.7 + 0.2)
        steer = 12 * math.sin(t * 0.4)
        center = int(15 * math.sin(t * 0.4))

        self.speed_l.set(round(sl, 2))
        self.speed_r.set(round(sr, 2))
        self.steer_l.set(round(steer, 2))
        self.steer_r.set(round(steer, 2))
        self.tgt_speed.set(round((sl + sr) / 2, 1))
        self.tgt_steer.set(round(steer, 2))


        road_types = ["STRAIGHT", "TURN", "CROSS", "STOP", "BLACK"]
        rt_idx = int(t / 3) % len(road_types)
        self.road_type.set(road_types[rt_idx])

        self.speedo.set((sl + sr) / 2, self.road_type.get())

        # CCD far – simulate road detection
        ccd_far = [int(200 - 80 * math.exp(-((i-64+center*2)**2)/800))
                   for i in range(128)]
        el = max(0, 30 + center)
        ec = 64 + center
        er = min(127, 95 + center)
        self.ccd_far_bar.update_data(ccd_far, el, ec, er)
        self.ccd_far_road_width.set(str(er - el))
        self.ccd_far_avg.set(str(int(sum(ccd_far)/128)))
        self.ccd_far_edge_L.set(str(el))
        self.ccd_far_center.set(str(ec))
        self.ccd_far_edge_R.set(str(er))

        # CCD close
        ccd_close = [int(180 - 60 * math.exp(-((i-64+center)**2)/600))
                     for i in range(128)]
        el2 = max(0, 35 + center)
        ec2 = 64 + center
        er2 = min(127, 92 + center)
        self.ccd_close_bar.update_data(ccd_close, el2, ec2, er2)
        self.ccd_close_road_width.set(str(er2 - el2))
        self.ccd_close_avg.set(str(int(sum(ccd_close)/128)))
        self.ccd_close_edge_L.set(str(el2))
        self.ccd_close_center.set(str(ec2))
        self.ccd_close_edge_R.set(str(er2))

        self.after(self.refresh_rate, self._demo_tick)   # ~20 Hz   50ms

    def _ros_tick(self):
        # ccd detection imge and data
        detect_msg = self.node.ccd_detection_msg        # Replace with actual method to get latest data

        if self.system_paused:
            self.after(self.refresh_rate, self._ros_tick)
            return
        if detect_msg:
            # CCD far – simulate road detection
            self.ccd_far_bar.update_data(
                detect_msg.far_img,
                detect_msg.far_left_edge,
                detect_msg.far_center,
                detect_msg.far_right_edge)
            self.ccd_far_road_width.set(str(detect_msg.far_road_width))
            self.ccd_far_avg.set(str(detect_msg.far_avg_brightness))
            self.ccd_far_edge_L.set(str(detect_msg.far_left_edge if detect_msg.far_left_edge > 0 else "-"))
            self.ccd_far_center.set(str(detect_msg.far_center))
            self.ccd_far_edge_R.set(str(detect_msg.far_right_edge if detect_msg.far_right_edge > 0 else "-"))

            # CCD close
            self.ccd_close_bar.update_data(
                detect_msg.close_img,
                detect_msg.close_left_edge,
                detect_msg.close_center,
                detect_msg.close_right_edge
            )
            self.ccd_close_road_width.set(str(detect_msg.close_road_width))
            self.ccd_close_avg.set(str(detect_msg.close_avg_brightness))
            self.ccd_close_edge_L.set(str(detect_msg.close_left_edge if detect_msg.close_left_edge > 0 else "-"))
            self.ccd_close_center.set(str(detect_msg.close_center))
            self.ccd_close_edge_R.set(str(detect_msg.close_right_edge if detect_msg.close_right_edge > 0 else "-"))

        # Update car info
        car_info_msg = self.node.car_info_msg        # Replace with actual method to get latest data
        if car_info_msg:
            radius = 32  # wheel diameter in mm, adjust if different
            self.steer_l.set(round(math.degrees(car_info_msg.steer_left), 2))
            self.steer_r.set(round(math.degrees(car_info_msg.steer_right), 2))
            self.speed_l.set(round(car_info_msg.speed_left * radius, 2))   # rad/s * mm = mm/s
            self.speed_r.set(round(car_info_msg.speed_right * radius, 2))
            self.output_speed.set(round(car_info_msg.output_speed * radius, 2))

            self.tgt_steer.set(round(math.degrees(car_info_msg.target_steer), 2))
            self.tgt_speed.set(round(car_info_msg.target_speed * radius, 2))

            road_types = ["STRAIGHT", "TURN", "CROSS", "STOP", "BLACK"]
            rt_idx = car_info_msg.road_type                       # Replace with actual road type index from message
            self.road_type.set(road_types[rt_idx] if rt_idx >=0 and rt_idx < len(road_types) else "----")

        self.speedo.set(self.output_speed.get(), self.road_type.get())

        self.after(self.refresh_rate, self._ros_tick)   # ~20 Hz   50ms

# =========================
# main
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros2", action="store_true",
                        help="Enable ros2 to receive real data instead of demo mode")
    args, _ = parser.parse_known_args()

    node = None
    if args.ros2:
        # init ROS and create node here, pass node to Car
        rclpy.init()
        node = MonitorNode()  # Replace with actual ROS2 node initialization

        ros_thread = threading.Thread(
            target = rclpy.spin,
            args=(node,),
        )

        ros_thread.start()
    else:
        print("Demo model enabled – no ROS2 connection")

    gui = CarDashboard(node)
    # Window size hint
    gui.geometry("800x840")
    gui.mainloop()

    if args.ros2:
        ros_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()



# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    main()
