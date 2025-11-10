# common/gait_features.py
import os, csv, math, time
import numpy as np

class GaitFeatureExtractor:
    """
    Ekstraksi fitur gait real-time dari stream point cloud radar.
    Menulis ke {out_dir}/gait_features.csv setiap frame.
    Fitur: centroid (cx,cy,cz), v_walk, step_event, step_time, step_length, step_count, cadence_spm
    """

    def __init__(self, out_dir, axis_forward='x', use_signal='z', min_step_interval=0.28,smooth_alpha=0.2, fps_hint=None, csv_name="gait_features.csv"):
        self.out_dir = out_dir
        os.makedirs(self.out_dir, exist_ok=True)
        self.fpath = os.path.join(self.out_dir, csv_name)

        # konfigurasi
        self.axis_forward = axis_forward   # 'x' atau 'y'
        self.use_signal   = use_signal     # 'z' (minima) atau 'v' (peak v_forward)
        self.min_step_interval = float(min_step_interval)
        self.alpha = float(smooth_alpha)   # EMA smoothing
        self.fps_hint = fps_hint

        # state
        self.prev_centroid = None
        self.prev_t = None
        self.v_smooth = None
        self.z_smooth = None
        self._prev_s = None
        self._prev_ds = 0.0
        self._ema = None
        self._ema_var = 0.0

        self.step_times = []
        self.step_timestamps = []
        self.n_steps_total = 0
        self.last_step_ts = -1e9

        # header CSV
        if not os.path.isfile(self.fpath):
            with open(self.fpath, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp","frame","cx","cy","cz",
                            "v_walk","step_event","step_time","step_length",
                            "step_count","cadence_spm"])

    # ---------- util ----------
    @staticmethod
    def _robust_centroid(pc_xyz):
        if pc_xyz is None:
            return None
        pc_xyz = np.asarray(pc_xyz)
        if pc_xyz.size == 0:
            return None
        return np.median(pc_xyz, axis=0)

    @staticmethod
    def _fmt(x):
        try:
            if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
                return ""
            return f"{float(x):.6f}"
        except Exception:
            return ""

    def _cadence_from_window(self, now_ts, window_sec=10.0):
        if len(self.step_timestamps) == 0:
            return 0.0
        lo = now_ts - window_sec
        recent = [t for t in self.step_timestamps if t >= lo]
        duration = min(window_sec, max(1e-6, now_ts - recent[0])) if recent else window_sec
        return 60.0 * (len(recent) / duration) if duration > 0 else 0.0

    def _write_row(self, ts, frame, cx, cy, cz, v_walk, step_event, step_time, step_len,step_count, cadence_spm):
        with open(self.fpath, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([f"{ts:.6f}",
                        int(frame) if frame is not None else "",
                        self._fmt(cx), self._fmt(cy), self._fmt(cz),
                        self._fmt(v_walk), int(step_event),
                        self._fmt(step_time), self._fmt(step_len),
                        int(step_count), self._fmt(cadence_spm)])

    # ---------- API utama ----------
    def update(self, outputDict):
        """
        Panggil sekali per frame dengan outputDict dari parser.
        Diharapkan keys:
          - 'pointCloud' : ndarray (N, >=5) [x,y,z,doppler,SNR,...]
          - 'frameNum'   : int
          - 'timeStamp'  : float detik (opsional) atau 'unixTimeMs' : int ms
        """
        # timestamp
        ts = None
        if outputDict.get('timeStamp') is not None:
            ts = float(outputDict['timeStamp'])
        elif outputDict.get('unixTimeMs') is not None:
            ts = float(outputDict['unixTimeMs']) / 1000.0
        else:
            ts = time.time()
        frame = int(outputDict.get('frameNum', -1))

        pc = outputDict.get('pointCloud', None)
        if pc is None or len(pc) == 0:
            # tulis baris kosong utk menjaga basis waktu
            self._write_row(ts, frame, np.nan, np.nan, np.nan, np.nan,0, np.nan, np.nan, self.n_steps_total, self._cadence_from_window(ts))
            self.prev_t = ts
            return

        # centroid
        pc_xyz = np.asarray(pc)[:, :3]
        c = self._robust_centroid(pc_xyz)
        if c is None:
            self._write_row(ts, frame, np.nan, np.nan, np.nan, np.nan,
                            0, np.nan, np.nan, self.n_steps_total, self._cadence_from_window(ts))
            self.prev_t = ts
            return
        cx, cy, cz = float(c[0]), float(c[1]), float(c[2])

        # dt
        dt = None
        if self.prev_t is None:
            dt = 1.0 / self.fps_hint if self.fps_hint else np.nan
        else:
            dt = max(1e-6, ts - self.prev_t)

        v_walk = np.nan
        step_len = np.nan
        step_time = np.nan
        step_event = 0

        if self.prev_centroid is not None and not np.isnan(dt):
            dx = cx - self.prev_centroid[0]
            dy = cy - self.prev_centroid[1]
            dz = cz - self.prev_centroid[2]
            # kecepatan planar (x-y)
            v_walk = math.sqrt(dx*dx + dy*dy) / dt

            # forward component utk deteksi langkah
            v_forward = abs(dx) if self.axis_forward == 'x' else abs(dy)

            # smoothing
            self.v_smooth = v_forward if self.v_smooth is None else (self.alpha*v_forward + (1-self.alpha)*self.v_smooth)
            self.z_smooth = cz        if self.z_smooth is None else (self.alpha*cz        + (1-self.alpha)*self.z_smooth)

            s_t = self.z_smooth if self.use_signal == 'z' else self.v_smooth

            if self._ema is None:
                self._ema = s_t
            prev_s = self._prev_s if self._prev_s is not None else s_t
            ds = s_t - prev_s
            # update statistik untuk threshold adaptif
            self._ema = 0.9*self._ema + 0.1*s_t
            self._ema_var = 0.9*self._ema_var + 0.1*((s_t - self._ema)**2)
            sigma = math.sqrt(max(self._ema_var, 1e-9))
            refractory_ok = (ts - self.last_step_ts) >= self.min_step_interval

            # deteksi ekstremum
            if self.use_signal == 'z':
                # heel-strike ~ minima z: derivatif dari negatif ke positif + prominence
                is_extremum = (self._prev_ds < 0 and ds > 0 and (self._ema - s_t) > 0.5*sigma)
            else:
                # peak v_forward: derivatif dari positif ke negatif + prominence
                is_extremum = (self._prev_ds > 0 and ds < 0 and (s_t - self._ema) > 0.5*sigma)

            if is_extremum and refractory_ok:
                step_event = 1
                if len(self.step_timestamps) > 0:
                    step_time = ts - self.step_timestamps[-1]
                if step_time and not np.isnan(v_walk):
                    step_len = v_walk * step_time
                self.step_timestamps.append(ts)
                self.n_steps_total += 1
                self.last_step_ts = ts

            self._prev_ds = ds
            self._prev_s = s_t

        cadence_spm = self._cadence_from_window(ts)
        self._write_row(ts, frame, cx, cy, cz, v_walk, step_event, step_time, step_len,
                        self.n_steps_total, cadence_spm)

        # update state
        self.prev_centroid = (cx, cy, cz)
        self.prev_t = ts
