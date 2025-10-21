"""
ROI and Cup Points Selection Tool
Click to select ROI polygon and cup positions, automatically update config.py
"""

import cv2
import numpy as np
import os
import re
import logging

# Import our custom logging configuration
from logger_config import setup_logging, get_logger, log_error

class ROISelector:
    def __init__(self, rtsp_url: str):
        self.rtsp_url = rtsp_url
        self.roi_points = []
        self.cup_points = []
        self.mode = "roi"  # "roi" or "cups"
        self.drawing = False
        self.frame = None
        self.cap = None
        self.zoom_factor = 1.0
        
        # Setup logging
        setup_logging(log_file="logs/app.log")
        self.logger = get_logger("ROISelector")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mode == "roi":
                self.roi_points.append((x, y))
                self.logger.warning(f"ROI Point {len(self.roi_points)}: ({x}, {y})")
            else:
                if len(self.cup_points) < 4:
                    self.cup_points.append((x, y))
                    self.logger.warning(f"Cup Position {len(self.cup_points)}: ({x}, {y})")
                else:
                    self.logger.warning("Maximum 4 cup positions allowed!")
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            if self.mode == "roi" and self.roi_points:
                temp = self.frame.copy()
                self.draw_all(temp)
                cv2.line(temp, self.roi_points[-1], (x, y), (0, 255, 0), 2)
                cv2.imshow('ROI & Cup Selector', temp)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False

    def draw_all(self, frame):
        # ROI points & polygon
        for i, pt in enumerate(self.roi_points):
            cv2.circle(frame, pt, 8, (0, 0, 255), -1)
            cv2.putText(frame, f"R{i+1}", (pt[0]+10, pt[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        if len(self.roi_points) >= 3:
            pts = np.array(self.roi_points, np.int32)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

        # Cup points
        for i, pt in enumerate(self.cup_points):
            cv2.circle(frame, pt, 6, (255, 0, 0), -1)
            cv2.putText(frame, f"C{i+1}", (pt[0]+10, pt[1]+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    def update_config_file(self):
        cfg = "config.py"
        if not os.path.exists(cfg):
            log_error(self.logger, f"Config file not found: {cfg}")
            return False

        with open(cfg, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read()

        if len(self.roi_points) >= 3:
            roi_str = "np.array([\n" + "".join(
                f"    [{x}, {y}],\n" for x, y in self.roi_points
            ).rstrip(",\n") + "\n], dtype=np.int32)"

            # Match either np.array([...], dtype=np.int32) OR a plain list [...]
            content = re.sub(
                r'ROI_POLYGON\s*=\s*(?:np\.array\(\[.*?\],\s*dtype=np\.int32\)|\[[^\]]*\])',
                f'ROI_POLYGON = {roi_str}',
                content, flags=re.DOTALL
            )

            # Ensure import exists (if someone removed it)
            if "import numpy as np" not in content:
                content = content.replace(
                    '"""', '"""'  # keep your module docstring intact
                )
                content = 'import numpy as np\n' + content

        # CUP_POSITIONS
        if self.cup_points:
            cups_str = "[\n" + "".join(
                f"    ({x}, {y}),\n" for x, y in self.cup_points
            ).rstrip(",\n") + "\n]"
            content = re.sub(
                r'CUP_POSITIONS = \[.*?\]',
                f'CUP_POSITIONS = {cups_str}',
                content, flags=re.DOTALL
            )

        with open(cfg, "w", encoding="utf-8") as f:
            f.write(content)

        self.logger.warning(f"Updated {cfg}!")
        if len(self.roi_points) >= 3:
            self.logger.warning(f"ROI: {len(self.roi_points)} points")
        if self.cup_points:
            self.logger.warning(f"Cup positions: {len(self.cup_points)} points")
        return True

    def run(self):
        self.logger.warning("ROI & Cup Points Selection Tool started")
        self.logger.warning("Controls: 1=ROI, 2=Cups, s=Save, f=Info, q=Quit, r=Reset current mode")

        # Quick camera probe
        try:
            test_cap = cv2.VideoCapture(self.rtsp_url)
            if not test_cap.isOpened():
                log_error(self.logger, f"Could not connect to RTSP: {self.rtsp_url}")
                return
            ok, test_frame = test_cap.read()
            if ok and test_frame is not None:
                h, w = test_frame.shape[:2]
                self.logger.warning(f"Camera resolution: {w}x{h}")
            else:
                log_error(self.logger, "Could not read frame from RTSP stream")
                test_cap.release()
                return
            test_cap.release()
        except Exception as e:
            log_error(self.logger, f"RTSP error: {e}")
            return

        cv2.namedWindow('ROI & Cup Selector', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('ROI & Cup Selector', self.mouse_callback)

        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            log_error(self.logger, f"Could not open RTSP stream: {self.rtsp_url}")
            return
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        while True:
            ok, self.frame = self.cap.read()
            if not ok or self.frame is None:
                log_error(self.logger, "Could not read frame from RTSP stream")
                break

            disp = self.frame.copy()
            self.draw_all(disp)

            # Apply zoom if needed
            if self.zoom_factor != 1.0:
                h, w = disp.shape[:2]
                new_h, new_w = int(h * self.zoom_factor), int(w * self.zoom_factor)
                disp = cv2.resize(disp, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

            mode_text = "ROI Mode" if self.mode == "roi" else "Cup Mode"
            h, w = disp.shape[:2]
            cv2.putText(disp, f"Mode: {mode_text}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(disp, "1=ROI  2=Cups  s=Save  f=Info  q=Quit",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(disp, f"ROI: {len(self.roi_points)} | Cups: {len(self.cup_points)}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(disp, f"Resolution: {w}x{h} | Zoom: {self.zoom_factor:.1f}x",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            cv2.imshow('ROI & Cup Selector', disp)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('1'):
                self.mode = "roi"; self.logger.warning("Switched to ROI mode")
            elif key == ord('2'):
                self.mode = "cups"; self.logger.warning("Switched to Cup mode")
            elif key == ord('r'):
                if self.mode == "roi":
                    self.roi_points = []; self.logger.warning("ROI points reset!")
                else:
                    self.cup_points = []; self.logger.warning("Cup points reset!")
            elif key == ord('s'):
                self.update_config_file()
            elif key == ord('f'):
                self.logger.warning(f"Frame Info - Resolution: {w}x{h}, ROI points: {self.roi_points}, Cup points: {self.cup_points}")
            elif key == ord('+') or key == ord('='):
                self.zoom_factor = min(self.zoom_factor + 0.2, 3.0)
                self.logger.warning(f"Zoom: {self.zoom_factor:.1f}x")
            elif key == ord('-'):
                self.zoom_factor = max(self.zoom_factor - 0.2, 0.5)
                self.logger.warning(f"Zoom: {self.zoom_factor:.1f}x")
            elif key == 122:  # F11 key
                # Toggle fullscreen
                current_prop = cv2.getWindowProperty('ROI & Cup Selector', cv2.WND_PROP_FULLSCREEN)
                if current_prop == cv2.WINDOW_FULLSCREEN:
                    cv2.setWindowProperty('ROI & Cup Selector', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                    self.logger.warning("Exited fullscreen")
                else:
                    cv2.setWindowProperty('ROI & Cup Selector', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                    self.logger.warning("Entered fullscreen")

        if self.cap is not None: self.cap.release()
        cv2.destroyAllWindows()
        self.logger.warning("Selection completed!")

def main():
    # Read RTSP from config if present
    rtsp_url = "rtsp://admin:QSS2030QSS@192.168.200.16:554/stream1"
    # Setup logging for main function
    setup_logging(log_file="logs/app.log")
    logger = get_logger("ROISelectorMain")
    
    try:
        with open("config.py", "r", encoding="utf-8", errors="ignore") as f:
            m = re.search(r'RTSP_URL\s*=\s*"([^"]+)"', f.read())
            if m:
                rtsp_url = m.group(1)
                logger.warning(f"Using RTSP URL from config.py: {rtsp_url}")
    except Exception:
        logger.warning(f"Using default RTSP URL: {rtsp_url}")
    ROISelector(rtsp_url).run()

if __name__ == "__main__":
    main()