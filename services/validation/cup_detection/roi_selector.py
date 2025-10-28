"""
ROI and Cup Points Selection Tool
Click to select ROI polygon and cup positions, automatically update config.py
"""

import cv2
import numpy as np
import os
import re


# Import our custom logging configuration
from logger_config import setup_logging, get_logger, log_error

class ROISelector:
    def __init__(self, rtsp_url: str):
        self.rtsp_url = rtsp_url
        # Table (original) ROI and cup points
        self.table_roi_points = []
        self.table_cup_points = []
        # Milk dispenser ROI and cup points
        self.milk_roi_points = []
        self.milk_cup_points = []
        # Sauce dispenser ROI and cup points
        self.sauce_roi_points = []
        self.sauce_cup_points = []
        self.mode = "table_roi"  # "table_roi", "table_cups", "milk_roi", "milk_cups", "sauce_roi", "sauce_cups"
        self.drawing = False
        self.frame = None
        self.cap = None
        self.zoom_factor = 1.0
        
        # Setup logging
        setup_logging(log_file="logs/app.log")
        self.logger = get_logger("ROISelector")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mode == "table_roi":
                self.table_roi_points.append((x, y))
                self.logger.warning(f"Table ROI Point {len(self.table_roi_points)}: ({x}, {y})")
            elif self.mode == "table_cups":
                if len(self.table_cup_points) < 4:
                    self.table_cup_points.append((x, y))
                    self.logger.warning(f"Table Cup Position {len(self.table_cup_points)}: ({x}, {y})")
                else:
                    self.logger.warning("Maximum 4 table cup positions allowed!")
            elif self.mode == "milk_roi":
                self.milk_roi_points.append((x, y))
                self.logger.warning(f"Milk ROI Point {len(self.milk_roi_points)}: ({x}, {y})")
            elif self.mode == "milk_cups":
                if len(self.milk_cup_points) < 1:
                    self.milk_cup_points.append((x, y))
                    self.logger.warning(f"Milk Cup Position: ({x}, {y})")
                else:
                    self.logger.warning("Only 1 milk cup position allowed!")
            elif self.mode == "sauce_roi":
                self.sauce_roi_points.append((x, y))
                self.logger.warning(f"Sauce ROI Point {len(self.sauce_roi_points)}: ({x}, {y})")
            elif self.mode == "sauce_cups":
                if len(self.sauce_cup_points) < 1:
                    self.sauce_cup_points.append((x, y))
                    self.logger.warning(f"Sauce Cup Position: ({x}, {y})")
                else:
                    self.logger.warning("Only 1 sauce cup position allowed!")
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            temp = self.frame.copy()
            self.draw_all(temp)
            if self.mode == "table_roi" and self.table_roi_points:
                cv2.line(temp, self.table_roi_points[-1], (x, y), (0, 255, 0), 2)
            elif self.mode == "milk_roi" and self.milk_roi_points:
                cv2.line(temp, self.milk_roi_points[-1], (x, y), (0, 255, 0), 2)
            elif self.mode == "sauce_roi" and self.sauce_roi_points:
                cv2.line(temp, self.sauce_roi_points[-1], (x, y), (0, 255, 0), 2)
            cv2.imshow('ROI & Cup Selector', temp)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False

    def draw_all(self, frame):
        # Table ROI points & polygon (green)
        for i, pt in enumerate(self.table_roi_points):
            cv2.circle(frame, pt, 8, (0, 255, 0), -1)
            cv2.putText(frame, f"TR{i+1}", (pt[0]+10, pt[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if len(self.table_roi_points) >= 3:
            pts = np.array(self.table_roi_points, np.int32)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

        # Milk ROI points & polygon (red)
        for i, pt in enumerate(self.milk_roi_points):
            cv2.circle(frame, pt, 8, (0, 0, 255), -1)
            cv2.putText(frame, f"MR{i+1}", (pt[0]+10, pt[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        if len(self.milk_roi_points) >= 3:
            pts = np.array(self.milk_roi_points, np.int32)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

        # Sauce ROI points & polygon (purple)
        for i, pt in enumerate(self.sauce_roi_points):
            cv2.circle(frame, pt, 8, (128, 0, 128), -1)
            cv2.putText(frame, f"SR{i+1}", (pt[0]+10, pt[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 0, 128), 2)
        if len(self.sauce_roi_points) >= 3:
            pts = np.array(self.sauce_roi_points, np.int32)
            cv2.polylines(frame, [pts], True, (255, 0, 255), 2)

        # Table cup points (white)
        for i, pt in enumerate(self.table_cup_points):
            cv2.circle(frame, pt, 6, (255, 255, 255), -1)
            cv2.putText(frame, f"TC{i+1}", (pt[0]+10, pt[1]+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Milk cup points (blue)
        for i, pt in enumerate(self.milk_cup_points):
            cv2.circle(frame, pt, 6, (255, 0, 0), -1)
            cv2.putText(frame, f"MC{i+1}", (pt[0]+10, pt[1]+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Sauce cup points (orange)
        for i, pt in enumerate(self.sauce_cup_points):
            cv2.circle(frame, pt, 6, (0, 165, 255), -1)
            cv2.putText(frame, f"SC{i+1}", (pt[0]+10, pt[1]+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    def update_config_file(self):
        cfg = "config.py"
        if not os.path.exists(cfg):
            log_error(self.logger, f"Config file not found: {cfg}")
            return False

        with open(cfg, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read()

        # Update Table ROI (original)
        if len(self.table_roi_points) >= 3:
            table_roi_str = "np.array([\n" + "".join(
                f"    [{x}, {y}],\n" for x, y in self.table_roi_points
            ).rstrip(",\n") + "\n], dtype=np.int32)"

            content = re.sub(
                r'ROI_POLYGON\s*=\s*(?:np\.array\(\[.*?\],\s*dtype=np\.int32\)|\[[^\]]*\])',
                f'ROI_POLYGON = {table_roi_str}',
                content, flags=re.DOTALL
            )

        # Update Table Cup Positions (original)
        if self.table_cup_points:
            table_cups_str = "[\n" + "".join(
                f"    ({x}, {y}),\n" for x, y in self.table_cup_points
            ).rstrip(",\n") + "\n]"
            content = re.sub(
                r'CUP_POSITIONS = \[.*?\]',
                f'CUP_POSITIONS = {table_cups_str}',
                content, flags=re.DOTALL
            )

        # Update Milk ROI
        if len(self.milk_roi_points) >= 3:
            milk_roi_str = "np.array([\n" + "".join(
                f"    [{x}, {y}],\n" for x, y in self.milk_roi_points
            ).rstrip(",\n") + "\n], dtype=np.int32)"

            content = re.sub(
                r'MILK_ROI_POLYGON\s*=\s*(?:np\.array\(\[.*?\],\s*dtype=np\.int32\)|\[[^\]]*\])',
                f'MILK_ROI_POLYGON = {milk_roi_str}',
                content, flags=re.DOTALL
            )

        # Update Sauce ROI
        if len(self.sauce_roi_points) >= 3:
            sauce_roi_str = "np.array([\n" + "".join(
                f"    [{x}, {y}],\n" for x, y in self.sauce_roi_points
            ).rstrip(",\n") + "\n], dtype=np.int32)"

            content = re.sub(
                r'SAUCE_ROI_POLYGON\s*=\s*(?:np\.array\(\[.*?\],\s*dtype=np\.int32\)|\[[^\]]*\])',
                f'SAUCE_ROI_POLYGON = {sauce_roi_str}',
                content, flags=re.DOTALL
            )

        # Update Table Cup Positions (original)
        if self.table_cup_points:
            table_cups_str = "[\n" + "".join(
                f"    ({x}, {y}),\n" for x, y in self.table_cup_points
            ).rstrip(",\n") + "\n]"
            content = re.sub(
                r'CUP_POSITIONS = \[.*?\]',
                f'CUP_POSITIONS = {table_cups_str}',
                content, flags=re.DOTALL
            )

        # Update Milk Cup Positions
        if self.milk_cup_points:
            milk_cups_str = "[\n" + "".join(
                f"    ({x}, {y}),\n" for x, y in self.milk_cup_points
            ).rstrip(",\n") + "\n]"
            content = re.sub(
                r'MILK_CUP_POSITIONS = \[.*?\]',
                f'MILK_CUP_POSITIONS = {milk_cups_str}',
                content, flags=re.DOTALL
            )

        # Update Sauce Cup Positions
        if self.sauce_cup_points:
            sauce_cups_str = "[\n" + "".join(
                f"    ({x}, {y}),\n" for x, y in self.sauce_cup_points
            ).rstrip(",\n") + "\n]"
            content = re.sub(
                r'SAUCE_CUP_POSITIONS = \[.*?\]',
                f'SAUCE_CUP_POSITIONS = {sauce_cups_str}',
                content, flags=re.DOTALL
            )

        # Ensure import exists (if someone removed it)
        if "import numpy as np" not in content:
            content = content.replace(
                '"""', '"""'  # keep your module docstring intact
            )
            content = 'import numpy as np\n' + content

        with open(cfg, "w", encoding="utf-8") as f:
            f.write(content)

        self.logger.warning(f"Updated {cfg}!")
        if len(self.table_roi_points) >= 3:
            self.logger.warning(f"Table ROI: {len(self.table_roi_points)} points")
        if len(self.milk_roi_points) >= 3:
            self.logger.warning(f"Milk ROI: {len(self.milk_roi_points)} points")
        if len(self.sauce_roi_points) >= 3:
            self.logger.warning(f"Sauce ROI: {len(self.sauce_roi_points)} points")
        if self.table_cup_points:
            self.logger.warning(f"Table cup positions: {len(self.table_cup_points)} points")
        if self.milk_cup_points:
            self.logger.warning(f"Milk cup positions: {len(self.milk_cup_points)} points")
        if self.sauce_cup_points:
            self.logger.warning(f"Sauce cup positions: {len(self.sauce_cup_points)} points")
        return True

    def run(self):
        self.logger.warning("Multi-Dispenser ROI & Cup Points Selection Tool started")
        self.logger.warning("Controls: 1=Table ROI, 2=Table Cups, 3=Milk ROI, 4=Milk Cups, 5=Sauce ROI, 6=Sauce Cups")
        self.logger.warning("s=Save, f=Info, q=Quit, r=Reset current mode")

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

            mode_text = {
                "table_roi": "Table ROI Mode",
                "table_cups": "Table Cups Mode",
                "milk_roi": "Milk ROI Mode",
                "milk_cups": "Milk Cups Mode", 
                "sauce_roi": "Sauce ROI Mode",
                "sauce_cups": "Sauce Cups Mode"
            }.get(self.mode, "Unknown Mode")
            
            h, w = disp.shape[:2]
            cv2.putText(disp, f"Mode: {mode_text}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(disp, "1=Table ROI  2=Table Cups  3=Milk ROI  4=Milk Cups",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(disp, "5=Sauce ROI  6=Sauce Cups  s=Save  f=Info  q=Quit  r=Reset",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(disp, f"Table ROI: {len(self.table_roi_points)} | Table Cups: {len(self.table_cup_points)}",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(disp, f"Milk ROI: {len(self.milk_roi_points)} | Milk Cups: {len(self.milk_cup_points)}",
                        (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(disp, f"Sauce ROI: {len(self.sauce_roi_points)} | Sauce Cups: {len(self.sauce_cup_points)}",
                        (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(disp, f"Resolution: {w}x{h} | Zoom: {self.zoom_factor:.1f}x",
                        (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            cv2.imshow('ROI & Cup Selector', disp)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('1'):
                self.mode = "table_roi"; self.logger.warning("Switched to Table ROI mode")
            elif key == ord('2'):
                self.mode = "table_cups"; self.logger.warning("Switched to Table Cups mode")
            elif key == ord('3'):
                self.mode = "milk_roi"; self.logger.warning("Switched to Milk ROI mode")
            elif key == ord('4'):
                self.mode = "milk_cups"; self.logger.warning("Switched to Milk Cups mode")
            elif key == ord('5'):
                self.mode = "sauce_roi"; self.logger.warning("Switched to Sauce ROI mode")
            elif key == ord('6'):
                self.mode = "sauce_cups"; self.logger.warning("Switched to Sauce Cups mode")
            elif key == ord('r'):
                if self.mode == "table_roi":
                    self.table_roi_points = []; self.logger.warning("Table ROI points reset!")
                elif self.mode == "table_cups":
                    self.table_cup_points = []; self.logger.warning("Table cup points reset!")
                elif self.mode == "milk_roi":
                    self.milk_roi_points = []; self.logger.warning("Milk ROI points reset!")
                elif self.mode == "milk_cups":
                    self.milk_cup_points = []; self.logger.warning("Milk cup points reset!")
                elif self.mode == "sauce_roi":
                    self.sauce_roi_points = []; self.logger.warning("Sauce ROI points reset!")
                elif self.mode == "sauce_cups":
                    self.sauce_cup_points = []; self.logger.warning("Sauce cup points reset!")
            elif key == ord('s'):
                self.update_config_file()
            elif key == ord('f'):
                self.logger.warning(f"Frame Info - Resolution: {w}x{h}")
                self.logger.warning(f"Table ROI points: {self.table_roi_points}")
                self.logger.warning(f"Table cup points: {self.table_cup_points}")
                self.logger.warning(f"Milk ROI points: {self.milk_roi_points}")
                self.logger.warning(f"Milk cup points: {self.milk_cup_points}")
                self.logger.warning(f"Sauce ROI points: {self.sauce_roi_points}")
                self.logger.warning(f"Sauce cup points: {self.sauce_cup_points}")
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