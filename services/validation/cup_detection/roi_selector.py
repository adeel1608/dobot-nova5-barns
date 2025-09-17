"""
ROI and Cup Points Selection Tool
Click to select ROI polygon and cup positions, automatically update config.py
"""

import cv2
import numpy as np
import os
import re
import requests

class ROISelector:
    def __init__(self, snapshot_url: str):
        self.snapshot_url = snapshot_url
        self.roi_points = []
        self.cup_points = []
        self.mode = "roi"  # "roi" or "cups"
        self.drawing = False
        self.frame = None
        self.cap = None
        
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mode == "roi":
                self.roi_points.append((x, y))
                print(f"ROI Point {len(self.roi_points)}: ({x}, {y})")
            elif self.mode == "cups":
                if len(self.cup_points) < 4:
                    self.cup_points.append((x, y))
                    print(f"Cup Position {len(self.cup_points)}: ({x}, {y})")
                else:
                    print("Maximum 4 cup positions allowed!")
            
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            # Show preview line for ROI
            if self.mode == "roi" and len(self.roi_points) > 0:
                temp_frame = self.frame.copy()
                self.draw_all(temp_frame)
                cv2.line(temp_frame, self.roi_points[-1], (x, y), (0, 255, 0), 2)
                cv2.imshow('ROI & Cup Selector', temp_frame)
            
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
    
    def draw_all(self, frame):
        """Draw all points and shapes"""
        # Draw ROI points and polygon
        for i, point in enumerate(self.roi_points):
            cv2.circle(frame, point, 8, (0, 0, 255), -1)
            cv2.putText(frame, f"R{i+1}", (point[0]+10, point[1]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        if len(self.roi_points) >= 3:
            pts = np.array(self.roi_points, np.int32)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        
        # Draw cup points
        for i, point in enumerate(self.cup_points):
            cv2.circle(frame, point, 6, (255, 0, 0), -1)
            cv2.putText(frame, f"C{i+1}", (point[0]+10, point[1]+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
    def update_config_file(self):
        """Update config.py with new ROI and cup points"""
        config_file = "config.py"
        
        if not os.path.exists(config_file):
            print(f"Error: {config_file} not found!")
            return False
        
        # Read current config
        with open(config_file, 'r') as f:
            content = f.read()
        
        # Update ROI_POLYGON
        if len(self.roi_points) >= 3:
            roi_str = "np.array([\n"
            for i, (x, y) in enumerate(self.roi_points):
                roi_str += f"    [{x}, {y}]"
                if i < len(self.roi_points) - 1:
                    roi_str += ","
                roi_str += "\n"
            roi_str += "], dtype=np.int32)"
            
            # Replace ROI_POLYGON
            roi_pattern = r'ROI_POLYGON = np\.array\(\[.*?\], dtype=np\.int32\)'
            roi_replacement = f'ROI_POLYGON = {roi_str}'
            content = re.sub(roi_pattern, roi_replacement, content, flags=re.DOTALL)
        
        # Update CUP_POSITIONS
        if len(self.cup_points) > 0:
            cup_str = "[\n"
            for i, (x, y) in enumerate(self.cup_points):
                cup_str += f"    ({x}, {y})"
                if i < len(self.cup_points) - 1:
                    cup_str += ","
                cup_str += "\n"
            cup_str += "]"
            
            # Replace CUP_POSITIONS
            cup_pattern = r'CUP_POSITIONS = \[.*?\]'
            cup_replacement = f'CUP_POSITIONS = {cup_str}'
            content = re.sub(cup_pattern, cup_replacement, content, flags=re.DOTALL)
        
        # Write updated config
        with open(config_file, 'w') as f:
            f.write(content)
        
        print(f"✅ Updated {config_file}!")
        if len(self.roi_points) >= 3:
            print(f"   - ROI: {len(self.roi_points)} points")
        if len(self.cup_points) > 0:
            print(f"   - Cup positions: {len(self.cup_points)} points")
        return True
    
    def run(self):
        """Run ROI and cup points selection"""
        print("ROI & Cup Points Selection Tool")
        print("=" * 50)
        print("Instructions:")
        print("1. Press '1' to select ROI mode (red points)")
        print("2. Press '2' to select Cup positions mode (blue points)")
        print("3. Click to add points")
        print("4. Press 'r' to reset current mode points")
        print("5. Press 's' to save to config.py")
        print("6. Press 'q' to quit")
        print("7. Press 'f' to show frame info")
        print("=" * 50)
        
        # Test HTTP snapshot
        try:
            response = requests.get(self.snapshot_url, timeout=5)
            response.raise_for_status()
            image_array = np.frombuffer(response.content, dtype=np.uint8)
            test_frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            
            if test_frame is not None:
                h, w = test_frame.shape[:2]
                print(f"Camera resolution: {w}x{h}")
                if w == 1920 and h == 1080:
                    print("✅ 1920x1080 resolution detected")
                else:
                    print(f"⚠️  Non-standard resolution: {w}x{h}")
            else:
                print("⚠️  Could not decode image from HTTP response")
                return
        except Exception as e:
            print(f"Error: Could not get snapshot from {self.snapshot_url}")
            print(f"Error details: {e}")
            return
        
        # Create window and set mouse callback
        cv2.namedWindow('ROI & Cup Selector', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('ROI & Cup Selector', self.mouse_callback)
        
        while True:
            try:
                # Get fresh frame from HTTP snapshot
                response = requests.get(self.snapshot_url, timeout=2)
                response.raise_for_status()
                image_array = np.frombuffer(response.content, dtype=np.uint8)
                self.frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                
                if self.frame is None:
                    print("Error: Could not decode frame")
                    break
            except Exception as e:
                print(f"Error getting frame: {e}")
                break
            
            # Draw current points and shapes
            display_frame = self.frame.copy()
            self.draw_all(display_frame)
            
            # Show instructions on frame
            mode_text = "ROI Mode" if self.mode == "roi" else "Cup Mode"
            h, w = display_frame.shape[:2]
            cv2.putText(display_frame, f"Mode: {mode_text}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display_frame, "1=ROI, 2=Cups, s=Save, f=Info, q=Quit", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_frame, f"ROI: {len(self.roi_points)} | Cups: {len(self.cup_points)}", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_frame, f"Resolution: {w}x{h}", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow('ROI & Cup Selector', display_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('1'):
                self.mode = "roi"
                print("Switched to ROI mode (red points)")
            elif key == ord('2'):
                self.mode = "cups"
                print("Switched to Cup positions mode (blue points)")
            elif key == ord('r'):
                if self.mode == "roi":
                    self.roi_points = []
                    print("ROI points reset!")
                else:
                    self.cup_points = []
                    print("Cup points reset!")
            elif key == ord('s'):
                if self.update_config_file():
                    print("Configuration saved to config.py!")
                else:
                    print("Failed to save configuration!")
            elif key == ord('f'):
                h, w = self.frame.shape[:2]
                print(f"\nFrame Information:")
                print(f"  Resolution: {w}x{h}")
                print(f"  ROI points: {len(self.roi_points)}")
                print(f"  Cup points: {len(self.cup_points)}")
                if self.roi_points:
                    print(f"  ROI coordinates: {self.roi_points}")
                if self.cup_points:
                    print(f"  Cup coordinates: {self.cup_points}")
                print()
        
        # Cleanup
        cv2.destroyAllWindows()
        print("Selection completed!")

def main():
    """Main function"""
    # Get snapshot URL from config or use default
    snapshot_url = "http://qltyss:QSS2030QSS@192.168.200.60/cgi-bin/snapshot.cgi"
    
    # Try to read from config.py
    try:
        with open("config.py", 'r') as f:
            content = f.read()
            match = re.search(r'SNAPSHOT_URL = "([^"]+)"', content)
            if match:
                snapshot_url = match.group(1)
                print(f"Using snapshot URL from config.py: {snapshot_url}")
    except:
        print(f"Using default snapshot URL: {snapshot_url}")
    
    # Create and run ROI selector
    selector = ROISelector(snapshot_url)
    selector.run()

if __name__ == "__main__":
    main()
