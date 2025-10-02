"""
ROI Selection Tool
Interactive tool to select Region of Interest (ROI) points and update the config file
"""

import cv2
import json
import numpy as np
from typing import List, Tuple, Optional
import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

class ROISelector:
    def __init__(self, config_file: str = "detection_config.json"):
        self.config_file = config_file
        self.points: List[Tuple[int, int]] = []
        self.image = None
        self.window_name = "ROI Selector - Click points, press 'r' to reset, 's' to save, 'q' to quit"
        
    def load_config(self) -> dict:
        """Load configuration from JSON file"""
        try:
            with open(self.config_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"Config file {self.config_file} not found!")
            return {}
        except json.JSONDecodeError as e:
            print(f"Error parsing config file: {e}")
            return {}
    
    def save_config(self, config: dict):
        """Save configuration to JSON file"""
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
            print(f"ROI points saved to {self.config_file}")
        except Exception as e:
            print(f"Error saving config: {e}")
    
    def get_camera_image(self, config: dict) -> Optional[np.ndarray]:
        """Get image from camera using the config URL"""
        try:
            # Setup session with retries
            session = requests.Session()
            retry_strategy = Retry(
                total=config.get('max_retries', 3),
                backoff_factor=config.get('retry_backoff', 0.1),
                status_forcelist=[429, 500, 502, 503, 504]
            )
            adapter = HTTPAdapter(max_retries=retry_strategy)
            session.mount("http://", adapter)
            session.mount("https://", adapter)
            
            # Get image
            response = session.get(
                config['snapshot_url'],
                timeout=(config.get('connect_timeout', 0.5), config.get('read_timeout', 1.0))
            )
            response.raise_for_status()
            
            # Convert to OpenCV format
            image_data = np.frombuffer(response.content, dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            
            if image is not None:
                # Resize to config dimensions if needed
                target_width = config.get('frame_width', 1920)
                target_height = config.get('frame_height', 1080)
                if image.shape[1] != target_width or image.shape[0] != target_height:
                    image = cv2.resize(image, (target_width, target_height))
                
                return image
            else:
                print("Failed to decode image from camera")
                return None
                
        except Exception as e:
            print(f"Error getting camera image: {e}")
            return None
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks to add ROI points"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))
            print(f"Added point {len(self.points)}: ({x}, {y})")
            self.draw_image()
    
    def draw_image(self):
        """Draw the image with current ROI points"""
        if self.image is None:
            return
            
        display_image = self.image.copy()
        
        # Draw all points
        for i, point in enumerate(self.points):
            cv2.circle(display_image, point, 5, (0, 255, 0), -1)
            cv2.putText(display_image, str(i+1), (point[0]+10, point[1]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw polygon if we have at least 3 points
        if len(self.points) >= 3:
            pts = np.array(self.points, np.int32)
            cv2.polylines(display_image, [pts], True, (0, 255, 0), 2)
        
        # Draw instructions
        cv2.putText(display_image, "Click to add points", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_image, "r=reset, s=save, q=quit", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_image, f"Points: {len(self.points)}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow(self.window_name, display_image)
    
    def run(self):
        """Main ROI selection loop"""
        # Load config
        config = self.load_config()
        if not config:
            return
        
        # Get image from camera
        print("Getting image from camera...")
        self.image = self.get_camera_image(config)
        if self.image is None:
            print("Failed to get image from camera. Please check your camera connection.")
            return
        
        # Load existing ROI points if any
        if 'roi_points' in config and config['roi_points']:
            self.points = [tuple(point) for point in config['roi_points']]
            print(f"Loaded {len(self.points)} existing ROI points")
        
        # Create window and set mouse callback
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        # Initial draw
        self.draw_image()
        
        print("\nROI Selection Instructions:")
        print("- Click on the image to add ROI points")
        print("- Press 'r' to reset all points")
        print("- Press 's' to save points to config")
        print("- Press 'q' to quit without saving")
        print("- You need at least 3 points to form a polygon")
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("Quitting without saving...")
                break
            elif key == ord('r'):
                self.points = []
                print("Reset all points")
                self.draw_image()
            elif key == ord('s'):
                if len(self.points) < 3:
                    print("Need at least 3 points to save!")
                    continue
                
                # Convert points to list format for JSON
                roi_points = [[int(x), int(y)] for x, y in self.points]
                config['roi_points'] = roi_points
                
                self.save_config(config)
                print(f"Saved {len(self.points)} ROI points to config")
                break
        
        cv2.destroyAllWindows()

def main():
    """Main function"""
    selector = ROISelector()
    selector.run()

if __name__ == "__main__":
    main()