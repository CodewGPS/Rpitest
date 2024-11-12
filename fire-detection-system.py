import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import serial

class FireDetectionSystem:
    def __init__(self):
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Pin Definitions
        self.BUZ_PIN = 4
        self.PMP1_PIN = 2
        self.PMP2_PIN = 3
        
        # Initialize GPIO pins
        self.setup_gpio()
        
        # Initialize camera
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            raise RuntimeError("Failed to open camera")
            
        # Fire detection parameters
        self.FIRE_THRESHOLD = 20000  # Adjust based on testing
        self.lower_fire = np.array([18, 50, 50], dtype="uint8")
        self.upper_fire = np.array([35, 255, 255], dtype="uint8")

    def setup_gpio(self):
        """Initialize GPIO pins"""
        for pin in [self.BUZ_PIN, self.PMP1_PIN, self.PMP2_PIN]:
            GPIO.setup(pin, GPIO.OUT)
        
        # Set initial states
        GPIO.output(self.BUZ_PIN, GPIO.HIGH)  # Buzzer off
        GPIO.output(self.PMP1_PIN, GPIO.LOW)  # Pump 1 off
        GPIO.output(self.PMP2_PIN, GPIO.LOW)  # Pump 2 off

    def process_frame(self, frame):
        """Process a single frame for fire detection"""
        # Apply Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(frame, (21, 21), 0)
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        
        # Create mask for fire-like colors
        mask = cv2.inRange(hsv, self.lower_fire, self.upper_fire)
        
        # Calculate number of fire-like pixels
        fire_pixels = cv2.countNonZero(mask)
        
        return mask, fire_pixels

    def handle_fire_detected(self, frame):
        """Handle fire detection event"""
        print('Fire detected!')
        GPIO.output(self.BUZ_PIN, GPIO.LOW)  # Turn on buzzer
        GPIO.output(self.PMP1_PIN, GPIO.HIGH)  # Turn on pump 1
        
        # Save the frame showing fire
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        cv2.imwrite(f'fire_detected_{timestamp}.jpg', frame)
        
        time.sleep(3)  # Run pump for 3 seconds
        GPIO.output(self.BUZ_PIN, GPIO.HIGH)  # Turn off buzzer

    def run(self):
        """Main loop for fire detection"""
        try:
            while True:
                ret, frame = self.camera.read()
                if not ret:
                    print("Failed to grab frame")
                    break

                mask, fire_pixels = self.process_frame(frame)
                
                # Show processed images
                cv2.imshow("Original", frame)
                cv2.imshow("Fire Detection", cv2.bitwise_and(frame, frame, mask=mask))
                
                # Check for fire condition
                if fire_pixels > self.FIRE_THRESHOLD:
                    self.handle_fire_detected(frame)
                else:
                    GPIO.output(self.PMP1_PIN, GPIO.LOW)
                    GPIO.output(self.PMP2_PIN, GPIO.LOW)

                # Break loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            print("Stopping fire detection system...")
        
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources"""
        self.camera.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()

if __name__ == "__main__":
    try:
        fire_system = FireDetectionSystem()
        fire_system.run()
    except Exception as e:
        print(f"Error: {e}")
        GPIO.cleanup()
