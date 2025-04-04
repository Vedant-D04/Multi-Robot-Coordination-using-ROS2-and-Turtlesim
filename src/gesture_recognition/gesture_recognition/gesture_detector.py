#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import numpy as np

class GestureDetector(Node):
    def __init__(self):
        super().__init__('gesture_detector')
        self.gesture_publisher = self.create_publisher(String, '/gesture_commands', 10)
        self.timer = self.create_timer(0.1, self.process_frame)
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        self.get_logger().info('Gesture recognition node started')
    
    def process_frame(self):
        success, image = self.cap.read()
        if not success:
            self.get_logger().error('Failed to capture frame from webcam')
            return
        
        # Convert the BGR image to RGB
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Process the image and detect hands
        results = self.hands.process(image_rgb)
        
        # Draw hand landmarks on the image
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Detect gestures based on landmark positions
                gesture = self.detect_gesture(hand_landmarks)
                if gesture:
                    msg = String()
                    msg.data = gesture
                    self.gesture_publisher.publish(msg)
                    self.get_logger().info(f'Detected gesture: {gesture}')
        
        # Display the image with hand landmarks
        cv2.imshow('MediaPipe Hands', image)
        cv2.waitKey(1)
    
    def detect_gesture(self, hand_landmarks):
        # Extract landmark positions
        landmarks = [(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark]
        
        # Check for open palm (all fingers extended) → Cleaning
        if self.is_open_palm(landmarks):
            return "cleaning"
        
        # Check for fist (all fingers closed) → Stop/Reset
        elif self.is_fist(landmarks):
            return "stop_reset"
        
        return None
    
    def is_open_palm(self, landmarks):
        """Check if all fingers are extended (Open Palm)."""
        wrist = landmarks[0]
        finger_tips = [landmarks[8], landmarks[12], landmarks[16], landmarks[20]]
        
        # All fingertips should be above the wrist
        for tip in finger_tips:
            if tip[1] > wrist[1]:  # y-coordinate is smaller when higher in the image
                return False
        return True
    
    def is_fist(self, landmarks):
        """Check if all fingers are closed (Fist)."""
        wrist = landmarks[0]
        palm = landmarks[9]
        finger_tips = [landmarks[8], landmarks[12], landmarks[16], landmarks[20]]
        
        # For a fist, fingertips should be close to the palm
        for tip in finger_tips:
            distance = np.sqrt((tip[0] - palm[0])**2 + (tip[1] - palm[1])**2)
            if distance > 0.15:  # Threshold for considering a finger as extended
                return False
        return True
    
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GestureDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
