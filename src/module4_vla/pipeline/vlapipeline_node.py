#!/usr/bin/env python3
"""
VLA Pipeline Node - Main orchestrator for Vision-Language-Action system
"""

import rospy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
import openai
import whisper
import sounddevice as sd
import numpy as np
import cv2
from cv_bridge import CvBridge


class VLAPipelineNode:
    """
    Main orchestrator for the Vision-Language-Action (VLA) system.
    Integrates voice recognition, cognitive planning, vision processing, and action execution.
    """

    def __init__(self):
        rospy.init_node('vla_pipeline_node')

        # Initialize components
        self.voice_recognizer = VoiceToActionNode()
        self.cognitive_planner = CognitivePlanningNode()
        self.bridge = CvBridge()

        # Publishers and subscribers
        self.voice_sub = rospy.Subscriber('/audio_input', String, self.voice_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Action clients
        self.navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.manipulation_client = actionlib.SimpleActionClient('manipulate_object', ManipulateObjectAction)

        # Internal state
        self.current_plan = None
        self.vision_data = None

        rospy.loginfo("VLA Pipeline Node initialized")

    def voice_callback(self, msg):
        """Process voice commands and trigger planning"""
        rospy.loginfo(f"Received voice command: {msg.data}")

        # Send to cognitive planner
        plan = self.cognitive_planner.process_command(msg.data)

        if plan:
            self.execute_plan(plan)

    def image_callback(self, msg):
        """Process vision data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.vision_data = cv_image
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def execute_plan(self, plan):
        """Execute the generated plan"""
        rospy.loginfo(f"Executing plan: {plan}")

        for action in plan:
            if action['type'] == 'navigation':
                self.execute_navigation(action)
            elif action['type'] == 'manipulation':
                self.execute_manipulation(action)
            elif action['type'] == 'basic_motion':
                self.execute_basic_motion(action)

    def execute_navigation(self, action):
        """Execute navigation action"""
        goal = MoveBaseGoal()
        goal.target_pose = action['target_pose']

        self.navigation_client.send_goal(goal)
        self.navigation_client.wait_for_result()

    def execute_manipulation(self, action):
        """Execute manipulation action"""
        goal = ManipulateObjectGoal()
        goal.object_name = action['object_name']
        goal.action_type = action['action_type']

        self.manipulation_client.send_goal(goal)
        self.manipulation_client.wait_for_result()

    def execute_basic_motion(self, action):
        """Execute basic motion commands (cmd_vel)"""
        twist = Twist()

        if action['command'] == 'move_forward':
            twist.linear.x = 0.5
        elif action['command'] == 'move_backward':
            twist.linear.x = -0.5
        elif action['command'] == 'turn_left':
            twist.angular.z = 0.5
        elif action['command'] == 'turn_right':
            twist.angular.z = -0.5

        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)  # Move for 1 second
        self.cmd_vel_pub.publish(Twist())  # Stop

    def run(self):
        """Main execution loop"""
        rospy.spin()


class VoiceToActionNode:
    """
    Voice recognition component that converts speech to text and maps to ROS actions
    """

    def __init__(self):
        # Initialize Whisper model
        self.model = whisper.load_model("base")
        self.sample_rate = 16000

    def record_audio(self, duration=5):
        """Record audio from microphone"""
        rospy.loginfo(f"Recording audio for {duration} seconds...")
        audio_data = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()  # Wait for recording to complete
        return audio_data.flatten()

    def transcribe_audio(self, audio_data):
        """Transcribe audio to text using Whisper"""
        # Convert audio to the format expected by Whisper
        audio_tensor = np.array(audio_data)

        # Transcribe
        result = self.model.transcribe(audio_tensor)
        return result["text"]

    def process_voice_command(self, audio_data=None):
        """Process voice command from audio data or record new audio"""
        if audio_data is None:
            audio_data = self.record_audio()

        text = self.transcribe_audio(audio_data)
        rospy.loginfo(f"Transcribed: {text}")

        # Map voice command to ROS action
        action = self.map_command_to_action(text)
        return action, text

    def map_command_to_action(self, text):
        """Map transcribed text to ROS action commands"""
        text_lower = text.lower().strip()

        if "move forward" in text_lower or "go forward" in text_lower:
            return {"type": "basic_motion", "command": "move_forward"}
        elif "move backward" in text_lower or "go backward" in text_lower:
            return {"type": "basic_motion", "command": "move_backward"}
        elif "turn left" in text_lower:
            return {"type": "basic_motion", "command": "turn_left"}
        elif "turn right" in text_lower:
            return {"type": "basic_motion", "command": "turn_right"}
        elif "go to" in text_lower or "navigate to" in text_lower:
            # Extract destination from command
            destination = self.extract_destination(text_lower)
            return {"type": "navigation", "destination": destination}
        elif "pick up" in text_lower or "grasp" in text_lower or "take" in text_lower:
            # Extract object from command
            obj = self.extract_object(text_lower)
            return {"type": "manipulation", "action_type": "pick_up", "object_name": obj}
        elif "put down" in text_lower or "place" in text_lower:
            # Extract object from command
            obj = self.extract_object(text_lower)
            return {"type": "manipulation", "action_type": "place", "object_name": obj}
        else:
            return {"type": "unknown", "command": text_lower}

    def extract_destination(self, text):
        """Extract destination from navigation command"""
        # Simple extraction - in practice, this would be more sophisticated
        if "kitchen" in text:
            return "kitchen"
        elif "living room" in text:
            return "living_room"
        elif "bedroom" in text:
            return "bedroom"
        else:
            return "unknown_location"

    def extract_object(self, text):
        """Extract object name from manipulation command"""
        # Simple extraction - in practice, this would use vision feedback
        if "cup" in text:
            return "cup"
        elif "bottle" in text:
            return "bottle"
        elif "book" in text:
            return "book"
        else:
            return "unknown_object"


class CognitivePlanningNode:
    """
    Cognitive planning component that converts natural language goals to symbolic plans
    """

    def __init__(self):
        # Initialize OpenAI client
        self.client = openai.OpenAI()

    def process_command(self, command):
        """Process natural language command and generate execution plan"""
        prompt = f"""
        You are a robot task planner. Convert the following natural language command into a sequence of robot actions.
        Return the plan as a list of action dictionaries with 'type' and relevant parameters.

        Command: {command}

        Available action types:
        - navigation: for moving to locations
        - manipulation: for picking up/placing objects
        - basic_motion: for simple movement commands

        Example format:
        [
            {{"type": "navigation", "target_pose": "..."}},
            {{"type": "manipulation", "object_name": "cup", "action_type": "pick_up"}},
            {{"type": "navigation", "target_pose": "..."}},
            {{"type": "manipulation", "object_name": "cup", "action_type": "place"}}
        ]

        Plan:
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            plan_text = response.choices[0].message.content
            # In a real implementation, we would parse the JSON response
            # For now, we'll return a simplified plan based on the command
            return self.parse_plan_from_command(command)
        except Exception as e:
            rospy.logerr(f"Error in cognitive planning: {e}")
            return self.fallback_plan(command)

    def parse_plan_from_command(self, command):
        """Parse command and return appropriate plan"""
        command_lower = command.lower()

        if "go to kitchen and bring cup" in command_lower:
            return [
                {"type": "navigation", "target_pose": "kitchen_pose"},
                {"type": "manipulation", "object_name": "cup", "action_type": "pick_up"},
                {"type": "navigation", "target_pose": "starting_pose"},
                {"type": "manipulation", "object_name": "cup", "action_type": "place"}
            ]
        elif "move forward" in command_lower:
            return [{"type": "basic_motion", "command": "move_forward"}]
        elif "turn left" in command_lower:
            return [{"type": "basic_motion", "command": "turn_left"}]
        elif "turn right" in command_lower:
            return [{"type": "basic_motion", "command": "turn_right"}]
        else:
            # Fallback: try to extract intent
            return self.fallback_plan(command)

    def fallback_plan(self, command):
        """Generate a fallback plan when LLM processing fails"""
        # Use simple keyword matching as fallback
        voice_node = VoiceToActionNode()
        action = voice_node.map_command_to_action(command)
        return [action]


if __name__ == '__main__':
    try:
        node = VLAPipelineNode()
        node.run()
    except rospy.ROSInterruptException:
        pass