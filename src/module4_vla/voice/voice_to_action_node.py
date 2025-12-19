#!/usr/bin/env python3
"""
Whisper-based Voice Recognition Node
Implements the VoiceToActionNode component from the VLA architecture
"""

import rospy
import whisper
import sounddevice as sd
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
from threading import Thread, Event
import queue


class VoiceToActionNode:
    """
    ROS node that converts speech to text using OpenAI Whisper and maps to ROS actions
    """

    def __init__(self):
        rospy.init_node('voice_to_action_node')

        # Initialize Whisper model
        model_size = rospy.get_param('~whisper_model', 'base')
        rospy.loginfo(f"Loading Whisper model: {model_size}")
        self.model = whisper.load_model(model_size)

        # Audio parameters
        self.sample_rate = rospy.get_param('~sample_rate', 16000)
        self.record_duration = rospy.get_param('~record_duration', 3.0)
        self.silence_threshold = rospy.get_param('~silence_threshold', 0.01)

        # Audio recording state
        self.recording = False
        self.recording_thread = None
        self.stop_event = Event()
        self.audio_queue = queue.Queue()

        # Publishers
        self.text_pub = rospy.Publisher('/vla/voice/text', String, queue_size=10)
        self.command_pub = rospy.Publisher('/vla/voice/command', String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        self.voice_command_sub = rospy.Subscriber('/vla/voice/command_raw', String, self.voice_command_callback)

        # Command mapping
        self.command_map = {
            'move forward': self._move_forward,
            'go forward': self._move_forward,
            'move backward': self._move_backward,
            'go backward': self._move_backward,
            'turn left': self._turn_left,
            'turn right': self._turn_right,
            'stop': self._stop,
            'halt': self._stop,
            'spin left': self._spin_left,
            'spin right': self._spin_right,
        }

        # Start audio recording
        self.start_recording()

        rospy.loginfo("VoiceToActionNode initialized and ready")

    def start_recording(self):
        """Start continuous audio recording"""
        self.recording = True
        self.stop_event.clear()
        self.recording_thread = Thread(target=self._recording_loop)
        self.recording_thread.daemon = True
        self.recording_thread.start()
        rospy.loginfo("Started audio recording thread")

    def stop_recording(self):
        """Stop audio recording"""
        self.recording = False
        self.stop_event.set()
        if self.recording_thread:
            self.recording_thread.join(timeout=2.0)
        rospy.loginfo("Stopped audio recording thread")

    def _recording_loop(self):
        """Main recording loop running in separate thread"""
        while not self.stop_event.is_set() and self.recording:
            try:
                # Record audio
                audio_data = self._record_audio_segment()

                # Check if audio has significant content (not just silence)
                if not self._is_silent(audio_data):
                    # Transcribe audio
                    text = self._transcribe_audio(audio_data)

                    if text.strip():
                        rospy.loginfo(f"Transcribed: {text}")

                        # Publish transcribed text
                        text_msg = String()
                        text_msg.data = text
                        self.text_pub.publish(text_msg)

                        # Process the command
                        self._process_transcribed_text(text)

            except Exception as e:
                rospy.logerr(f"Error in recording loop: {e}")

            # Small delay to prevent excessive CPU usage
            rospy.sleep(0.1)

    def _record_audio_segment(self):
        """
        Record a segment of audio

        Returns:
            numpy.ndarray: Audio data
        """
        rospy.logdebug(f"Recording {self.record_duration}s audio segment...")

        # Record audio
        audio_data = sd.rec(
            int(self.record_duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()  # Wait for recording to complete

        # Flatten to 1D array
        audio_data = audio_data.flatten()

        return audio_data

    def _transcribe_audio(self, audio_data):
        """
        Transcribe audio using Whisper

        Args:
            audio_data (numpy.ndarray): Audio data

        Returns:
            str: Transcribed text
        """
        try:
            # Transcribe the audio
            result = self.model.transcribe(audio_data)
            return result["text"].strip()
        except Exception as e:
            rospy.logerr(f"Error transcribing audio: {e}")
            return ""

    def _is_silent(self, audio_data, threshold=None):
        """
        Check if audio data is mostly silence

        Args:
            audio_data (numpy.ndarray): Audio data
            threshold (float): Silence threshold

        Returns:
            bool: True if audio is silent
        """
        if threshold is None:
            threshold = self.silence_threshold

        max_amplitude = np.max(np.abs(audio_data))
        return max_amplitude < threshold

    def _process_transcribed_text(self, text):
        """
        Process transcribed text and map to ROS commands

        Args:
            text (str): Transcribed text
        """
        # Convert to lowercase for matching
        text_lower = text.lower().strip()

        # Try to find a matching command
        for command_phrase, command_func in self.command_map.items():
            if command_phrase in text_lower:
                rospy.loginfo(f"Recognized command: {command_phrase}")

                # Publish the recognized command
                cmd_msg = String()
                cmd_msg.data = command_phrase
                self.command_pub.publish(cmd_msg)

                # Execute the command
                command_func()
                return

        # If no command matched, log it
        rospy.loginfo(f"No command matched for: '{text}'")

    def voice_command_callback(self, msg):
        """
        Callback for voice command messages

        Args:
            msg (String): Voice command message
        """
        rospy.loginfo(f"Received voice command: {msg.data}")
        self._process_transcribed_text(msg.data)

    def _move_forward(self):
        """Execute move forward command"""
        rospy.loginfo("Executing move forward")
        twist = Twist()
        twist.linear.x = 0.5  # Forward at 0.5 m/s
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)  # Move for 1 second
        self._stop()  # Stop after movement

    def _move_backward(self):
        """Execute move backward command"""
        rospy.loginfo("Executing move backward")
        twist = Twist()
        twist.linear.x = -0.5  # Backward at 0.5 m/s
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)  # Move for 1 second
        self._stop()  # Stop after movement

    def _turn_left(self):
        """Execute turn left command"""
        rospy.loginfo("Executing turn left")
        twist = Twist()
        twist.angular.z = 0.5  # Turn left at 0.5 rad/s
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)  # Turn for 1 second
        self._stop()  # Stop after turn

    def _turn_right(self):
        """Execute turn right command"""
        rospy.loginfo("Executing turn right")
        twist = Twist()
        twist.angular.z = -0.5  # Turn right at 0.5 rad/s (negative)
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.0)  # Turn for 1 second
        self._stop()  # Stop after turn

    def _spin_left(self):
        """Execute spin left command (faster turn)"""
        rospy.loginfo("Executing spin left")
        twist = Twist()
        twist.angular.z = 1.0  # Spin left at 1.0 rad/s
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)  # Spin for 0.5 seconds
        self._stop()  # Stop after spin

    def _spin_right(self):
        """Execute spin right command (faster turn)"""
        rospy.loginfo("Executing spin right")
        twist = Twist()
        twist.angular.z = -1.0  # Spin right at 1.0 rad/s (negative)
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)  # Spin for 0.5 seconds
        self._stop()  # Stop after spin

    def _stop(self):
        """Stop all movement"""
        rospy.loginfo("Stopping movement")
        twist = Twist()  # Zero velocities
        self.cmd_vel_pub.publish(twist)

    def run(self):
        """Main execution loop"""
        rospy.loginfo("VoiceToActionNode running...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down VoiceToActionNode...")
            self.stop_recording()


if __name__ == '__main__':
    try:
        node = VoiceToActionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Node interrupted by user")