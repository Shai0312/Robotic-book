#!/usr/bin/env python3
"""
Voice Recognition Base Class
"""

import rospy
import whisper
import sounddevice as sd
import numpy as np
from std_msgs.msg import String
from threading import Thread
import queue


class VoiceRecognizer:
    """
    Base class for voice recognition functionality.
    Uses OpenAI Whisper for speech-to-text conversion.
    """

    def __init__(self, model_size="base"):
        """
        Initialize the voice recognizer

        Args:
            model_size (str): Size of the Whisper model ('tiny', 'base', 'small', 'medium', 'large')
        """
        rospy.loginfo(f"Loading Whisper model: {model_size}")
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000
        self.recording = False
        self.audio_queue = queue.Queue()

        # Publishers
        self.text_pub = rospy.Publisher('/vla/voice/text', String, queue_size=10)

        # Parameters
        self.record_duration = rospy.get_param('~record_duration', 5.0)
        self.silence_threshold = rospy.get_param('~silence_threshold', 0.01)

    def start_listening(self):
        """Start continuous listening for voice commands"""
        rospy.loginfo("Starting voice recognition...")
        self.recording = True

        # Start audio recording thread
        self.recording_thread = Thread(target=self._recording_loop)
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def stop_listening(self):
        """Stop voice recognition"""
        rospy.loginfo("Stopping voice recognition...")
        self.recording = False
        if hasattr(self, 'recording_thread'):
            self.recording_thread.join(timeout=2.0)

    def _recording_loop(self):
        """Main recording loop"""
        while self.recording:
            try:
                # Record audio
                audio_data = self.record_audio(self.record_duration)

                # Transcribe
                text = self.transcribe_audio(audio_data)

                if text.strip():  # Only publish if we got meaningful text
                    rospy.loginfo(f"Transcribed: {text}")

                    # Publish transcribed text
                    text_msg = String()
                    text_msg.data = text
                    self.text_pub.publish(text_msg)

                    # Trigger any callbacks
                    self.on_voice_command(text)

            except Exception as e:
                rospy.logerr(f"Error in recording loop: {e}")
                rospy.sleep(1.0)

    def record_audio(self, duration):
        """
        Record audio from microphone

        Args:
            duration (float): Duration to record in seconds

        Returns:
            numpy.ndarray: Audio data
        """
        rospy.loginfo(f"Recording audio for {duration} seconds...")

        # Record audio
        audio_data = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()  # Wait for recording to complete

        # Flatten and normalize
        audio_data = audio_data.flatten()

        return audio_data

    def transcribe_audio(self, audio_data):
        """
        Transcribe audio to text using Whisper

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

    def on_voice_command(self, text):
        """
        Callback method called when a voice command is recognized.
        Override this method in subclasses to handle commands.

        Args:
            text (str): Recognized text
        """
        pass

    def is_silent(self, audio_data, threshold=None):
        """
        Check if audio data is silent

        Args:
            audio_data (numpy.ndarray): Audio data
            threshold (float): Silence threshold (uses default if None)

        Returns:
            bool: True if audio is silent
        """
        if threshold is None:
            threshold = self.silence_threshold

        max_amplitude = np.max(np.abs(audio_data))
        return max_amplitude < threshold


class VoiceCommandProcessor(VoiceRecognizer):
    """
    Enhanced voice recognizer that processes commands and maps them to actions
    """

    def __init__(self, model_size="base"):
        super().__init__(model_size)

        # Publishers for specific command types
        self.command_pub = rospy.Publisher('/vla/voice/command', String, queue_size=10)

        # Command mapping
        self.command_mappings = {
            'move forward': 'move_forward',
            'go forward': 'move_forward',
            'move backward': 'move_backward',
            'go backward': 'move_backward',
            'turn left': 'turn_left',
            'turn right': 'turn_right',
            'stop': 'stop',
            'halt': 'stop',
        }

    def on_voice_command(self, text):
        """
        Process recognized voice command and publish mapped action

        Args:
            text (str): Recognized text
        """
        # Map to command
        command = self.map_command(text)

        if command:
            rospy.loginfo(f"Mapped command: {command}")

            # Publish command
            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)

    def map_command(self, text):
        """
        Map recognized text to specific command

        Args:
            text (str): Recognized text

        Returns:
            str: Mapped command or None if no match
        """
        text_lower = text.lower().strip()

        # Exact match
        if text_lower in self.command_mappings:
            return self.command_mappings[text_lower]

        # Partial match with keywords
        for key, value in self.command_mappings.items():
            if key in text_lower:
                return value

        # If no match, return None
        return None

    def add_command_mapping(self, text_pattern, command):
        """
        Add a new command mapping

        Args:
            text_pattern (str): Text pattern to match
            command (str): Command to execute
        """
        self.command_mappings[text_pattern.lower()] = command


if __name__ == '__main__':
    # Example usage
    rospy.init_node('voice_recognizer_node')

    recognizer = VoiceCommandProcessor(model_size="base")
    recognizer.start_listening()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        recognizer.stop_listening()