---
title: "Module 4, Chapter 1: Voice-to-Action"
sidebar_label: "Chapter 1: Voice-to-Action"
sidebar_position: 1
description: "Implementing voice command recognition and translation to ROS 2 actions using OpenAI Whisper"
---

# Module 4, Chapter 1: Voice-to-Action

This chapter focuses on implementing voice command recognition and translating spoken language into robotic actions using OpenAI Whisper for speech-to-text and mapping voice commands to ROS 2 actions.

## Learning Objectives

After completing this chapter, you will be able to:
- Set up and configure OpenAI Whisper for speech-to-text processing
- Create ROS 2 nodes that can receive voice commands
- Map voice commands to specific robotic actions
- Implement error handling for voice recognition failures
- Test voice-to-action pipelines in simulation

## Introduction to Voice Command Systems

Voice command systems enable natural human-robot interaction by allowing users to control robots through spoken language. This technology combines speech recognition, natural language processing, and robotic action execution to create intuitive interfaces.

### Key Components
- **Speech Recognition**: Converting spoken words to text
- **Intent Recognition**: Understanding the meaning behind spoken commands
- **Action Mapping**: Translating recognized intents to specific robot behaviors
- **Execution**: Carrying out the requested robotic actions

## Setting Up OpenAI Whisper

OpenAI Whisper is a robust speech recognition model that can be used for voice command processing. We'll set up a ROS 2 node that integrates Whisper for real-time voice command recognition.

### Installation Requirements
- Python 3.8+ with ROS 2 Humble Hawksbill
- OpenAI Whisper library
- Audio input device (microphone)
- Appropriate ROS 2 message types for audio data

```bash
pip install openai-whisper
pip install sounddevice
pip install numpy
```

### Basic Whisper Node Implementation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import whisper
import numpy as np
import sounddevice as sd
from std_msgs.msg import String
from std_msgs.msg import Bool

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')

        # Initialize Whisper model
        self.model = whisper.load_model("base")

        # Audio recording parameters
        self.sample_rate = 16000
        self.chunk_duration = 3  # seconds
        self.recording = False

        # Publishers and subscribers
        self.voice_command_pub = self.create_publisher(String, 'voice_commands', 10)
        self.action_request_pub = self.create_publisher(String, 'action_requests', 10)
        self.voice_active_sub = self.create_subscription(
            Bool, 'voice_active', self.voice_active_callback, 10
        )

        # Timer for continuous recording
        self.timer = self.create_timer(0.1, self.check_recording)

        self.get_logger().info('Voice-to-Action node initialized')

    def voice_active_callback(self, msg):
        self.recording = msg.data

    def check_recording(self):
        if self.recording:
            self.get_logger().info('Recording voice command...')
            self.record_and_process_voice()

    def record_and_process_voice(self):
        try:
            # Record audio for the specified duration
            duration = self.chunk_duration
            audio_data = sd.rec(
                int(duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32'
            )
            sd.wait()  # Wait for recording to complete

            # Convert to the format expected by Whisper
            audio_array = audio_data.flatten()

            # Process with Whisper
            result = self.model.transcribe(audio_array)
            recognized_text = result['text'].strip()

            if recognized_text:
                self.get_logger().info(f'Recognized: {recognized_text}')

                # Publish the recognized command
                cmd_msg = String()
                cmd_msg.data = recognized_text
                self.voice_command_pub.publish(cmd_msg)

                # Process the command and publish action request
                action_msg = String()
                action_msg.data = self.parse_voice_command(recognized_text)
                self.action_request_pub.publish(action_msg)

        except Exception as e:
            self.get_logger().error(f'Error in voice processing: {str(e)}')

    def parse_voice_command(self, text):
        """
        Parse the recognized text and map to specific robot actions
        """
        text_lower = text.lower()

        if 'move forward' in text_lower or 'go forward' in text_lower:
            return 'move_forward'
        elif 'move backward' in text_lower or 'go back' in text_lower:
            return 'move_backward'
        elif 'turn left' in text_lower:
            return 'turn_left'
        elif 'turn right' in text_lower:
            return 'turn_right'
        elif 'stop' in text_lower:
            return 'stop'
        elif 'pick up' in text_lower or 'grasp' in text_lower:
            return 'pick_object'
        elif 'drop' in text_lower or 'release' in text_lower:
            return 'release_object'
        else:
            # Try to identify more complex commands
            if 'go to' in text_lower:
                # Extract location if specified
                return f'go_to_{text_lower.split("go to")[-1].strip()}'
            elif 'bring' in text_lower:
                return f'fetch_{text_lower.split("bring")[-1].strip()}'

        return 'unknown_command'

def main(args=None):
    rclpy.init(args=args)
    node = VoiceToActionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Voice-to-Action node')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Command Processing Pipeline

The voice command processing pipeline consists of several stages:

1. **Audio Capture**: Real-time audio input from microphone
2. **Speech Recognition**: Converting audio to text using Whisper
3. **Command Parsing**: Interpreting the recognized text
4. **Action Mapping**: Converting commands to robot actions
5. **Execution**: Sending commands to appropriate ROS 2 nodes

### Audio Preprocessing

Before processing with Whisper, audio data needs to be properly formatted:

```python
def preprocess_audio(self, audio_data):
    """
    Preprocess audio data for Whisper
    """
    # Normalize audio
    audio_normalized = audio_data / np.max(np.abs(audio_data))

    # Ensure proper format
    if audio_normalized.dtype != np.float32:
        audio_normalized = audio_normalized.astype(np.float32)

    return audio_normalized
```

## Mapping Voice Commands to ROS 2 Actions

The core of the voice-to-action system is the mapping between recognized voice commands and robot actions. This requires:

1. **Command Recognition**: Identifying specific keywords and phrases
2. **Context Understanding**: Understanding the intent behind commands
3. **Action Translation**: Converting to appropriate ROS 2 service calls or topic messages

### Example Action Mapping

```python
class VoiceActionMapper:
    def __init__(self):
        self.command_mappings = {
            'move_forward': self.move_forward_action,
            'move_backward': self.move_backward_action,
            'turn_left': self.turn_left_action,
            'turn_right': self.turn_right_action,
            'stop': self.stop_action,
            'pick_object': self.pick_object_action,
            'release_object': self.release_object_action,
        }

    def execute_action(self, command):
        if command in self.command_mappings:
            return self.command_mappings[command]()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return False

    def move_forward_action(self):
        # Publish Twist message to cmd_vel
        pass

    def turn_left_action(self):
        # Publish angular velocity command
        pass
```

## Testing Voice Commands in Simulation

To test voice commands in simulation:

1. Launch the simulated robot environment
2. Start the voice recognition node
3. Activate voice command mode
4. Speak commands to the microphone
5. Observe robot responses

### Example Test Script

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String

class VoiceTestNode(Node):
    def __init__(self):
        super().__init__('voice_test_node')

        self.voice_active_pub = self.create_publisher(Bool, 'voice_active', 10)
        self.action_sub = self.create_subscription(
            String, 'action_requests', self.action_callback, 10
        )

        self.timer = self.create_timer(5.0, self.activate_voice)

    def activate_voice(self):
        msg = Bool()
        msg.data = True
        self.voice_active_pub.publish(msg)
        self.get_logger().info('Voice recognition activated')

    def action_callback(self, msg):
        self.get_logger().info(f'Received action: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting Voice Recognition

Common issues and solutions:

1. **Poor Audio Quality**: Ensure proper microphone placement and reduce background noise
2. **Recognition Errors**: Train the system with domain-specific vocabulary
3. **Latency Issues**: Optimize processing pipeline and consider smaller Whisper models
4. **Command Misinterpretation**: Improve command parsing and add confirmation steps

## Summary

This chapter covered the implementation of voice command recognition using OpenAI Whisper and mapping voice commands to ROS 2 actions. You learned how to set up the voice recognition pipeline, process audio data, and translate spoken commands into robotic actions.

In the next chapter, we'll explore cognitive planning with LLMs to convert natural language goals into symbolic plans that can be executed by ROS 2 nodes.

## Exercises

1. Implement a voice command to control a simulated robot's movement
2. Add command confirmation to reduce errors from misrecognition
3. Extend the command parser to handle more complex commands with parameters