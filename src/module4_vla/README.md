# Vision-Language-Action (VLA) Module

This module implements a complete Vision-Language-Action pipeline for robotics applications, enabling robots to understand voice commands, plan actions using LLMs, and execute them in simulation.

## Architecture

The VLA system consists of three main components:

1. **Voice Recognition**: Uses OpenAI Whisper to convert speech to text
2. **Cognitive Planning**: Uses LLMs to convert natural language goals to action plans
3. **Action Execution**: Executes plans using ROS 2 navigation and manipulation

## Components

### Voice Recognition
- `voice_recognizer.py`: Base class for voice recognition
- `voice_to_action_node.py`: ROS node that maps voice commands to ROS actions

### Cognitive Planning
- `cognitive_planner.py`: Base class for LLM-based planning

### VLA Pipeline
- `vlapipeline_node.py`: Main orchestrator that integrates all components

## Dependencies

- Python 3.8+
- ROS 2 Humble Hawksbill
- OpenAI Whisper
- OpenAI GPT API
- OpenCV
- sounddevice

## Installation

```bash
pip install -r requirements.txt
```

## Usage

Run the main VLA pipeline node:
```bash
python -m src.module4_vla.pipeline.vlapipeline_node
```

Or run individual components:
```bash
python -m src.module4_vla.voice.voice_to_action_node
python -m src.module4_vla.planning.cognitive_planner
```

## ROS Topics

- `/vla/voice/text` - Transcribed voice commands
- `/vla/voice/command` - Mapped ROS commands
- `/vla/planning/plan` - Generated action plans
- `/cmd_vel` - Robot velocity commands

## ROS Parameters

- `~whisper_model` - Whisper model size (default: 'base')
- `~openai_api_key` - OpenAI API key
- `~llm_model` - LLM model to use (default: 'gpt-3.5-turbo')