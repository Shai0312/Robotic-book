#!/usr/bin/env python3
"""
Test script for VLA components
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

def test_voice_recognizer():
    """Test voice recognition functionality"""
    print("Testing Voice Recognizer...")

    try:
        from voice.voice_recognizer import VoiceRecognizer

        # Create a basic instance
        recognizer = VoiceRecognizer(model_size="tiny")  # Use tiny model for testing
        print("✓ Voice Recognizer created successfully")

        # Test basic functionality
        print(f"✓ Model loaded: {type(recognizer.model).__name__}")
        print(f"✓ Sample rate: {recognizer.sample_rate} Hz")

        return True
    except ImportError as e:
        print(f"✗ Voice Recognizer test failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Voice Recognizer test failed: {e}")
        return False

def test_cognitive_planner():
    """Test cognitive planning functionality"""
    print("\nTesting Cognitive Planner...")

    try:
        from planning.cognitive_planner import CognitivePlanner

        # Create a basic instance
        planner = CognitivePlanner()
        print("✓ Cognitive Planner created successfully")

        # Test basic functionality
        print(f"✓ Robot capabilities: {planner.robot_capabilities}")

        # Test fallback planning
        goal = "go to kitchen"
        plan = planner._generate_plan_fallback(goal)
        print(f"✓ Fallback plan for '{goal}': {len(plan)} actions")

        return True
    except ImportError as e:
        print(f"✗ Cognitive Planner test failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Cognitive Planner test failed: {e}")
        return False

def test_vla_pipeline():
    """Test VLA pipeline functionality"""
    print("\nTesting VLA Pipeline...")

    try:
        from pipeline.vlapipeline_node import VLAPipelineNode, VoiceToActionNode, CognitivePlanningNode

        # Test component creation
        voice_node = VoiceToActionNode()
        print("✓ VoiceToActionNode created successfully")

        cognitive_node = CognitivePlanningNode()
        print("✓ CognitivePlanningNode created successfully")

        # Note: We don't actually initialize the full pipeline node here
        # as it would require ROS to be running
        print("✓ VLA Pipeline components created successfully")

        return True
    except ImportError as e:
        print(f"✗ VLA Pipeline test failed: {e}")
        return False
    except Exception as e:
        print(f"✗ VLA Pipeline test failed: {e}")
        return False

def main():
    """Main test function"""
    print("VLA Module Test Suite")
    print("=" * 30)

    tests = [
        test_voice_recognizer,
        test_cognitive_planner,
        test_vla_pipeline
    ]

    results = []
    for test in tests:
        results.append(test())

    print("\n" + "=" * 30)
    print("Test Results:")
    print(f"Passed: {sum(results)}/{len(results)}")

    if all(results):
        print("✓ All tests passed!")
        return 0
    else:
        print("✗ Some tests failed!")
        return 1

if __name__ == '__main__':
    sys.exit(main())