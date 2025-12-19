#!/usr/bin/env python3
"""
Cognitive Planning Node with LLM Integration
Implements the CognitivePlanningNode component from the VLA architecture
"""

import rospy
import openai
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import re
from typing import List, Dict, Any


class CognitivePlanningNode:
    """
    ROS node that uses LLMs to convert natural language goals into symbolic action plans
    """

    def __init__(self):
        rospy.init_node('cognitive_planning_node')

        # Initialize OpenAI client
        self.api_key = rospy.get_param('~openai_api_key', '')
        if not self.api_key:
            rospy.logwarn("No OpenAI API key provided. Planning will use fallback methods.")

        self.client = openai.OpenAI(api_key=self.api_key) if self.api_key else None
        self.model = rospy.get_param('~llm_model', 'gpt-3.5-turbo')

        # Publishers
        self.plan_pub = rospy.Publisher('/vla/planning/plan', String, queue_size=10)
        self.feedback_pub = rospy.Publisher('/vla/planning/feedback', String, queue_size=10)

        # Subscribers
        self.goal_sub = rospy.Subscriber('/vla/planning/goal', String, self.goal_callback)

        # Robot capabilities (to be configured based on actual robot)
        self.robot_capabilities = {
            'navigation': True,
            'manipulation': True,
            'perception': True,
            'speech': True
        }

        # Predefined locations and objects (in a real system, these would come from world knowledge)
        self.locations = {
            'kitchen': {'x': 1.0, 'y': 2.0, 'theta': 0.0},
            'bedroom': {'x': -1.0, 'y': 1.0, 'theta': 1.57},
            'living_room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'office': {'x': 2.0, 'y': -1.0, 'theta': 3.14}
        }

        self.objects = ['cup', 'bottle', 'book', 'phone', 'keys', 'apple', 'water', 'food']

        rospy.loginfo("CognitivePlanningNode initialized")

    def goal_callback(self, msg):
        """
        Callback for goal messages

        Args:
            msg (String): Natural language goal
        """
        rospy.loginfo(f"Received goal: {msg.data}")

        try:
            # Generate plan
            plan = self.generate_plan(msg.data)

            if plan:
                # Validate plan
                if self.validate_plan(plan):
                    rospy.loginfo(f"Generated valid plan with {len(plan)} actions")

                    # Publish plan
                    plan_msg = String()
                    plan_msg.data = json.dumps(plan)
                    self.plan_pub.publish(plan_msg)

                    feedback_msg = String()
                    feedback_msg.data = f"Plan generated successfully with {len(plan)} actions"
                    self.feedback_pub.publish(feedback_msg)
                else:
                    rospy.logerr("Generated plan is invalid")

                    feedback_msg = String()
                    feedback_msg.data = "Error: Generated plan is invalid"
                    self.feedback_pub.publish(feedback_msg)
            else:
                rospy.logerr("Failed to generate plan")

                feedback_msg = String()
                feedback_msg.data = "Error: Failed to generate plan"
                self.feedback_pub.publish(feedback_msg)

        except Exception as e:
            rospy.logerr(f"Error processing goal: {e}")

            feedback_msg = String()
            feedback_msg.data = f"Error: {str(e)}"
            self.feedback_pub.publish(feedback_msg)

    def generate_plan(self, goal: str) -> List[Dict[str, Any]]:
        """
        Generate a plan for the given goal using LLM or fallback method

        Args:
            goal (str): Natural language goal

        Returns:
            List[Dict]: List of action dictionaries
        """
        if self.client:
            try:
                return self._generate_plan_with_llm(goal)
            except Exception as e:
                rospy.logwarn(f"LLM planning failed: {e}. Using fallback method.")
                return self._generate_plan_fallback(goal)
        else:
            return self._generate_plan_fallback(goal)

    def _generate_plan_with_llm(self, goal: str) -> List[Dict[str, Any]]:
        """
        Generate plan using LLM

        Args:
            goal (str): Natural language goal

        Returns:
            List[Dict]: List of action dictionaries
        """
        prompt = f"""
        You are a robot task planner. Convert the following natural language goal into a sequence of specific robot actions.

        Goal: {goal}

        Available action types:
        - navigation: for moving to locations (parameters: target_pose with x, y, theta)
        - manipulation: for interacting with objects (parameters: object_name, action_type like 'pick_up', 'place', 'grasp')
        - perception: for sensing the environment (parameters: object_type, search_area)
        - speech: for speaking (parameters: text)

        Robot capabilities: {self.robot_capabilities}

        Predefined locations: {list(self.locations.keys())}
        Known objects: {self.objects}

        Return the plan as a JSON array of action objects. Each action should have:
        - type: the action type
        - parameters: relevant parameters for the action

        Example format:
        [
            {{
                "type": "navigation",
                "parameters": {{
                    "target_pose": {{
                        "x": 1.0,
                        "y": 2.0,
                        "theta": 0.0
                    }}
                }}
            }},
            {{
                "type": "perception",
                "parameters": {{
                    "object_type": "cup",
                    "search_area": "kitchen_counter"
                }}
            }},
            {{
                "type": "manipulation",
                "parameters": {{
                    "object_name": "cup",
                    "action_type": "pick_up"
                }}
            }}
        ]

        Plan (JSON only, no additional text):
        """

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1,
            max_tokens=1000
        )

        plan_text = response.choices[0].message.content.strip()

        # Extract JSON from response (in case LLM adds extra text)
        json_match = re.search(r'\[.*\]', plan_text, re.DOTALL)
        if json_match:
            plan_json = json_match.group(0)
            try:
                plan = json.loads(plan_json)
                return plan
            except json.JSONDecodeError as e:
                rospy.logerr(f"Error parsing LLM response: {e}")
                rospy.logdebug(f"LLM response: {plan_text}")
                return self._generate_plan_fallback(goal)
        else:
            rospy.logerr(f"Could not extract JSON from LLM response: {plan_text}")
            return self._generate_plan_fallback(goal)

    def _generate_plan_fallback(self, goal: str) -> List[Dict[str, Any]]:
        """
        Generate plan using fallback rule-based method

        Args:
            goal (str): Natural language goal

        Returns:
            List[Dict]: List of action dictionaries
        """
        goal_lower = goal.lower()
        plan = []

        # Simple rule-based planning based on keywords
        if "go to" in goal_lower or "navigate to" in goal_lower:
            # Extract destination
            destination = self._extract_location(goal_lower)
            if destination and destination in self.locations:
                plan.append({
                    "type": "navigation",
                    "parameters": {
                        "target_pose": self.locations[destination]
                    }
                })

        elif any(obj in goal_lower for obj in self.objects):
            # Handle object manipulation
            obj = self._extract_object(goal_lower)

            # If it's a fetch task (e.g., "bring me a cup")
            if any(word in goal_lower for word in ["bring", "get", "fetch", "take"]):
                # Go to object location
                plan.append({
                    "type": "navigation",
                    "parameters": {
                        "target_pose": {"x": 0.5, "y": 1.5, "theta": 0.0}  # Near object
                    }
                })

                # Perceive the object
                plan.append({
                    "type": "perception",
                    "parameters": {
                        "object_type": obj,
                        "search_area": "current_location"
                    }
                })

                # Manipulate the object (pick up)
                plan.append({
                    "type": "manipulation",
                    "parameters": {
                        "object_name": obj,
                        "action_type": "pick_up"
                    }
                })

                # Return to user
                plan.append({
                    "type": "navigation",
                    "parameters": {
                        "target_pose": {"x": 0.0, "y": 0.0, "theta": 0.0}  # Back to start
                    }
                })

                # Place the object
                plan.append({
                    "type": "manipulation",
                    "parameters": {
                        "object_name": obj,
                        "action_type": "place"
                    }
                })
            else:
                # Simple navigation to where object might be
                plan.append({
                    "type": "navigation",
                    "parameters": {
                        "target_pose": {"x": 1.0, "y": 2.0, "theta": 0.0}  # Generic location
                    }
                })

        elif any(motion in goal_lower for motion in ["move", "go", "turn", "walk"]):
            # Handle simple motion commands
            if "forward" in goal_lower:
                plan.append({
                    "type": "navigation",
                    "parameters": {
                        "target_pose": {"x": 1.0, "y": 0.0, "theta": 0.0}
                    }
                })
            elif "backward" in goal_lower:
                plan.append({
                    "type": "navigation",
                    "parameters": {
                        "target_pose": {"x": -1.0, "y": 0.0, "theta": 0.0}
                    }
                })
            elif "left" in goal_lower:
                plan.append({
                    "type": "navigation",
                    "parameters": {
                        "target_pose": {"x": 0.0, "y": 1.0, "theta": 1.57}
                    }
                })
            elif "right" in goal_lower:
                plan.append({
                    "type": "navigation",
                    "parameters": {
                        "target_pose": {"x": 0.0, "y": -1.0, "theta": -1.57}
                    }
                })

        # Default: no plan
        return plan if plan else [{
            "type": "speech",
            "parameters": {
                "text": f"I don't know how to '{goal}'. Can you rephrase?"
            }
        }]

    def _extract_location(self, goal: str) -> str:
        """
        Extract location from goal text

        Args:
            goal (str): Goal text

        Returns:
            str: Extracted location
        """
        goal_lower = goal.lower()

        # Check for known locations
        for location in self.locations:
            if location in goal_lower:
                return location

        # If no known location, return None
        return None

    def _extract_object(self, goal: str) -> str:
        """
        Extract object from goal text

        Args:
            goal (str): Goal text

        Returns:
            str: Extracted object
        """
        goal_lower = goal.lower()

        # Check for known objects
        for obj in self.objects:
            if obj in goal_lower:
                return obj

        # If no known object, return generic
        return "unknown_object"

    def validate_plan(self, plan: List[Dict[str, Any]]) -> bool:
        """
        Validate the generated plan

        Args:
            plan (List[Dict]): Plan to validate

        Returns:
            bool: True if plan is valid
        """
        if not plan:
            return False

        for action in plan:
            if not isinstance(action, dict):
                rospy.logerr("Action is not a dictionary")
                return False

            if 'type' not in action:
                rospy.logerr("Action missing 'type' field")
                return False

            action_type = action['type']
            if action_type not in ['navigation', 'manipulation', 'perception', 'speech']:
                rospy.logerr(f"Unknown action type: {action_type}")
                return False

            if 'parameters' not in action:
                rospy.logerr("Action missing 'parameters' field")
                return False

            params = action['parameters']
            if action_type == 'navigation':
                if 'target_pose' not in params:
                    rospy.logerr("Navigation action missing 'target_pose' parameter")
                    return False
                pose = params['target_pose']
                if not isinstance(pose, dict) or 'x' not in pose or 'y' not in pose:
                    rospy.logerr("Invalid target_pose format")
                    return False

        return True

    def run(self):
        """Main execution loop"""
        rospy.loginfo("CognitivePlanningNode running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = CognitivePlanningNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Node interrupted by user")