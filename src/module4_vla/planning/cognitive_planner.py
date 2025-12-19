#!/usr/bin/env python3
"""
Cognitive Planning Base Class
"""

import rospy
import openai
from std_msgs.msg import String
import json
import re
from typing import List, Dict, Any


class CognitivePlanner:
    """
    Base class for cognitive planning functionality.
    Uses LLMs to convert natural language goals into symbolic action plans.
    """

    def __init__(self):
        """
        Initialize the cognitive planner
        """
        # Initialize OpenAI client
        api_key = rospy.get_param('~openai_api_key', '')
        if not api_key:
            rospy.logwarn("No OpenAI API key provided. Planning will use fallback methods.")

        self.client = openai.OpenAI(api_key=api_key) if api_key else None
        self.model = rospy.get_param('~llm_model', 'gpt-3.5-turbo')

        # Publishers
        self.plan_pub = rospy.Publisher('/vla/planning/plan', String, queue_size=10)

        # Robot capabilities (to be defined based on the robot platform)
        self.robot_capabilities = {
            'navigation': True,
            'manipulation': True,
            'perception': True,
            'speech': True
        }

        rospy.loginfo("Cognitive Planner initialized")

    def generate_plan(self, goal: str) -> List[Dict[str, Any]]:
        """
        Generate a plan for the given goal

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
        - navigation: for moving to locations (parameters: target_pose)
        - manipulation: for interacting with objects (parameters: object_name, action_type)
        - perception: for sensing the environment (parameters: sensor_type, target_object)
        - speech: for speaking (parameters: text)

        Robot capabilities: {self.robot_capabilities}

        Return the plan as a JSON array of action objects. Each action should have:
        - type: the action type
        - parameters: relevant parameters for the action

        Example format:
        [
            {{
                "type": "perception",
                "parameters": {{
                    "sensor_type": "camera",
                    "target_object": "cup"
                }}
            }},
            {{
                "type": "navigation",
                "parameters": {{
                    "target_pose": "kitchen_area"
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

        # Simple rule-based planning
        if "go to" in goal_lower or "navigate to" in goal_lower:
            # Extract destination
            destination = self._extract_destination(goal_lower)
            return [{
                "type": "navigation",
                "parameters": {"target_pose": destination}
            }]
        elif "pick up" in goal_lower or "get" in goal_lower and "bring" in goal_lower:
            # Extract object and destination
            obj = self._extract_object(goal_lower)
            destination = self._extract_destination(goal_lower) if "bring" in goal_lower else None

            plan = []

            # Go to object location
            plan.append({
                "type": "navigation",
                "parameters": {"target_pose": f"{obj}_location"}
            })

            # Pick up object
            plan.append({
                "type": "manipulation",
                "parameters": {"object_name": obj, "action_type": "pick_up"}
            })

            # Go to destination if specified
            if destination:
                plan.append({
                    "type": "navigation",
                    "parameters": {"target_pose": destination}
                })

                # Place object
                plan.append({
                    "type": "manipulation",
                    "parameters": {"object_name": obj, "action_type": "place"}
                })

            return plan
        elif "move" in goal_lower or "go" in goal_lower:
            # Simple movement commands
            if "forward" in goal_lower:
                return [{
                    "type": "navigation",
                    "parameters": {"target_pose": "forward_position"}
                }]
            elif "backward" in goal_lower:
                return [{
                    "type": "navigation",
                    "parameters": {"target_pose": "backward_position"}
                }]
            elif "left" in goal_lower:
                return [{
                    "type": "navigation",
                    "parameters": {"target_pose": "left_position"}
                }]
            elif "right" in goal_lower:
                return [{
                    "type": "navigation",
                    "parameters": {"target_pose": "right_position"}
                }]

        # Default: no plan
        return []

    def _extract_destination(self, goal: str) -> str:
        """
        Extract destination from goal text

        Args:
            goal (str): Goal text

        Returns:
            str: Extracted destination
        """
        # Look for common destination words
        destinations = ['kitchen', 'bedroom', 'living room', 'office', 'dining room', 'bathroom', 'garage']

        for dest in destinations:
            if dest in goal:
                return dest.replace(' ', '_')

        # If no known destination, return a generic one
        return "unknown_location"

    def _extract_object(self, goal: str) -> str:
        """
        Extract object from goal text

        Args:
            goal (str): Goal text

        Returns:
            str: Extracted object
        """
        # Look for common object words
        objects = ['cup', 'bottle', 'book', 'phone', 'keys', 'apple', 'water', 'food', 'box', 'toy']

        for obj in objects:
            if obj in goal:
                return obj

        # If no known object, return a generic one
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

        return True

    def execute_plan(self, plan: List[Dict[str, Any]]):
        """
        Execute the plan (placeholder - actual execution would be in action execution module)

        Args:
            plan (List[Dict]): Plan to execute
        """
        rospy.loginfo(f"Executing plan with {len(plan)} actions")

        for i, action in enumerate(plan):
            rospy.loginfo(f"Executing action {i+1}/{len(plan)}: {action['type']}")
            # In a real implementation, this would send actions to the appropriate execution nodes
            rospy.sleep(0.5)  # Simulate execution time

    def process_goal(self, goal_msg: String):
        """
        ROS callback to process goal messages

        Args:
            goal_msg (String): ROS message containing the goal
        """
        rospy.loginfo(f"Processing goal: {goal_msg.data}")

        # Generate plan
        plan = self.generate_plan(goal_msg.data)

        # Validate plan
        if self.validate_plan(plan):
            rospy.loginfo(f"Generated valid plan with {len(plan)} actions")

            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            # Optionally execute immediately
            self.execute_plan(plan)
        else:
            rospy.logerr("Generated plan is invalid")


class EnhancedCognitivePlanner(CognitivePlanner):
    """
    Enhanced cognitive planner with additional features like plan validation and optimization
    """

    def __init__(self):
        super().__init__()

        # Additional publishers
        self.optimized_plan_pub = rospy.Publisher('/vla/planning/optimized_plan', String, queue_size=10)

    def optimize_plan(self, plan: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Optimize the plan by removing redundant actions or combining similar ones

        Args:
            plan (List[Dict]): Original plan

        Returns:
            List[Dict]: Optimized plan
        """
        if not plan:
            return plan

        optimized_plan = []
        i = 0

        while i < len(plan):
            current_action = plan[i]

            # Look for opportunities to combine similar actions
            if (i < len(plan) - 1 and
                current_action['type'] == plan[i + 1]['type'] and
                current_action['type'] == 'navigation'):
                # Combine consecutive navigation actions if they're part of the same path
                combined_action = {
                    "type": "navigation",
                    "parameters": {
                        "target_pose": plan[i + 1]['parameters']['target_pose']
                    }
                }
                optimized_plan.append(combined_action)
                i += 2  # Skip next action since it's combined
            else:
                optimized_plan.append(current_action)
                i += 1

        return optimized_plan

    def validate_and_optimize(self, plan: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Validate and optimize a plan

        Args:
            plan (List[Dict]): Plan to validate and optimize

        Returns:
            List[Dict]: Validated and optimized plan, or empty list if invalid
        """
        if not self.validate_plan(plan):
            rospy.logerr("Plan validation failed")
            return []

        optimized_plan = self.optimize_plan(plan)

        # Publish optimized plan
        if optimized_plan:
            optimized_plan_msg = String()
            optimized_plan_msg.data = json.dumps(optimized_plan)
            self.optimized_plan_pub.publish(optimized_plan_msg)

        return optimized_plan


if __name__ == '__main__':
    # Example usage
    rospy.init_node('cognitive_planner_node')

    planner = EnhancedCognitivePlanner()

    # Example goals
    example_goals = [
        "Go to kitchen and bring me a cup",
        "Move forward and turn left",
        "Navigate to the bedroom"
    ]

    for goal in example_goals:
        print(f"\nProcessing goal: {goal}")
        plan = planner.generate_plan(goal)
        print(f"Generated plan: {plan}")

        if planner.validate_plan(plan):
            print("Plan is valid")
            optimized_plan = planner.optimize_plan(plan)
            print(f"Optimized plan: {optimized_plan}")
        else:
            print("Plan is invalid")

    rospy.spin()