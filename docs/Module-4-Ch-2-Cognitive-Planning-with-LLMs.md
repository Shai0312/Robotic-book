---
title: "Module 4, Chapter 2: Cognitive Planning with LLMs"
sidebar_label: "Chapter 2: Cognitive Planning with LLMs"
sidebar_position: 2
description: "Using Large Language Models to translate natural language goals into symbolic plans mapped to ROS 2 nodes and actions"
---

# Module 4, Chapter 2: Cognitive Planning with LLMs

This chapter focuses on using Large Language Models (LLMs) to translate natural language goals into symbolic plans that can be executed by ROS 2 nodes and actions. We'll explore how to leverage the cognitive capabilities of LLMs for robotic planning and task execution.

## Learning Objectives

After completing this chapter, you will be able to:
- Integrate LLMs with ROS 2 for cognitive planning
- Convert natural language goals into symbolic action plans
- Map LLM-generated plans to ROS 2 nodes and services
- Implement error handling and plan validation
- Test cognitive planning in simulated environments

## Introduction to LLM-Based Cognitive Planning

Cognitive planning with LLMs represents a significant advancement in robotics, allowing robots to understand and execute complex tasks described in natural language. Unlike traditional planning approaches that require predefined action sequences, LLM-based planning can interpret high-level goals and generate appropriate action plans dynamically.

### Key Concepts
- **Natural Language Understanding**: Interpreting human goals expressed in natural language
- **Symbolic Planning**: Converting high-level goals into executable action sequences
- **Action Mapping**: Connecting abstract plans to concrete ROS 2 operations
- **Plan Execution**: Executing the generated plans through ROS 2 nodes

## Setting Up LLM Integration

For this chapter, we'll use OpenAI's GPT models, but the concepts apply to other LLMs as well. The integration requires:

### Installation Requirements
```bash
pip install openai
pip install langchain
pip install langchain-openai
pip install pydantic
```

### Basic LLM Node Implementation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import openai
import json
from std_msgs.msg import String
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile

class CognitivePlanningNode(Node):
    def __init__(self):
        super().__init__('cognitive_planning_node')

        # Initialize OpenAI API key (use environment variable in production)
        openai.api_key = self.get_parameter_or('openai_api_key', 'your-api-key').value

        # Publishers and subscribers
        self.plan_request_sub = self.create_subscription(
            String, 'natural_language_goals', self.plan_request_callback, 10
        )
        self.plan_output_pub = self.create_publisher(String, 'symbolic_plans', 10)
        self.execution_status_pub = self.create_publisher(String, 'execution_status', 10)

        # Planning parameters
        self.planning_active = False

        # Robot capabilities (define what the robot can do)
        self.robot_capabilities = {
            "navigation": ["move_to_location", "follow_path", "avoid_obstacles"],
            "manipulation": ["pick_object", "place_object", "grasp", "release"],
            "perception": ["detect_objects", "recognize_faces", "measure_distance"],
            "communication": ["speak", "listen", "display_message"]
        }

        self.get_logger().info('Cognitive Planning node initialized')

    def plan_request_callback(self, msg):
        """
        Callback to handle natural language goal requests
        """
        try:
            goal = msg.data
            self.get_logger().info(f'Received goal: {goal}')

            # Generate plan using LLM
            plan = self.generate_plan_with_llm(goal)

            if plan:
                # Publish the generated plan
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.plan_output_pub.publish(plan_msg)

                self.get_logger().info(f'Generated plan: {plan}')

                # Execute the plan
                self.execute_plan(plan)

        except Exception as e:
            self.get_logger().error(f'Error in planning: {str(e)}')
            status_msg = String()
            status_msg.data = f"error: {str(e)}"
            self.execution_status_pub.publish(status_msg)

    def generate_plan_with_llm(self, goal):
        """
        Generate a symbolic plan using LLM based on the natural language goal
        """
        # Define the system prompt for the LLM
        system_prompt = f"""
        You are a robot task planner. Your job is to convert natural language goals into structured action plans.
        The robot has the following capabilities: {self.robot_capabilities}

        Return the plan as a JSON object with the following structure:
        {{
            "goal": "original goal",
            "plan": [
                {{
                    "action": "action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "description": "human-readable description"
                }}
            ]
        }}

        Actions must be from the robot's capabilities. Be specific with parameters.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"Create a plan for: {goal}"}
                ],
                temperature=0.1  # Lower temperature for more consistent planning
            )

            plan_text = response.choices[0].message.content.strip()

            # Extract JSON from the response
            json_start = plan_text.find('{')
            json_end = plan_text.rfind('}') + 1

            if json_start != -1 and json_end != 0:
                plan_json = plan_text[json_start:json_end]
                plan = json.loads(plan_json)
                return plan
            else:
                self.get_logger().error(f'Could not extract JSON from LLM response: {plan_text}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error calling LLM: {str(e)}')
            return None

    def execute_plan(self, plan):
        """
        Execute the plan by publishing to appropriate ROS 2 topics/services
        """
        self.get_logger().info(f'Executing plan: {plan["goal"]}')

        for i, action in enumerate(plan['plan']):
            self.get_logger().info(f'Executing action {i+1}/{len(plan["plan"])}: {action["description"]}')

            # Map action to ROS 2 operation
            self.execute_action(action)

    def execute_action(self, action):
        """
        Execute a single action by publishing to the appropriate ROS 2 interface
        """
        action_name = action['action']
        parameters = action.get('parameters', {})

        # This is a simplified mapping - in practice, you'd have more sophisticated action execution
        if action_name in ['move_to_location', 'navigate_to']:
            self.execute_navigation_action(parameters)
        elif action_name in ['pick_object', 'grasp']:
            self.execute_manipulation_action(parameters)
        elif action_name in ['detect_objects', 'perceive']:
            self.execute_perception_action(parameters)
        elif action_name in ['speak', 'communicate']:
            self.execute_communication_action(parameters)
        else:
            self.get_logger().warn(f'Unknown action: {action_name}')

    def execute_navigation_action(self, params):
        """
        Execute navigation-related actions
        """
        # Publish to navigation topic
        # This would typically involve sending goals to navigation2
        pass

    def execute_manipulation_action(self, params):
        """
        Execute manipulation-related actions
        """
        # Publish to manipulation topic
        # This would typically involve sending goals to MoveIt! or similar
        pass

    def execute_perception_action(self, params):
        """
        Execute perception-related actions
        """
        # Publish to perception topic
        # This would typically involve triggering object detection, etc.
        pass

    def execute_communication_action(self, params):
        """
        Execute communication-related actions
        """
        # Publish to speech synthesis topic
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CognitivePlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Cognitive Planning node')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Planning with LangChain

For more sophisticated planning, we can use LangChain to create structured agents that understand the robot's capabilities:

```python
from langchain_openai import ChatOpenAI
from langchain.agents import initialize_agent, Tool
from langchain.agents import AgentType
import json

class AdvancedCognitivePlanner:
    def __init__(self, robot_capabilities):
        self.llm = ChatOpenAI(temperature=0, model_name="gpt-3.5-turbo")

        # Define tools that represent robot capabilities
        self.tools = [
            Tool(
                name="Navigate",
                func=self.navigate_tool,
                description="Move the robot to a specific location. Input: location name as string."
            ),
            Tool(
                name="Detect Objects",
                func=self.detect_objects_tool,
                description="Detect objects in the environment. Input: description of what to look for."
            ),
            Tool(
                name="Manipulate",
                func=self.manipulate_tool,
                description="Manipulate objects. Input: action and object description."
            ),
            Tool(
                name="Communicate",
                func=self.communicate_tool,
                description="Communicate with humans. Input: message to communicate."
            )
        ]

        # Initialize the agent
        self.agent = initialize_agent(
            self.tools,
            self.llm,
            agent=AgentType.STRUCTURED_CHAT_ZERO_SHOT_REACT_DESCRIPTION,
            verbose=True
        )

    def generate_plan(self, goal):
        """
        Generate a plan using the LangChain agent
        """
        prompt = f"""
        As a robot task planner, create a step-by-step plan to achieve: {goal}
        Use the available tools to break down the task into executable actions.
        Consider the robot's capabilities and the environment.
        """

        result = self.agent.run(prompt)
        return result

    def navigate_tool(self, location):
        """Tool for navigation actions"""
        # This would interface with ROS 2 navigation
        return f"Navigation command sent to reach {location}"

    def detect_objects_tool(self, object_description):
        """Tool for object detection"""
        # This would interface with ROS 2 perception
        return f"Looking for {object_description}"

    def manipulate_tool(self, action_and_object):
        """Tool for manipulation actions"""
        # This would interface with ROS 2 manipulation
        return f"Performing {action_and_object}"

    def communicate_tool(self, message):
        """Tool for communication"""
        # This would interface with ROS 2 speech
        return f"Communicating: {message}"
```

## Plan Validation and Error Handling

LLM-generated plans need validation before execution:

```python
class PlanValidator:
    def __init__(self, robot_capabilities):
        self.capabilities = robot_capabilities

    def validate_plan(self, plan):
        """
        Validate that the plan is executable by the robot
        """
        errors = []

        for action in plan.get('plan', []):
            action_name = action.get('action')

            # Check if the robot has the capability to perform this action
            is_valid = False
            for capability_category, actions in self.capabilities.items():
                if action_name in actions:
                    is_valid = True
                    break

            if not is_valid:
                errors.append(f"Action '{action_name}' not supported by robot")

        return len(errors) == 0, errors

    def validate_parameters(self, action):
        """
        Validate that action parameters are appropriate
        """
        # Implement parameter validation logic
        pass
```

## Integration with ROS 2 Action Servers

For more sophisticated execution, integrate with ROS 2 action servers:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def execute_navigation(self, pose):
        """
        Execute navigation using ROS 2 action server
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)

        return future
```

## Testing Cognitive Planning in Simulation

To test cognitive planning in simulation:

1. Launch the simulated robot environment
2. Start the cognitive planning node
3. Send natural language goals to the planning system
4. Observe the generated plans and execution

### Example Test Script

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PlanningTestNode(Node):
    def __init__(self):
        super().__init__('planning_test_node')
        self.goal_publisher = self.create_publisher(String, 'natural_language_goals', 10)

        # Test after a delay to ensure other nodes are ready
        self.timer = self.create_timer(2.0, self.send_test_goals)

    def send_test_goals(self):
        # Send a few test goals
        goals = [
            "Navigate to the kitchen and bring me a cup",
            "Go to the living room and turn on the light",
            "Find the red ball and bring it to me"
        ]

        for goal in goals:
            msg = String()
            msg.data = goal
            self.goal_publisher.publish(msg)
            self.get_logger().info(f'Sent goal: {goal}')

            # Wait a bit between goals
            self.timer = self.create_timer(5.0, lambda: None)

def main(args=None):
    rclpy.init(args=args)
    node = PlanningTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting Cognitive Planning

Common issues and solutions:

1. **LLM Not Generating Valid Plans**: Ensure proper system prompts and validation
2. **Planning Too General**: Provide more specific context about robot capabilities
3. **Execution Failures**: Implement robust error handling and recovery
4. **Security Concerns**: Validate all LLM outputs before execution

## Summary

This chapter covered the implementation of cognitive planning using LLMs to translate natural language goals into symbolic plans that can be executed by ROS 2 nodes. You learned how to set up LLM integration, validate plans, and execute complex tasks through natural language commands.

In the next chapter, we'll combine everything into a complete Vision-Language-Action pipeline for an autonomous humanoid robot.

## Exercises

1. Implement a cognitive planner that can handle multi-step tasks
2. Add plan validation to ensure safety before execution
3. Create a more sophisticated action mapping system that can handle complex parameters