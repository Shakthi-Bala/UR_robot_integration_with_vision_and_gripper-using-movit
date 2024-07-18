#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PointStamped
from my_moveit_planner.msg import FinalAction, FinalGoal
from deprojection_pipeline_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import cv_bridge
import cv2
from PIL import Image
import io
import base64
import json
import requests

class CentralClient:
    def __init__(self):
        self.get_object_locations_service = rospy.ServiceProxy("get_object_locations", GetObjectLocations)
        self.action_client = actionlib.SimpleActionClient('task_server', FinalAction)

    def get_object_locations(self):
        try:
            response = self.get_object_locations_service()
            return response
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def convert_image_to_text(self, image: Image) -> str:
        with io.BytesIO() as output:
            image.save(output, format="PNG")
            data = output.getvalue()
        return base64.b64encode(data).decode("utf-8")
    
    def test_demo_api(self, instruction: str, response: GetObjectLocationsResponse, url: str = "https://handsomely-lying-octagon.glitch.me/generate_plan"):
        image_ = cv_bridge.CvBridge().imgmsg_to_cv2(response.result.image, desired_encoding="bgr8")
        image_ = Image.fromarray(cv2.cvtColor(image_, cv2.COLOR_BGR2RGB))

        # create a timestamp using ros
        timestamp = rospy.Time.now()
        image_.save(str(timestamp)+".png")

        objects = []
        for object_position in response.result.object_position:
            objects.append({
                "object_id": object_position.id,
                "x_min": object_position.x_min,
                "y_min": object_position.y_min,
                "x_max": object_position.x_max,
                "y_max": object_position.y_max,
            })
        
        image = {
            "base64_string": self.convert_image_to_text(image_),
            "objects": objects
        }

        data = {
            "instruction": instruction,
            "images": [image],
        }

        headers = {"Content-Type": "application/json"}
        response = requests.post(url, data=json.dumps(data), headers=headers)
        return response.json()
    
    def send_goal(self, start_pose, end_pose):
        self.action_client.wait_for_server()
        goal = FinalGoal()
        goal.start_pose = start_pose
        goal.end_pose = end_pose

        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        return self.action_client.get_result()

    # execute all the actions in the action list one by one here.
    def execute_actions(self, action_list):
        print("Executing actions ...")
        for action in action_list:
            print("Executing action : ", action["action_type"])
            print("Source object position : ")
            print(action["source_object_position"])
            print("Target object position : ") 
            print(action["target_object_position"])

            start_pose = PointStamped()
            start_pose.header.frame_id = "base_link"
            start_pose.header.stamp = rospy.Time.now()
            start_pose.point.x = action["source_object_position"].point.x + 0.02
            start_pose.point.y = action["source_object_position"].point.y
            start_pose.point.z = action["source_object_position"].point.z + 0.120

            end_pose = PointStamped()
            end_pose.header.frame_id = "base_link"
            end_pose.header.stamp = rospy.Time.now()
            end_pose.point.x = action["target_object_position"].point.x  + 0.02
            end_pose.point.y = action["target_object_position"].point.y
            end_pose.point.z = action["target_object_position"].point.z  + 0.120

            result = self.send_goal(start_pose, end_pose)
            if not result.success:
                print("Action failed")
                break

if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.1)
    
    # Call the service, response contains object_positions, bounding_boxes, and image
    response = central_client.get_object_locations()
    if not response:
        rospy.logerr("Failed to get object locations")
        exit(1)

    rospy.sleep(0.5)
    instruction = "pick up all the oranges and place it in a bowl"
    print("Requesting for plan ...")
    print("Prompt : ", instruction)

    plan = central_client.test_demo_api(instruction, response)
    for action in plan["plan_actions"]:
        action["source_object_id"] = int(action["source_object_id"])
        action["target_object_id"] = int(action["target_object_id"])

    action_list = []
    for action in plan["plan_actions"]:
        action_parsed = {
            "action_type": action["action_type"],
            "source_object_position": response.result.object_position[action["source_object_id"]].position,
            "target_object_position": response.result.object_position[action["target_object_id"]].position
        }
        action_list.append(action_parsed)

    central_client.execute_actions(action_list)
