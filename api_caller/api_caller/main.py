import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray
import os
import openai
from openai import OpenAI
client = openai.Client(
    api_key="YOUR OPENAI API KEY"
)
# TODO: The 'openai.organization' option isn't read in the client API. You will need to pass it when you instantiate the client, e.g. 'OpenAI(organization="org-qJnZJrCe8VAZJW4qI59c8UMd")'
# openai.organization = "org-qJnZJrCe8VAZJW4qI59c8UMd"

# Use a model from OpenAI (assuming "text-embedding-ada-002" exists for this example)
model_name="gpt-3.5-turbo"

class ApiCaller(Node):
    def __init__(self):
        super().__init__('api_caller')
        self.movesPublisher = self.create_publisher(Int8MultiArray, 'moves', 10)
        self.responseSubscriber = self.create_subscription(Float32MultiArray, 'response', self.responseCallback, 10)
        # timer
        self.timer = self.create_timer(21, self.sendMoves)

        self.angle = 90
        self.distance = 0
        self.visible = False
        self.commandHistory = []

        self.numAgents = 1
        self.waitingForResponse = False

    def responseCallback(self, msg):
        self.distance = msg.data[0]
        self.get_logger().info('Distance: %f' % self.distance)
        self.angle = msg.data[1]
        self.get_logger().info('Angle: %f' % self.angle)
        self.waitingForResponse = False

    def sendMoves(self):
        msg = Int8MultiArray()
        if self.waitingForResponse:
            return
        msg.data = self.sampleResponses()
        self.commandHistory = msg.data
        self.movesPublisher.publish(msg)

    def genPrompt(self):
        prompt = f"""You are the best and most precise NASA Robot Controller in the world.
    Your are currently controlling a mobile robot. Your goal is to accomplish the given task. You respond with a bullet point list of moves.
    Your Task is:
    Reach the Red Cube
    ......
    Your observations are:
    {self.getObservations()}
    .....
    Your last commands were:
    {self.getCommandHistory()}
    .....
    The moves you can use are:
    - Move Forward
    - Move Backward
    - Rotate Right
    - Rotate Left
    .....
    Reply with a bullet point list of moves. The format should be: `- <name of the move>` separated by a new line.
    Example if your task is to reach an object 1m away from you that is 20 degree to your right:
    - Rotate Right
    - Rotate Right
    - Move Forward
    """
        return prompt

    def getObservations(self):
        if abs(self.angle) < 70:
            observation = f"- Red Cube is {self.distance}m away from you and {abs(self.angle)} degrees to your {self.angle < 0 and 'right' or 'left'}"
        else:
            observation = "- No object is visible"
        return observation

    def getCommandHistory(self):
        if len(self.commandHistory) == 0:
            return "You have not made any moves yet"
        else:
            commandHistory = ""
        for i in range(len(self.commandHistory)):
            if self.commandHistory[i] == 0:
                commandHistory += "- Move Forward\n"
            elif self.commandHistory[i] == 1:
                commandHistory += "- Move Backward\n"
            elif self.commandHistory[i] == 2:
                commandHistory += "- Rotate Left\n"
            elif self.commandHistory[i] == 3:
                commandHistory += "- Rotate Right\n"
        return commandHistory

    def getMoves(self, message):
        response = client.chat.completions.create(
        model="gpt-3.5-turbo-0125",
        messages=[{"role":"user","content":message}],
        temperature=0.5,
        )
        movesString = response.choices[0].message.content
        movesStrArr = movesString.split("\n")
        moves = []
        for move in movesStrArr:
            if move == "- Move Forward":
                moves.append(0)
            elif move == "- Move Backward":
                moves.append(1)
            elif move == "- Rotate Left":
                moves.append(2)
            elif move == "- Rotate Right":
                moves.append(3)

        return moves

    def sampleResponses(self):
        # implementation of multi-agent setup
        # we will use sampling and voting to select the best response
        # https://arxiv.org/pdf/2402.05120.pdf -- More Agents is All You Need
        message = self.genPrompt()
        self.get_logger().info('Prompt: %s' % message)
        self.waitingForResponse = True
        #create 2d array to store responses
        responses = []
        for i in range(self.numAgents):
            responses.append(self.getMoves(message))

        # make all responses the same length by appending -1
        maxLength = 0
        for response in responses:
            if len(response) > maxLength:
                maxLength = len(response)
        for response in responses:
            while len(response) < maxLength:
                response.append(-1)

        # find the most common response for each index
        bestResponse = []
        for i in range(maxLength):
            counts = [0,0,0,0]
            for response in responses:
                if response[i] != -1:
                    counts[response[i]] += 1
            bestResponse.append(counts.index(max(counts)))

        # remove -1 from the response
        bestResponse = [x for x in bestResponse if x != -1]
        self.get_logger().info('Best Response: %s' % bestResponse)
        return bestResponse


def main(args=None):
    rclpy.init(args=args)
    api_caller = ApiCaller()
    api_caller.sendMoves()
    rclpy.spin(api_caller)
    api_caller.destroy_node()
    rclpy.shutdown()
