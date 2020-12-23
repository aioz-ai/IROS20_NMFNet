import rospy
import time
import math


class RobotCalculator():
    def __init__(self, distances, range_degree):
        """

        :param distances: A tuple of distances configuration
            d1: Horizontal distance between middle of robot and wheels
            d2: Vertical distance between middle of robot and rear wheels
            d3: Vertical distance between middle of robot and front wheels
            d4: Horizontal distance between middle of robot and middle wheels

            1 --------- 4
            |           |
            2 --------- 3
            |           |
            4 --------- 6
        """

        self.d1, self.d2, self.d3, self.d4 = distances
        self.min_degree, self.max_degree = range_degree
        self.range_radius = (
            self.d3 / math.tan(self.max_degree) + self.d1,
            self.d3 / math.tan(self.min_degree) - self.d1
        )
        print(self.range_radius)

    def calculateVelocity(self, velocity, radius, isRotate):
        """
        Calculate velocity for each wheels
        :param velocity: linear velocity
        :param radius: specific value to compute turing radius
        :param isRotate: check if delivery-robot is rotating
        :return: The dictionary of each wheel's velocity
        """
        if velocity == 0:
            # Not moving
            return {
                wheel: 0 for wheel in ['1', '2', '3', '4', '5', '6']
            }

        if not radius and not isRotate:
            # Move forward
            return {
                '1': velocity,
                '2': velocity,
                '3': velocity,
                '4': -velocity,
                '5': -velocity,
                '6': -velocity,
            }

        isRight = 1 if radius < 0 else -1
        if radius is not None:
            # Calculate turning radius
            radius = self.range_radius[1] - abs(radius)

        if isRotate:
            radius = 0

        R1 = math.sqrt(self.d3**2 + (abs(radius) + isRight*self.d1)**2)
        R4 = math.sqrt(self.d3**2 + (abs(radius) - isRight*self.d1)**2)

        R2 = abs(radius) + isRight*self.d4
        R5 = abs(radius) - isRight*self.d4

        R3 = math.sqrt(self.d2**2 + (abs(radius) + isRight*self.d1)**2)
        R6 = math.sqrt(self.d2**2 + (abs(radius) - isRight*self.d1)**2)

        R = R2 if isRight == 1 else R5

        if not isRotate:
            return{
                '1': velocity * R1 / R,
                '2': velocity * R2 / R,
                '3': velocity * R3 / R,
                '4': -velocity * R4 / R,
                '5': -velocity * R5 / R,
                '6': -velocity * R6 / R
            }
        else:
            return {
                '1': velocity * R1 / R,
                '2': velocity * abs(R2 / R),
                '3': velocity * R3 / R,
                '4': velocity * R4 / R,
                '5': velocity * abs(R5 / R),
                '6': velocity * R6 / R
            }

    def calculateDegree(self, radius, isRotate):
        """
        Calculate steering angle for each steering wheels
        :param radius: specific value for compute turing radius
        :param isRotate: check if delivery-robot is rotating
        :return:
        Steering angle for each wheels
        """
        if isRotate:
            # Compute specific steering angle values when delivery-robot is rotating
            print("---Rotation 360 degree---")
            return {
                '1': -float(math.atan(self.d3 / self.d1)),
                '4': float(math.atan(self.d3 / self.d1)),
                '3': float(math.atan(self.d2 / self.d1)),
                '6': -float(math.atan(self.d2 / self.d1))
            }

        if not radius:
            # Not moving
            print('---Go---')
            return {
                '1': 0,
                '4': 0,
                '3': 0,
                '6': 0
            }

        isRight = 1 if radius < 0 else -1

        if isRight == 1:
            print("Turn Right--->")
        else:
            print("<---Turn Left")

        # Compute turning radius
        radius = self.range_radius[1] - abs(radius)

        if radius < self.range_radius[0]:
            print('---Turing degree is too large---')
            radius = self.range_radius[0]

        return {
            '1': -isRight * math.atan(self.d3 / (abs(radius) + isRight * self.d1)),
            '4': -isRight * math.atan(self.d3 / (abs(radius) - isRight * self.d1)),
            '3': isRight * math.atan(self.d2 / (abs(radius) + isRight * self.d1)),
            '6': isRight * math.atan(self.d2 / (abs(radius) - isRight * self.d1))
        }


