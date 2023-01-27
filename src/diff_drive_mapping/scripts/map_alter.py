#!/usr/bin/python3
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped


class mapAlter:
    def __init__(self):
        rospy.init_node('map_alter',anonymous=True)
        rospy.Subscriber('/map',OccupancyGrid,self.mapcallback)
        rospy.Subscriber('clicked_point', PointStamped,self.clickpoint)
        self.map_pub = rospy.Publisher('/map_altered',OccupancyGrid,queue_size=10)
        self.PX = 0.0
        self.PY = 0.0
        self.PxX = 0.0
        self.PxY = 0.0
        self.res = 0.05
        self.points = []
        self.mappoint = 0
        self.map_D = OccupancyGrid()

    def mapcallback(self, map):
        self.map_D.header = map.header
        self.map_D.info = map.info
        self.map_D.data = map.data


    def clickpoint(self, point):
        self.PX = point.point.x
        self.PY = point.point.y
        self.PxX = self.PX / self.res
        self.PxY = self.PY / self.res
        self.points.append([int(self.PxX), int(self.PxY)])
        if len(self.points)%2 == 0:
            req = input("Update Or Remove\nType U or R\n")
            if req == "U" or req == "u":
                self.mappoint += 2
                self.draw(self.mappoint)
            elif req == "R" or req == "r":
                print("Whcich one to remove\n")
                print(self.points)
                wh = int(input("\nWhich one to remove\n Give index\n"))
                self.remove(wh)
                self.mappoint -= 2
                self.points.pop(wh)
                self.points.pop(wh+1)
                self.points.pop(len(self.points)-1)
                self.points.pop(len(self.points)-2)
                print("Removed :-)\n")
                rospy.loginfo(self.points)

            else:
                print("sorry could not be processed!")  
        
    def mappub(self):
        while not rospy.is_shutdown():
            self.map_pub.publish(self.map_D)

    def draw(self, mappoint):
        width = abs(self.points[mappoint-1][1] - self.points[mappoint-2][1])
        height = abs(self.points[mappoint-2][0] - self.points[mappoint-1][0])
        
        for i in range(height):
            start = ((400-i+self.points[mappoint-2][1]) * 800) + (400 + self.points[mappoint-2][0]) - width 
            end = start + width
            if i == 0 or i == height-1:
                for j in range(start, end):
                    map_l = list(self.map_D.data)
                    map_l[j] = 100
                    self.map_D.data = tuple(map_l)
            elif i != 0 and i != height-1:
                map_l = list(self.map_D.data)
                map_l[start] = 100
                map_l[end] = 100
                self.map_D.data = tuple(map_l)

    def remove(self, mappoint):
        width = abs(self.points[mappoint][1] - self.points[mappoint+1][1])
        height = abs(self.points[mappoint+1][0] - self.points[mappoint][0])
        
        for i in range(height):
            start = ((400-i+self.points[mappoint][1]) * 800) + (400 + self.points[mappoint][0]) - width 
            end = start + width
            if i == 0 or i == height-1:
                for j in range(start, end):
                    map_l = list(self.map_D.data)
                    map_l[j] = 0
                    self.map_D.data = tuple(map_l)
            elif i != 0 and i != height-1:
                map_l = list(self.map_D.data)
                map_l[start] = 0
                map_l[end] = 0
                self.map_D.data = tuple(map_l)
        


if __name__ == "__main__":
    m = mapAlter()
    m.mappub()
    rospy.spin()    
        