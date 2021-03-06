import matplotlib.pyplot as plt
import math
import copy
import random

show_animation = True


class CONNECT_RRT:
    def __init__(self, start, goal, obstaclelist, randaera, randsample=10, expanddis=0.5, maxiteration=5000):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstaclelist = obstaclelist
        self.minrand = randaera[0]
        self.maxrand = randaera[1]
        self.expanddis = expanddis
        self.maxiteration = maxiteration
        self.randsample = randsample
        self.iteration = 0

    def planning(self, animation=False):
        self.nodelistA = [self.start]             # TREE_A用于从起点向终点方向扩展
        self.nodelistB = [self.goal]              # TREE_B用于从终点向起点方向扩展
        while True:
            rnd = self.get_random(self.iteration) # Get a random point to extend
            if self.iteration % 2 == 0:           # 迭代次数为偶数（从0开始）领TREE_A进行扩展，TREE_B进行连接，奇数相反
                new_node = self.Extend(self.nodelistA, rnd)
            else:
                new_node = self.Extend(self.nodelistB, rnd)

            #  start to Connect  #
            if self.iteration % 2 == 0:    # 迭代次数为偶数（从0开始）领TREE_A进行扩展，TREE_B进行连接，奇数相反
                last_point = self.Extend(self.nodelistB, [new_node.x, new_node.y])
            else:
                last_point = self.Extend(self.nodelistA, [new_node.x, new_node.y])

            if math.sqrt((last_point.x - new_node.x) ** 2 + (last_point.y - new_node.y) ** 2) < self.expanddis:
                print('goal!!')
                break
            self.iteration += 1
        # 生成最终路径
        path = []
        Lastindex = len(self.nodelistB) - 1
        while self.nodelistB[Lastindex].parent is not None:
            node = self.nodelistB[Lastindex]
            path.append([node.x, node.y])
            Lastindex = node.parent
        path.append([self.goal.x, self.goal.y])
        path.reverse()
        Lastindex = len(self.nodelistA) - 1
        while self.nodelistA[Lastindex].parent is not None:
            node = self.nodelistA[Lastindex]
            path.append([node.x, node.y])
            Lastindex = node.parent

        return path

    def Extend(self, nodelist, rnd, animation=False):
        while True:
            nind = self.GetNearestIndex(nodelist, rnd)
            nearestnode = nodelist[nind]
            last_node = nearestnode
            theta = math.atan2(rnd[1] - nearestnode.y, rnd[0] - nearestnode.x)
            newnode = copy.deepcopy(nearestnode)
            newnode.x += self.expanddis * math.cos(theta)
            newnode.y += self.expanddis * math.sin(theta)
            newnode.parent = nind
            if not self.__checkcollision(newnode, self.obstaclelist):
                return last_node                                # Trapped
            nodelist.append(newnode)
            last_node = newnode
            # print('index: ' + str(len(nodelist)) + '   location： x: ' +
                  # str(round(newnode.x, 2))+ ' y: ' + str(round(newnode.y, 2)))
            print(len(nodelist))

            if math.sqrt((rnd[0] - newnode.x) ** 2 + (rnd[1] - newnode.y) ** 2) < self.expanddis:
                return last_node                                 # Reached
            if animation:
                self.drawgraph(rnd)

    def drawgraph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], '^k')
        for node in self.nodelistA:
            if node.parent is not None:
                plt.plot([node.x, self.nodelistA[node.parent].x],[node.y, self.nodelistA[node.parent].y], '-g')
        for node in self.nodelistB:
            if node.parent is not None:
                plt.plot([node.x, self.nodelistB[node.parent].x],[node.y, self.nodelistB[node.parent].y], '-g')

        for (node_x, node_y, size) in self.obstaclelist:
            plt.plot(node_x, node_y, 'ok', markersize=30*size)
        plt.plot(self.goal.x, self.goal.y, 'xr')
        plt.plot(self.start.x, self.start.y, 'xr')
        plt.axis([self.minrand, self.maxrand, self.minrand, self.maxrand])
        plt.grid(True)
        plt.pause(0.001)

    def __checkcollision(self, node, obstacle):
        for (ox, oy, size) in obstacle:
            dis = math.sqrt((node.x - ox) ** 2 + (node.y - oy) ** 2)
            if dis < size:
                return False
        return True

    def GetNearestIndex(self, nodelist, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodelist]
        minid = dlist.index(min(dlist))
        return minid

    def get_random(self, k):
        if random.randint(0, 100) > self.randsample:
            rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(self.minrand, self.maxrand)]
        else:
            if k % 2 == 0:
                rnd = [self.goal.x, self.goal.y]
            else:
                rnd = [self.start.x, self.start.y]
        return rnd


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main(gx=18.0, gy=18.0):
    obstaclelist = [
        (5, 5, 1),(3, 6, 2),(3, 8, 2),(3, 10, 2),(7, 5, 2),(9, 5, 2),
        (5,1,1.2),(5,3,1),(5,5,1),(5,7,1),(5,9,1),(5,11,1),(5,13,1),(5,15,1),(5,17,1),(5,19,1)]
        #(5,11,1),(5,12,1),(5,13,1),(5,16,3),(11,1,1),(11,3,1),(11,5,1),(11,7,1),(11,9,1),(11,11,1),(11,13,1),(11,15,1),(11,17,1),(13,4,1),
#(8,14,0.5),(11,-1,1.2),(11,-2,1.2),(5,4,1),(5,6,1),(5,8,1),(5,10,1),(5,12,1),(5,14,1),(5,16,1),(5,18,1),(5,20,1),
        #(5,12,1),(5,14,1),(5,19,1),(5,16,3),(11,2,1),(11,4,1),(11,6,1),(11,8,1),(11,10,1),(11,12,1),(11,14,1),(11,16,1),(11,18,1),(13,20,1),
#(8,14,0.5),(11,-1,1.2),(11,-2,1.2),(5,20,1),(11,0,1)]                                       # 用于生成障碍物节点信息
    start = [0.0, 0.0]
    rrt = CONNECT_RRT(start, goal=[gx, gy], obstaclelist=obstaclelist, randaera = [-2, 20])
    path = rrt.planning()
    if show_animation:
        rrt.drawgraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()
