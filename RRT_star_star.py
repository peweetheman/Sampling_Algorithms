from Node import Node
import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt


class RRT_star():
    # Basic RRT* algorithm using distance as cost function
    
    def __init__(self, start, end, space, obstacles, growth=0.5, max_iter=500, end_sample_percent=10):
        """

        :param start: [x,y] starting location
        :param end: [x,y[ ending location
        :param space: [min,max] bounds on square space
        :param obstacles: list of square obstacles
        :param growth: size of growth each new sample
        :param max_iter: max number of iterations for algorithm
        :param end_sample_percent: percent chance to get sample from goal location
        """
        self.start = Node(start[0], start[1])
        self.end = Node(end[0], end[1])
        self.space = space
        self.growth = growth
        self.end_sample_percent = end_sample_percent
        self.max_iter = max_iter
        self.obstacles = obstacles

    def rrt_star_algorithm(self):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            sample = self.get_sample()
            nearest_node = self.nearest_node(self.node_list, sample)
            new_node = self.steer(sample, nearest_node)

            if self.check_collision(new_node, self.obstacles):
                near_nodes = self.get_near_nodes(new_node)
                self.set_parent(new_node, near_nodes)
                self.node_list.append(new_node)
                self.rewire(new_node, near_nodes)
            # draw added edges
            self.draw_graph()

        # generate path
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def get_sample(self):

        if random.randint(0, 100) > self.end_sample_percent:
            sample = Node(random.uniform(self.space[0], self.space[1]),
                          random.uniform(self.space[0], self.space[1]))
        else:  # end point sampling
            sample = Node(self.end.x, self.end.y)

        return sample

    def steer(self, sample, nearest_node):
        # take nearest_node and expand in direction of sample
        angle = math.atan2(sample.y - nearest_node.y, sample.x - nearest_node.x)
        new_node = Node(sample.x, sample.y)
        currentDistance = self.dist(sample, nearest_node)
        # find a point within growth of nearest_node, and closest to sample
        if currentDistance <= self.growth:
            pass
        else:
            new_node.x = nearest_node.x + self.growth * math.cos(angle)
            new_node.y = nearest_node.y + self.growth * math.sin(angle)
        new_node.cost = float("inf")
        new_node.parent = None
        return new_node

    def set_parent(self, new_node, near_nodes):
        # connects new_node along a minimum cost(distance) path
        if not near_nodes:
            return
        dlist = []
        for near_node in near_nodes:
            dist = self.dist(new_node, near_node)
            if self.check_collision_path(near_node, new_node):
                dlist.append(near_node.cost + dist)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        min_node = near_nodes[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return
        new_node.cost = mincost
        new_node.parent = min_node

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_end(
            node.x, node.y) for node in self.node_list]
        endinds = [disglist.index(i) for i in disglist if i <= self.growth]

        if not endinds:
            return None

        mincost = min([self.node_list[i].cost for i in endinds])
        for i in endinds:
            if self.node_list[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, endind):
        path = [[self.end.x, self.end.y]]
        while self.node_list[endind].parent is not None:
            node = self.node_list[endind]
            path.append([node.x, node.y])
            endind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_end(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def get_near_nodes(self, new_node):
        # gamma_star = 2(1+1/d) ** (1/d) volume(free)/volume(total) ** 1/d and we need gamma > gamma_star
        # for asymptotical completeness see Kalman 2011. gamma = 1 satisfies
        d = 2  # dimension of the self.space
        nnode = len(self.node_list)
        r = min(50.0 * ((math.log(nnode) / nnode)) ** (1/d), self.growth * 5.0)
        dlist = [(node.x - new_node.x) ** 2 +
                 (node.y - new_node.y) ** 2 for node in self.node_list]
        near_nodes = [self.node_list[dlist.index(i)] for i in dlist if i <= r ** 2]
        return near_nodes

    def rewire(self, new_node, near_nodes):
        for near_node in near_nodes:
            dx = new_node.x - near_node.x
            dy = new_node.y - near_node.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = new_node.cost + d

            if near_node.cost > scost:
                if self.check_collision_path(near_node, new_node):
                    near_node.parent = new_node
                    near_node.cost = scost

    def check_collision_path(self, node1, node2):
        # check for collision on path from node1 to node2
        dist = self.dist(node1, node2)
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        angle = math.atan2(dy, dx)
        temp_node = copy.deepcopy(node1)
        for i in range(int(dist / self.growth)):
            temp_node.x += self.growth * math.cos(angle)
            temp_node.y += self.growth * math.sin(angle)
            if not self.check_collision(temp_node, self.obstacles):
                return False

        return True

    def check_collision(self, node, obstacles):
        for (x, y, side) in obstacles:
            if ((node.x > x - .8 * side/2) & (node.x<x+ .8 * side/2) & (node.y>y - side/2) & (node.y<y+side/2)):
                return False  # collision

        return True  # safe

    def nearest_node(self, node_list, sample):
        dlist = [self.dist(node, sample) for node in node_list]
        min_node = self.node_list[dlist.index(min(dlist))]
        return min_node
    
    def dist(self, node1, node2):
        # returns distance between two nodes
        return math.sqrt((node2.x - node1.x)**2 + (node2.y - node1.y) ** 2)

    def draw_graph(self):
        plt.clf()
        for node in self.node_list:
            plt.plot(node.x, node.y, "yH")
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [
                         node.y, node.parent.y], "-k")

        for (x, y, side) in self.obstacles:
            plt.plot(x, y, "sk", ms=8 * side)

        plt.plot(self.start.x, self.start.y, "oy")
        plt.plot(self.end.x, self.end.y, "or")
        plt.axis([0, 30, 0, 30])
        plt.grid(True)
        plt.title("RRT* (distance cost function)")
        plt.pause(0.01)


def main():
    # squares of [x,y,side length]
    obstacles = [
        (14, 17, 5),
        (4, 10, 4),
        (7, 23, 3),
        (19, 12, 5),
        (9, 15, 4)]

    #calling RRT*
    rrt_star = RRT_star(start=[15, 28], end=[15, 5], obstacles=obstacles, space=[0, 30])
    path = rrt_star.rrt_star_algorithm()
    
    #plotting code
    rrt_star.draw_graph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.show()


if __name__ == '__main__':
    main()