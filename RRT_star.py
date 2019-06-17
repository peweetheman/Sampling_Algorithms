from Graph import Graph
from Node import Node
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import math
import copy


# maybe excessively tracking by keeping track of graph edges and node parents, same for basic RRT, only small memory waste probably

class RRT_star():
    def __init__(self, start, goal, space, growth, obstacle_list, maxIter):
        """
        :param start:
        :param goal:
        :param space:
        :param growth:
        :param obstacle_list:
        :param maxIter:
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.goal_percent = 5
        self.space = space
        self.growth = growth
        self.obstacle_list = obstacle_list
        self.maxIter = maxIter
        self.graph = Graph()
        self.graph.add_vertex(self.start)

    def plan(self):
        for i in range(0, self.maxIter):
            sample = self.sample_free()
            self.draw_graph(sample)
            nearest = self.nearest_node(sample)
            self.draw_graph(nearest)
            new_node = self.steer(nearest, sample)

            if self.collision_check(new_node):
                continue

            near = self.find_near_nodes(new_node)
            new_node = self.connect_min_path(new_node, near)
            self.graph.add_vertex(new_node)
            self.rewire(new_node, near)
        path = self.get_path()
        return self.graph, path

    def sample_free(self):
        if random.randint(0, 100) < self.goal_percent:
            sample = Node(self.goal.x, self.goal.y)
        else:
            minSample = self.space[0]  # min bound on space
            maxSample = self.space[1]  # max bound on space
            sample = Node(random.uniform(minSample, maxSample),
                      random.uniform(minSample, maxSample))
        return sample

    def nearest_node(self, sample):  ##using L2 Euclidean distance
        distances = [(node.x - sample.x) ** 2 + (node.y - sample.y)
                     ** 2 for node in self.graph.vertices()]
        index = distances.index(min(distances))
        nearest = self.graph.vertices()[index]
        return nearest

    def find_near_nodes(self, new_node):
        # gamma_star = 2(1+1/d) ** (1/d) volume(free)/volume(total) ** 1/d and we need gamma > gamma_star
        # for asymptotical completeness see Kalman 2011. gamma = 1 satisfies
        d = 2  # dimension of the self.space
        card = len(self.graph.vertices())  # cardinality of vertex set
        r = min(math.pow(math.log(card) / card, 1 / d), self.growth)
        cost_list = [self.cost(node, new_node) for node in self.graph.vertices()]
        near_indices = [cost_list.index(dist) for dist in cost_list if dist <= r]
        near = [self.graph.vertices()[i] for i in near_indices]
        return near


    def connect_min_path(self, new_node, near):
        if not near:
            print("not near")
            return new_node

        cost_list = []
        for near_node in near:
            # angle = math.atan2(dy, dx)
            new_cost = self.cost(near_node, new_node)
            if not self.collision_check_on_path(near_node, new_node):
                cost_list.append(near_node.cost + new_cost)
            else:
                cost_list.append(float("inf"))
        min_cost = min(cost_list)
        min_node = near[cost_list.index(min_cost)]

        if min_cost == float("inf"):
            print("mincost is infinity?!? blame Patrick")
            return new_node

        new_node.cost = min_cost
        new_node.parent = min_node
        self.graph.add_edge({min_node, new_node})
        return new_node


    def rewire(self, new_node, near):
        for near_node in near:
            if not self.collision_check_on_path(new_node, near_node):
                if (new_node.cost + self.cost(new_node, near_node) < near_node.cost):
                    near_node.parent = new_node
                    near_node.cost = new_node.cost + self.cost(new_node, near_node)
                    self.graph.remove_edge({near_node.parent, near_node})
                    self.graph.add_edge({new_node, near_node})


    def steer(self, nearest, sample):
        angle = math.atan2(sample.y - nearest.y, sample.x - nearest.x)
        new_node = copy.deepcopy(nearest)
        new_node.x += math.cos(angle) * self.growth
        new_node.y += math.sin(angle) * self.growth
        return new_node


    def collision_check_on_path(self, node1, node2):
        temp_node = copy.deepcopy(node1)
        dy = node2.y - node1.y
        dx = node2.x - node1.x
        d = math.sqrt(dx ** 2 + dy ** 2)
        angle = math.atan2(dy, dx)
        for i in range(int(d / self.growth)):
            temp_node.x += self.growth * math.cos(angle)
            temp_node.y += self.growth * math.sin(angle)
            if self.collision_check(temp_node):
                print("collision")
                return True  # collision
        return False  # no collision


    def collision_check(self, node):
        for (ox, oy, size) in self.obstacle_list:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if math.sqrt(d) <= size + .3:
                return True  # collision

        return False  # no collision


    def get_path(self):
        dist2goal = [self.cost(node, self.goal) for node in self.graph.vertices()]
        goalinds = [dist2goal.index(i) for i in dist2goal if i <= self.growth]

        if not goalinds:
            return None

        mincost = min([self.graph.vertices()[i].cost for i in goalinds])
        last = self.graph.vertices().index(mincost)

        path = [[last.x, last.y]]
        while last.parent is not None:
            node = last
            path.append([node.x, node.y])
            last = node.parent
        return path


    def draw_graph(self, sample = None):
        plt.clf()

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "og", ms=21 * size)
        if sample is not None:
            plt.plot(sample.x, sample.y, "^k")
        for node in self.graph.vertices():
            plt.plot(node.x, node.y, "yH")
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [
                    node.y, node.parent.y], "-k")

        plt.plot(self.start.x, self.start.y, "yH")
        plt.plot(self.goal.x, self.goal.y, "rH")
        plt.axis([self.space[0], self.space[1], self.space[0], self.space[1]])
        plt.grid(True)
        plt.title('RRT*')
        plt.pause(.01)

    def cost(self, node, new_node):
        return math.sqrt((node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2)


def main():
    obstacle_list = [
        (5, 5, 3),
    ]  # [x,y,size(radius)]
    plt.show()

    rrt = RRT_star(start=[1, 1], goal=[20, 15],  space=[0, 30], growth=.5, obstacle_list=obstacle_list, maxIter=1000)
    graph, path = rrt.plan()
    print("path length: ", len(path))
    for i in range(1, len(path)):
        plt.plot([path[i][0], path[i - 1][0]], [path[i][1], path[i - 1][1]], "-b")

main()