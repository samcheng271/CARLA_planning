import carla
import random
import time
from queue import PriorityQueue

class DStarLite:
    def __init__(self, world, start_waypoint, end_waypoint):
        self.world = world
        self.start = start_waypoint
        self.goal = end_waypoint
        self.U = PriorityQueue()
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.epsilon = 1

    def heuristic(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)

    def calculate_key(self, s):
        if s not in self.g:
            self.g[s] = float('inf')
        if s not in self.rhs:
            self.rhs[s] = float('inf')
        return [
            min(self.g[s], self.rhs[s]) + self.epsilon * self.heuristic(s, self.start) + self.km,
            min(self.g[s], self.rhs[s])
        ]

    def initialize(self):
        self.U = PriorityQueue()
        self.km = 0
        for s in self.world.get_map().generate_waypoints(2.0):
            self.rhs[s] = float('inf')
            self.g[s] = float('inf')
        self.rhs[self.goal] = 0
        self.U.put((self.calculate_key(self.goal), self.goal))


    def update_vertex(self, u):
        if u != self.goal:
            self.rhs[u] = min(self.c(u, s) + self.g.get(s, inf) for s in self.succ(u))
        if self.g.get(u, inf) != self.rhs.get(u, inf):
            if u in self.U:
                self.U.update(u, self.calculate_key(u))
            else:
                self.U.insert(u, self.calculate_key(u))
        elif (self.g(u) == self.rhs(u) and u in self.U):
            self.U.remove(u)

    def ComputeShortestPath(self):
        while (self.U.TopKey() < self.calculate_key(self.s_start) or
            self.rhs(self.s_start) > self.g(self.s_start)):
            u = self.U.Top()
            k_old = self.U.TopKey()
            k_new = self.calculate_key(u)

        if k_old < k_new:
            self.U.Update(u, k_new)
        elif self.g(u) > self.rhs(u):
            self.g[u] = self.rhs(u)
            self.U.Remove(u)
            #write Pred
            for s in self.Pred(u):
                if s != self.s_goal:
                    self.rhs[s] = min(self.rhs(s), self.c(s, u) + self.g(u))
                self.UpdateVertex(s)
        else:
            g_old = self.g(u)
            self.g[u] = float('inf')
            for s in self.Pred(u) | {u}:
                if self.rhs(s) == self.c(s, u) + g_old:
                    if s != self.s_goal:
                        min_succ = float('inf')
                        for s_prime in succ:
                        new = min(self.c(s, s_prime) + self.g(s_prime)

                            if(min_succ > new):
                                min_succ = new
                        self.rhs[s] = min_succ
                self.UpdateVertex(s)


    def main():
        client = carla.Client('localhost', 4000)
        client.set_timeout(10.0)

        world = client.get_world()
        carla_map = world.get_map()

        spawn_points = carla_map.get_spawn_points()
        start_transform = random.choice(spawn_points)
        end_transform = random.choice(spawn_points)

        start_waypoint = carla_map.get_waypoint(start_transform.location)
        end_waypoint = carla_map.get_waypoint(end_transform.location)

        slast = sstart

        self.Initialize()
        self.ComputeShortestPath()

        while(sstart != sgoal):
            self.s_start = min(self.succ(self.s_start), key=lambda s: self.c(self.s_start, s) + self.g.get(s, inf))
            self.move_to(self.s_start)

            self.scan_graph()

            if c_change():
                self.km += self.heuristic(self.s_last, self.s_start)
                self.s_last = self.s_start

                for u,v in self.c_change():
                    c_old = self.c_old(u, v)
                    self.c_change(u, v)


                if(c_old > self.c(u,v)):
                    if(u != self.s_goal):
                        self.rhs[u] = min(self.rhs[u], self.c(u,v) + self.g.get(v, inf))
                elif(self.rhs.get(u, inf) ==  c_old + self.g.get(v, inf)):
                    if(u != self.s_goal):
                        self.rhs[u] = min(self.c(u, s_prime) + self.g.get(s_prime, inf) for s_prime in self.succ(u))

                self.UpdateVertex(u)
                self.ComputeShortestPath()

