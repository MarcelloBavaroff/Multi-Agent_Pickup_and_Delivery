"""
Python implementation of Conflict-based search
author: Ashwin Bose (@atb033)
author: Giacomo Lodigiani (@Lodz97)
"""
import sys

sys.path.insert(0, '../')
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy

from Simulation.CBS.a_star2 import AStar


class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return str((self.x, self.y))


class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time) + str(self.location.x) + str(self.location.y))

    def is_equal_except_time(self, state):
        return self.location == state.location

    def __str__(self):
        return str((self.time, self.location.x, self.location.y))


class Conflict(object):
    VERTEX = 1
    EDGE = 2

    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
            ', ' + str(self.location_1) + ', ' + str(self.location_2) + ')'


class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash(str(self.time) + str(self.location))

    def __str__(self):
        return '(' + str(self.time) + ', ' + str(self.location) + ')'


class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2

    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2

    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))

    def __str__(self):
        return '(' + str(self.time) + ', ' + str(self.location_1) + ', ' + str(self.location_2) + ')'


class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints]) + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])


class Environment(object):
    # passo charging_stations come dizionario
    def __init__(self, dimension, agents, obstacles, moving_obstacles=None, a_star_max_iter=-1, charging_stations=None):
        if charging_stations is None:
            charging_stations = dict()
        if moving_obstacles is None:
            moving_obstacles = []
        self.dimension = dimension
        self.obstacles = obstacles
        self.moving_obstacles = moving_obstacles
        self.a_star_max_iter = a_star_max_iter

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

        self.charging_stations = charging_stations

    def get_neighbors(self, state, agent_name):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y + 1))
        if self.state_valid(n) and self.transition_valid(state, n) and self.station_to_extra_slot(state, n) and self.station_only_if_goal(state, n, agent_name):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y - 1))
        if self.state_valid(n) and self.transition_valid(state, n) and self.station_to_extra_slot(state, n) and self.station_only_if_goal(state, n, agent_name):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x - 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n) and self.station_to_extra_slot(state, n) and self.station_only_if_goal(state, n, agent_name):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x + 1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n) and self.station_to_extra_slot(state, n) and self.station_only_if_goal(state, n, agent_name):
            neighbors.append(n)
        return neighbors

    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t + 1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t + 1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def get_all_obstacles(self, time):
        all_obs = set()
        for o in self.moving_obstacles:
            if o[2] < 0 and time >= -o[2]:
                all_obs.add((o[0], o[1]))
        return self.obstacles | all_obs

    def state_valid(self, state):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.get_all_obstacles(state.time) \
            and (state.location.x, state.location.y, state.time) not in self.moving_obstacles

    def transition_valid(self, state_1, state_2):
        tup_1 = (state_1.location.x, state_1.location.y, state_2.time)  # attento che prende state time 2
        tup_2 = (state_2.location.x, state_2.location.y, state_1.time)
        if tup_1 in self.moving_obstacles and tup_2 in self.moving_obstacles and \
                self.moving_obstacles[tup_1] == self.moving_obstacles[tup_2]:
            return False

        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def station_to_extra_slot(self, state_old, state_new):

        # se posizione nuova è il primo extra_slot, ci puoi arrivare solo dalla stazione di ricarica corrispondente
        tup_new = [state_new.location.x, state_new.location.y]
        for s in self.charging_stations:
            if tup_new == s['queue'][0]:
                if state_old.location.x == s['pos'][0] and state_old.location.y == s['pos'][1]:
                    return True
                else:
                    return False

        return True

    # se la nuova posizione è una stazione, non ci posso arrivare dal mio extra slot
    def no_extra_slot_to_station(self, state_old, state_new):
        tup_new = [state_new.location.x, state_new.location.y]
        for s in self.charging_stations:
            if tup_new == s['pos']:
                if state_old.location.x == s['queue'][0] and state_old.location.y == s['queue'][1]:
                    return False
                else:
                    return True
        return True

    # posso andare in una stazione solo se ho come goal quella stazione
    def station_only_if_goal(self, state_old, state_new, agent_name):
        tup_new = [state_new.location.x, state_new.location.y]
        for s in self.charging_stations:
            if tup_new == s['pos']:
                if self.is_at_goal(state_new, agent_name):
                    return True
                else:
                    return False
        return True


    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)

    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']: {'start': start_state, 'goal': goal_state}})

    def compute_solution(self):
        solution = {}
        espansioniA = 0
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution, espansioniA = self.a_star.search(agent)
            if not local_solution:
                return False, espansioniA
            solution.update({agent: local_solution})
        return solution, espansioniA

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])


class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost


class CBS(object):
    def __init__(self, environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()

    def search(self):
        start = HighLevelNode()
        # espansioniA = 0
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution, espansioniA = self.env.compute_solution()
        if not start.solution:
            return {}, espansioniA
        start.cost = self.env.compute_solution_cost(start.solution)

        # aggiungo start ad open set
        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = self.env.get_first_conflict(P.solution)
            if not conflict_dict:
                # print("Low level CBS - Solution found")

                return self.generate_plan(P.solution), espansioniA

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}, espansioniA

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t': state.time, 'x': state.location.x, 'y': state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-param", help="input file containing map and obstacles")
    parser.add_argument("-output", help="output file with the schedule")
    args = parser.parse_args()

    if args.param is None:
        args.param = 'input.yaml'
        args.output = 'output.yaml'

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    obstacles = param["map"]["obstacles"]
    agents = param['agents']

    env = Environment(dimension, agents, obstacles, a_star_max_iter=1000)

    # Searching
    cbs = CBS(env)
    solution = cbs.search()
    if not solution:
        print("Solution not found")
        exit(0)

    # Write to output file
    with open(args.output, 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    output["schedule"] = solution
    output["cost"] = env.compute_solution_cost(solution)
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)
