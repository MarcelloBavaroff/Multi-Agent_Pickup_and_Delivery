import argparse
import yaml
import json
import os
import time
from collections import defaultdict
from Simulation.TP_with_recovery import TokenPassingRecovery
import RoothPath
from Simulation.tasks_and_delays_maker import *


class SimulationNewRecovery(object):
    def __init__(self, tasks, agents, autonomies, charging_stations):
        random.seed(1234)
        self.tasks = tasks
        self.agents = agents
        self.time = 0
        self.start_times = []  # inizio dei task dei robot
        self.agents_pos_now = set()
        self.agents_moved = set()
        self.actual_paths = {}
        self.algo_time = 0
        self.initialize_simulation()
        self.max_autonomies = {}
        self.batteries_level = {}
        self.charging_stations = charging_stations
        self.move_consumption = 0.5
        self.wait_consumption = 0.05

        for i, a in enumerate(self.agents):
            self.max_autonomies[a['name']] = autonomies[i]
            self.batteries_level[a['name']] = autonomies[i]

    def initialize_simulation(self):
        for t in self.tasks:
            self.start_times.append(t['start_time'])
        for agent in self.agents:
            # x e y del path sono presi da 'start' dell'agente (posizione 0 e 1)
            self.actual_paths[agent['name']] = [{'t': 0, 'x': agent['start'][0], 'y': agent['start'][1]}]

    def handle_agents_lenPath1(self, agents_to_move, algorithm):

        for agent in agents_to_move:
            # ultimo elemento della lista dei path
            current_agent_pos = self.actual_paths[agent['name']][-1]
            # aggiorno posizione attuale agenti
            self.agents_pos_now.add(tuple([current_agent_pos['x'], current_agent_pos['y']]))

            if len(algorithm.get_token()['agents'][agent['name']]) == 1:
                self.agents_moved.add(agent['name'])
                self.actual_paths[agent['name']].append(
                    {'t': self.time, 'x': current_agent_pos['x'], 'y': current_agent_pos['y']})

                # se in fase di ricarica aumento il livello della sua batteria (upper bound autonomia massima)
                if agent['name'] in algorithm.get_token()['agents_to_tasks'] and \
                        algorithm.get_token()['agents_to_tasks'][agent['name']]['task_name'] == 'recharging':

                    self.batteries_level[agent['name']] += 10
                    # se carica completa lo metto in idle?
                    if self.batteries_level[agent['name']] >= self.max_autonomies[agent['name']]:
                        self.batteries_level[agent['name']] = self.max_autonomies[agent['name']]
                        algorithm.set_task_name(agent['name'], 'charge_complete')

                # abbasso livello di batteria
                elif self.actual_paths[agent['name']][self.time]['x'] == \
                        self.actual_paths[agent['name']][self.time - 1]['x'] and \
                        self.actual_paths[agent['name']][self.time]['y'] == \
                        self.actual_paths[agent['name']][self.time - 1]['y']:

                    self.batteries_level[agent['name']] -= self.wait_consumption
                else:
                    self.batteries_level[agent['name']] -= self.move_consumption

    def update_actual_paths(self, agent, algorithm, x_new, y_new, current_agent_pos):
        self.agents_moved.add(agent['name'])
        # dico che c'è un agente in questa posizione
        self.agents_pos_now.remove(tuple([current_agent_pos['x'], current_agent_pos['y']]))
        self.agents_pos_now.add(tuple([x_new, y_new]))

        # cancello il primo
        algorithm.get_token()['agents'][agent['name']] = algorithm.get_token()['agents'][agent['name']][
                                                         1:]
        # aggiorno il path dell'agente
        self.actual_paths[agent['name']].append({'t': self.time, 'x': x_new, 'y': y_new})
        if self.actual_paths[agent['name']][self.time]['x'] == \
                self.actual_paths[agent['name']][self.time - 1]['x'] and \
                self.actual_paths[agent['name']][self.time]['y'] == \
                self.actual_paths[agent['name']][self.time - 1]['y']:

            self.batteries_level[agent['name']] -= self.wait_consumption
        else:
            self.batteries_level[agent['name']] -= self.move_consumption

    def handle_loops(self, agents_to_move, algorithm):

        actual_pos = []
        for a in agents_to_move:
            actual_pos.append(tuple(algorithm.get_token()['agents'][a['name']][0]))

        correspondences = 0
        for a in agents_to_move:
            #next_pos è una lista, non una tupla
            next_pos = algorithm.get_token()['agents'][a['name']][1]
            if tuple(next_pos) in actual_pos:
                correspondences += 1
            else:
                break
        if correspondences == len(agents_to_move):
            return True
        else:
            return False

    # questa viene chiamata per simulare un singolo timestep in avanti
    def time_forward(self, algorithm):
        self.time = self.time + 1
        print('Time:', self.time)
        start_time = time.time()

        algorithm.time_forward()

        self.algo_time += time.time() - start_time
        self.agents_pos_now = set()
        self.agents_moved = set()
        agents_to_move = self.agents
        random.shuffle(agents_to_move)

        # First "move" idle agents
        self.handle_agents_lenPath1(agents_to_move, algorithm)

        # Check moving agents doesn't collide with others
        agents_to_move = [x for x in agents_to_move if x['name'] not in self.agents_moved]
        moved_this_step = -1
        while moved_this_step != 0:
            moved_this_step = 0

            for agent in agents_to_move:
                current_agent_pos = self.actual_paths[agent['name']][-1]
                if len(algorithm.get_token()['agents'][agent['name']]) > 1:
                    # accedo alla tupla della posizione
                    x_new = algorithm.get_token()['agents'][agent['name']][1][0]
                    y_new = algorithm.get_token()['agents'][agent['name']][1][1]

                    # in pratica non sapendo l'ordine in cui devo muovere gli agenti se becco
                    # che uno occupa la posizione dove dovrei andare vengo rimesso in coda per
                    # muovermi
                    if tuple([x_new, y_new]) not in self.agents_pos_now or \
                            tuple([x_new, y_new]) == tuple(tuple([current_agent_pos['x'], current_agent_pos['y']])):

                        self.update_actual_paths(agent, algorithm, x_new, y_new, current_agent_pos)

                        moved_this_step = moved_this_step + 1

            agents_to_move = [x for x in agents_to_move if x['name'] not in self.agents_moved]

        agents_to_move = [x for x in agents_to_move if x['name'] not in self.agents_moved]
        if len(agents_to_move) != 0:
            if self.handle_loops(agents_to_move, algorithm):
                for agent in agents_to_move:
                    current_agent_pos = self.actual_paths[agent['name']][-1]
                    if len(algorithm.get_token()['agents'][agent['name']]) > 1:
                        # accedo alla tupla della posizione
                        x_new = algorithm.get_token()['agents'][agent['name']][1][0]
                        y_new = algorithm.get_token()['agents'][agent['name']][1][1]

                        self.update_actual_paths(agent, algorithm, x_new, y_new, current_agent_pos)


            else:
                print('attenzione qualcosa non va')


    def get_time(self):
        return self.time

    def get_algo_time(self):
        return self.algo_time

    def get_actual_paths(self):
        return self.actual_paths

    def get_new_tasks(self):
        new = []
        for t in self.tasks:
            if t['start_time'] == self.time:
                new.append(t)
        return new

    def get_max_autonomies(self):
        return self.max_autonomies
    def get_batteries_level(self):
        return self.batteries_level

    def get_move_consumption(self):
        return self.move_consumption

    def get_wait_consumption(self):
        return self.wait_consumption


if __name__ == '__main__':
    random.seed(1234)
    parser = argparse.ArgumentParser()
    parser.add_argument('-param', help='Input file containing map and obstacles')
    parser.add_argument('-output', help='Output file with the schedule')
    args = parser.parse_args()

    if args.param is None:
        with open(os.path.join(RoothPath.get_root(), 'config.json'), 'r') as json_file:
            config = json.load(json_file)
        args.param = os.path.join(RoothPath.get_root(), os.path.join(config['input_path'], config['input_name']))
        args.output = os.path.join(RoothPath.get_root(), 'output.yaml')

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimensions = param['map']['dimensions']
    obstacles = param['map']['obstacles']
    non_task_endpoints = param['map']['non_task_endpoints']
    agents = param['agents']
    # Generate random tasks and delays
    tasks, delays = gen_tasks(param['map']['start_locations'], param['map']['goal_locations'], param['n_tasks'],
                              param['task_freq'])
    param['tasks'] = tasks
    param['delays'] = delays
    with open(args.param + config['visual_postfix'], 'w') as param_file:
        yaml.safe_dump(param, param_file)

    # Simulate
    simulation = SimulationNewRecovery(tasks, agents)
    tp = TokenPassingRecovery(agents, dimensions, obstacles, non_task_endpoints, simulation, a_star_max_iter=4000,
                              new_recovery=True)
    while tp.get_completed_tasks() != len(tasks):
        simulation.time_forward(tp)

    cost = 0
    for path in simulation.actual_paths.values():
        cost = cost + len(path)
    output = {'schedule': simulation.actual_paths, 'cost': cost,
              'completed_tasks_times': tp.get_completed_tasks_times(),
              'n_replans': tp.get_n_replans()}
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)
