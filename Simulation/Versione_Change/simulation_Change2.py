import time
from Simulation.tasks_and_delays_maker import *


class Simulation(object):
    def __init__(self, tasks, agents, autonomies, charging_stations, move_consumption=0.5, wait_consumption=0.05,
                 move_heavy_consumption=1):
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
        self.move_consumption = move_consumption
        self.wait_consumption = wait_consumption
        self.move_heavy_consumption = move_heavy_consumption

        for i, a in enumerate(self.agents):
            self.max_autonomies[a['name']] = autonomies[i]
            self.batteries_level[a['name']] = autonomies[i]

    def initialize_simulation(self):
        for t in self.tasks:
            self.start_times.append(t['start_time'])
        for agent in self.agents:
            # x e y del path sono presi da 'start' dell'agente (posizione 0 e 1)
            self.actual_paths[agent['name']] = [{'t': 0, 'x': agent['start'][0], 'y': agent['start'][1]}]

    def update_batteries(self, algorithm, agent, update_actuale_path=True):
        if agent['name'] in algorithm.get_token()['agents_to_tasks']:
            task_name = algorithm.get_token()['agents_to_tasks'][agent['name']]['task_name']
        else:
            task_name = None

        if task_name in algorithm.get_token()['start_tasks_times'] \
                and algorithm.get_token()['agents_to_tasks'][agent['name']]['start'] not in \
                algorithm.get_token()['agents'][agent['name']]:

            effective_move_consumption = self.move_heavy_consumption
        else:
            effective_move_consumption = self.move_consumption

        # abbasso livello di batteria
        if self.actual_paths[agent['name']][self.time]['x'] == \
                self.actual_paths[agent['name']][self.time - 1]['x'] and \
                self.actual_paths[agent['name']][self.time]['y'] == \
                self.actual_paths[agent['name']][self.time - 1]['y']:

            self.batteries_level[agent['name']] = round(
                self.batteries_level[agent['name']] - self.wait_consumption, 2)
        else:
            self.batteries_level[agent['name']] = round(
                self.batteries_level[agent['name']] - effective_move_consumption, 2)

            if update_actuale_path:
                if agent['name'] in algorithm.get_occupied_stations():

                    station_name = algorithm.get_occupied_stations()[agent['name']]
                    station_pos = tuple(algorithm.get_token()['charging_stations'][station_name]['pos'])
                    agent_old_pos = tuple((self.actual_paths[agent['name']][self.time - 1]['x'],
                                           self.actual_paths[agent['name']][self.time - 1]['y']))

                    if agent_old_pos == station_pos:
                        algorithm.set_free_station(algorithm.get_occupied_stations()[agent['name']])
                        algorithm.remove_occupied_station(agent['name'])

        if self.batteries_level[agent['name']] < 0:
            print("Batteria negativa")

    def handle_agents_lenPath1(self, agents_to_move, algorithm):

        for agent in agents_to_move:
            # ultimo elemento della lista dei path
            current_agent_pos = self.actual_paths[agent['name']][-1]
            # aggiorno posizione attuale agenti
            self.agents_pos_now.add(tuple([current_agent_pos['x'], current_agent_pos['y']]))

            # se in fase di ricarica aumento il livello della sua batteria (upper bound autonomia massima)
            if agent['name'] in algorithm.get_token()['agents_to_tasks'] and \
                    algorithm.get_token()['agents_to_tasks'][agent['name']]['task_name'] == 'recharging':

                self.agents_moved.add(agent['name'])
                self.actual_paths[agent['name']].append(
                    {'t': self.time, 'x': current_agent_pos['x'], 'y': current_agent_pos['y']})
                algorithm.get_token()['agents'][agent['name']] = algorithm.get_token()['agents'][agent['name']][
                                                                 1:]

                # self.batteries_level[agent['name']] += 10
                self.batteries_level[agent['name']] = round(self.batteries_level[agent['name']] + 10, 2)

                # se carica completa lo metto in idle?
                if self.batteries_level[agent['name']] >= self.max_autonomies[agent['name']]:
                    self.batteries_level[agent['name']] = self.max_autonomies[agent['name']]

                    algorithm.set_task_name(agent['name'], 'charge_complete')

            elif len(algorithm.get_token()['agents'][agent['name']]) == 1:
                self.agents_moved.add(agent['name'])
                self.actual_paths[agent['name']].append(
                    {'t': self.time, 'x': current_agent_pos['x'], 'y': current_agent_pos['y']})

                try:
                    self.update_batteries(algorithm, agent, update_actuale_path=False)
                except:
                    print("errore self.update_batteries")

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

        try:
            self.update_batteries(algorithm, agent, True)
        except:
            print("errore self.update_batteries")


    def handle_loops(self, agents_to_move, algorithm):

        actual_pos = []
        for a in agents_to_move:
            actual_pos.append(tuple(algorithm.get_token()['agents'][a['name']][0]))

        correspondences = 0
        for a in agents_to_move:
            # next_pos è una lista, non una tupla
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
                    # PS secondo me potrei muoverli tutti assieme
                    if tuple([x_new, y_new]) not in self.agents_pos_now or \
                            tuple([x_new, y_new]) == tuple(tuple([current_agent_pos['x'], current_agent_pos['y']])):
                        self.update_actual_paths(agent, algorithm, x_new, y_new, current_agent_pos)

                        moved_this_step = moved_this_step + 1

            agents_to_move = [x for x in agents_to_move if x['name'] not in self.agents_moved]

        agents_to_move = [x for x in agents_to_move if x['name'] not in self.agents_moved]
        if len(agents_to_move) > 3:
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

    def get_heavy_consumption(self):
        return self.move_heavy_consumption
