"""
Python implementation of Token Passing algorithms to solve MAPD problems with delays
author: Giacomo Lodigiani (@Lodz97)
"""
import math
from math import fabs
import random
from Simulation.CBS.cbs import CBS, Environment

# from Simulation.markov_chains import MarkovChainsMaker
# from collections import defaultdict

# Praticamente aggiungo al set idle_obstacles_agents tutti i delivery che non sono il suo delivery o la sua posizione corrente
States = ['safe_idle', 'recharging', 'charge_complete']


class TokenPassingRecovery(object):
    def __init__(self, agents, dimesions, obstacles, non_task_endpoints, charging_stations, simulation,
                 a_star_max_iter=4000, new_recovery=False):
        #random.seed(3)
        self.agents = agents
        self.dimensions = dimesions
        self.obstacles = set(obstacles)
        self.non_task_endpoints = non_task_endpoints
        if len(agents) > len(non_task_endpoints):
            print('There are more agents than non task endpoints, instance is not well-formed.')
            exit(1)
        # TODO: Check all properties for well-formedness
        self.token = {}
        self.simulation = simulation
        self.a_star_max_iter = a_star_max_iter
        self.new_recovery = new_recovery
        self.charging_stations = charging_stations
        self.move_consumption = self.simulation.get_move_consumption()
        self.wait_consumption = self.simulation.get_wait_consumption()
        self.init_token()
        # vedi sotto

    def init_token(self):
        self.token['agents'] = {}
        self.token['tasks'] = {}
        self.token['start_tasks_times'] = {}
        self.token['completed_tasks_times'] = {}
        for t in self.simulation.get_new_tasks():
            self.token['tasks'][t['task_name']] = [t['start'], t['goal']]
            self.token['start_tasks_times'][t['task_name']] = self.simulation.get_time()
        self.token['agents_to_tasks'] = {}
        self.token['completed_tasks'] = 0
        self.token['n_replans'] = 0
        self.token['path_ends'] = set()
        self.token['occupied_non_task_endpoints'] = set()
        # 0,1 = coordinates, 2 = t when the agent will finish its recharging process
        self.token['charging_stations'] = {}
        self.token['agent_at_end_path'] = []
        self.token['agent_at_end_path_pos'] = []
        self.token['agents_in_recovery_trial'] = []
        self.token['dead_agents'] = []

        for a in self.agents:
            # dovrebbe essere prendi gli agenti del token e poi nello specifico quello col nome specifico
            self.token['agents'][a['name']] = [a['start']]

            if tuple(a['start']) in self.non_task_endpoints:
                self.token['occupied_non_task_endpoints'].add(tuple(a['start']))
            else:
                self.token['path_ends'].add(tuple(a['start']))

        for c in self.charging_stations:
            # ogni stazione contiene un dizionario con posizione e free_time()
            self.token['charging_stations'][c['name']] = {}
            self.token['charging_stations'][c['name']]['pos'] = c['pos']
            self.token['charging_stations'][c['name']]['free_time'] = 0

    # in teoria agenti in idle hanno il path verso la loro posizione attuale (se sta caricando no)
    def get_idle_agents(self):
        agents = {}
        for name, path in self.token['agents'].items():
            if len(path) == 1 and not (name in self.token['agents_to_tasks'] and self.token['agents_to_tasks'][name][
                'task_name'] == 'recharging'):
                agents[name] = path
        return agents

    # distanza in celle verticali ed orizzontali
    def admissible_heuristic(self, task_pos, agent_pos):
        return fabs(task_pos[0] - agent_pos[0]) + fabs(task_pos[1] - agent_pos[1])

    # In teoria task più vicino al robot, in discarded tasks quelli già considerati e che
    # non rispettano le condizioni. In teoria ritorna sempre qualcosa, se tutti scartati
    # non dovrei nemmeno chiamarlo
    def get_closest_task_name(self, available_tasks, agent_pos, discarded_tasks):

        admissible_tasks = {}
        for a in available_tasks:
            if a not in discarded_tasks:
                admissible_tasks[a] = available_tasks[a]

        closest = random.choice(list(admissible_tasks.keys()))
        # closest = list(admissible_tasks.keys())[0]
        dist = self.admissible_heuristic(admissible_tasks[closest][0], agent_pos)
        for task_name, task in admissible_tasks.items():
            if self.admissible_heuristic(task[0], agent_pos) < dist:
                closest = task_name

        return closest

    def get_moving_obstacles_agents(self, agents, time_start):
        obstacles = {}
        for name, path in agents.items():  # agents.item ritorna un dizionario con nome agente e coordinate dello stesso
            if len(path) > time_start and len(path) > 1:
                for i in range(time_start, len(path)):
                    k = i - time_start
                    obstacles[(path[i][0], path[i][1], k)] = name
                    if i == len(path) - 1:
                        obstacles[(path[i][0], path[i][1], -k)] = name

            # se l'agente si sta caricando
            # considerazione
            elif len(path) == 1 and name in self.token['agents_to_tasks'].keys() and \
                    self.token['agents_to_tasks'][name]['task_name'] == 'recharging':
                station = None
                for s in self.token['charging_stations']:
                    t = self.token['charging_stations'][s]['pos']
                    t = tuple(t)
                    if t == tuple(path[0]):
                        station = s
                        break

                if station is not None and self.token['charging_stations'][station][
                    'free_time'] - self.simulation.get_time() >= time_start:

                    k = time_start
                    while k <= self.token['charging_stations'][station]['free_time'] - self.simulation.get_time():
                        obstacles[(path[0][0], path[0][1], k)] = name
                        k += 1

        return obstacles

    def get_idle_obstacles_agents(self, agents_paths, time_start):
        obstacles = set()
        charging_stations_pos = set()

        for c in self.charging_stations:
            charging_stations_pos.add(tuple(c['pos']))

        for path in agents_paths:
            # quelli nelle stazioni non li segno come ostacoli
            if len(path) == 1 and tuple(path[0]) not in charging_stations_pos:
                obstacles.add((path[0][0], path[0][1]))
            # presumo agenti che finiranno il loro percorso e si fermeranno? Quindi metto ultima
            # loro posizione
            if 1 < len(path) <= time_start:
                obstacles.add((path[-1][0], path[-1][1]))
        return obstacles

    def check_safe_idle(self, agent_pos):
        for task_name, task in self.token['tasks'].items():
            if tuple(task[0]) == tuple(agent_pos) or tuple(task[1]) == tuple(agent_pos):
                return False
        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(agent_pos):
                return False
        # dico che la stazioni di ricarica non sono posti safe
        for recharge_station in self.charging_stations:
            if tuple(recharge_station['pos']) == tuple(agent_pos):
                return False

        return True

    def get_closest_non_task_endpoint(self, agent_pos):
        dist = -1
        res = -1
        for endpoint in self.non_task_endpoints:
            if endpoint not in self.token['occupied_non_task_endpoints']:
                if dist == -1:
                    dist = self.admissible_heuristic(endpoint, agent_pos)
                    res = endpoint
                else:
                    tmp = self.admissible_heuristic(endpoint, agent_pos)
                    if tmp < dist:
                        dist = tmp
                        res = endpoint
        if res == -1:
            print('Error in finding non-task endpoint, is instance well-formed?')
            exit(1)
        return res

    def update_ends(self, agent_pos):
        if tuple(agent_pos) in self.token['path_ends']:
            self.token['path_ends'].remove(tuple(agent_pos))
        elif tuple(agent_pos) in self.token['occupied_non_task_endpoints']:
            self.token['occupied_non_task_endpoints'].remove(tuple(agent_pos))

    def get_agents_to_tasks_goals(self):
        goals = set()
        for el in self.token['agents_to_tasks'].values():
            goals.add(tuple(el['goal']))
        return goals

    def get_agents_to_tasks_starts_goals(self):
        starts_goals = set()
        for el in self.token['agents_to_tasks'].values():
            starts_goals.add(tuple(el['goal']))
            starts_goals.add(tuple(el['start']))
        return starts_goals

    def get_completed_tasks(self):
        return self.token['completed_tasks']

    def get_completed_tasks_times(self):
        return self.token['completed_tasks_times']

    def get_token(self):
        return self.token

    def search(self, cbs, agent_name, moving_obstacles_agents):

        path = cbs.search()
        return path

    def go_to_closest_non_task_endpoint(self, agent_name, agent_pos, all_idle_agents):
        closest_non_task_endpoint = self.get_closest_non_task_endpoint(agent_pos)
        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)

        # Avoid delivery endpoints unless it's the goal or current position
        idle_obstacles_agents |= set(self.non_task_endpoints) - {tuple(agent_pos),
                                                                 closest_non_task_endpoint}
        agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_non_task_endpoint}
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents, moving_obstacles_agents,
                          a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_non_task_endpoint = self.search(cbs, agent_name, moving_obstacles_agents)
        if not path_to_non_task_endpoint:
            print("Solution to non-task endpoint not found for agent", agent_name, " instance is not well-formed.")

        else:
            print('No available task for agent', agent_name, ' moving to safe idling position...')
            self.update_ends(agent_pos)
            self.token['occupied_non_task_endpoints'].add(tuple(closest_non_task_endpoint))
            cost = env.compute_solution_cost(path_to_non_task_endpoint)
            self.token['agents_to_tasks'][agent_name] = {'task_name': 'safe_idle', 'start': agent_pos,
                                                         'goal': closest_non_task_endpoint, 'predicted_cost': cost}
            self.token['agents'][agent_name] = []
            for el in path_to_non_task_endpoint[agent_name]:
                self.token['agents'][agent_name].append([el['x'], el['y']])

    # def get_random_close_cell(self, agent_pos, r):
    #     while True:
    #         cell = (
    #             agent_pos[0] + random.choice(range(-r - 1, r + 1)), agent_pos[1] + random.choice(range(-r - 1, r + 1)))
    #         if cell not in self.obstacles and cell not in self.token['path_ends'] and \
    #                 cell not in self.token['occupied_non_task_endpoints'] \
    #                 and cell not in self.get_agents_to_tasks_goals() \
    #                 and 0 <= cell[0] < self.dimensions[0] and 0 <= cell[1] < self.dimensions[1]:
    #             return cell

    def set_task_name(self, agent_name, task_name):
        self.token['agents_to_tasks'][agent_name]['task_name'] = task_name

    # inoltre rimuove gli agenti dallo stato complete_charge
    def update_completed_tasks(self):
        # Update completed tasks
        for agent_name in self.token['agents']:
            # pos = posizione attuale agente
            pos = self.simulation.actual_paths[agent_name][-1]

            # ---------------------CASO AGENTE ARRIVATO------------------

            # arrivo a stazione di ricarica in questo momento
            if agent_name in agent_name in self.token['agents_to_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.token['agents_to_tasks'][agent_name]['goal']) \
                    and len(self.token['agents'][agent_name]) == 1 and self.token['agents_to_tasks'][agent_name][
                'task_name'] in self.token['charging_stations']:
                station_name = self.token['agents_to_tasks'][agent_name]['task_name']
                self.token['agents_to_tasks'][agent_name]['task_name'] = 'recharging'
                estimated_time_to_recharge = (self.simulation.get_max_autonomies()[agent_name] -
                                              self.simulation.get_batteries_level()[
                                                  agent_name]) / 10
                estimated_time_to_recharge = math.ceil(estimated_time_to_recharge)

                self.token['charging_stations'][station_name][
                    'free_time'] = self.simulation.get_time() + estimated_time_to_recharge

            # se agente assegnato ad un task E le sue coordinate attuali sono = al suo goal
            # E il suo path attuale lungo 1 ed il suo task non è safe idle, recharging, charge_complete
            elif agent_name in self.token['agents_to_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.token['agents_to_tasks'][agent_name]['goal']) \
                    and len(self.token['agents'][agent_name]) == 1 and self.token['agents_to_tasks'][agent_name][
                'task_name'] not in States:
                self.token['completed_tasks'] = self.token['completed_tasks'] + 1
                self.token['completed_tasks_times'][
                    self.token['agents_to_tasks'][agent_name]['task_name']] = self.simulation.get_time()
                self.token['agents_to_tasks'].pop(agent_name)

            # se l'agente è in safe_idle o charge_complete lo rimuovo da agents_to_tasks
            elif agent_name in self.token['agents_to_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.token['agents_to_tasks'][agent_name]['goal']) \
                    and len(self.token['agents'][agent_name]) == 1 and (self.token['agents_to_tasks'][agent_name][
                                                                            'task_name'] == 'safe_idle' or
                                                                        self.token['agents_to_tasks'][agent_name][
                                                                            'task_name'] == 'charge_complete'):
                self.token['agents_to_tasks'].pop(agent_name)
                # capire se l'agente in carica completa deve cambiare altro

    def collect_new_tasks(self):
        for t in self.simulation.get_new_tasks():
            self.token['tasks'][t['task_name']] = [t['start'], t['goal']]
            self.token['start_tasks_times'][t['task_name']] = self.simulation.get_time()

    def find_available_tasks(self, agent_pos):
        available_tasks = {}
        for task_name, task in self.token['tasks'].items():
            # se inizio e fine task non in path ends degli agenti (meno me) AND nemmeno in goals
            if tuple(task[0]) not in self.token['path_ends'].difference({tuple(agent_pos)}) and tuple(
                    task[1]) not in self.token['path_ends'].difference({tuple(agent_pos)}) \
                    and tuple(task[0]) not in self.get_agents_to_tasks_goals() and tuple(
                task[1]) not in self.get_agents_to_tasks_goals():
                available_tasks[task_name] = task
        return available_tasks

    def no_availableTasks_try_recharge(self, agent_name, agent_pos, all_idle_agents, idle_agents):
        print("Nessun task assegnato anche se c'erano")
        # in questo caso parto dalla mia posizione (agent_pos) quindi 0 come costo base
        # quella restituita è sempre raggiungibile data la mia batteria oppure None
        discarded_stations = {}
        theoretically_ok_stations = []
        find_feasible_station = False

        nearest_station, consumption_to_station = self.search_nearest_available_station_to_agent(
            agent_pos, agent_name, discarded_stations)
        while not find_feasible_station and nearest_station is not None:
            # dico questa in teoria va bene
            theoretically_ok_stations.append(nearest_station)
            find_feasible_station = self.compute_real_path_station(agent_name, agent_pos,
                                                                   all_idle_agents, nearest_station)
            # se ho True vuol dire che ho assegnato la stazione e sono felice
            # altrimenti devo vedere altre stazioni
            if not find_feasible_station:
                discarded_stations[nearest_station] = self.token['charging_stations'][nearest_station]
                nearest_station, consumption_to_station = self.search_nearest_available_station_to_agent(
                    agent_pos, agent_name, discarded_stations)

        # se non ho trovato nessuna stazione al momento stampo un messaggio
        # in teoria dovrei provare a fanculizzare gli altri agenti
        if not find_feasible_station:
            print("Nessuna stazione di ricarica raggiungibile con la batteria attuale")
            while len(theoretically_ok_stations) > 0 and not find_feasible_station:
                station = theoretically_ok_stations[0]
                theoretically_ok_stations.remove(station)
                feasible_station, path_to_station_preemption = self.compute_real_path_station_preemption(
                    agent_name, agent_pos, station, all_idle_agents)

                if feasible_station:
                    untouchable_agents, modifiable_agents, going_to_goal_agents = self.find_conflicting_agents(
                        path_to_station_preemption)
                    if len(modifiable_agents) + len(going_to_goal_agents) > 0:

                        for a in modifiable_agents:
                            self.reset_path_and_task(a)
                            idle_agents[a] = self.token['agents'][a]

                        for b in going_to_goal_agents:
                            self.reset_path(b)
                            idle_agents[b] = self.token['agents'][b]

                        # in teoria in moving obstacles ora non ci sono gli agenti problematici
                        find_feasible_station = self.compute_real_path_station(agent_name, agent_pos,
                                                                               all_idle_agents,
                                                                               station)
                        if not find_feasible_station:
                            print(agent_name, " is dead")
                            self.token['dead_agents'].append(agent_name)

                else:
                    print(agent_name, " is dead")
                    self.token['dead_agents'].append(agent_name)

    # restituisce il nome della stazione più vicina a cui è possibile andare a caricarsi una
    # volta completato il task
    def search_nearest_available_station(self, task_cost, task_final_pos, agent_name):

        dist_min = -1
        closest_station_name = None
        estimated_station_cost = None

        for s in self.charging_stations:
            # stazione non assegnata a un altro agente
            if s['name'] not in self.token['agents_to_tasks']:
                estimated_station_cost = self.admissible_heuristic(task_final_pos, s['pos'])
                estimated_arrival_cost = task_cost + estimated_station_cost
                estimated_arrival_time = estimated_arrival_cost + self.simulation.get_time()

                # quando arrivo trovo la stazione libera? Mi basta la batteria per arrivarci?
                if self.token['charging_stations'][s['name']]['free_time'] < estimated_arrival_time \
                        and estimated_arrival_cost * self.move_consumption < self.simulation.get_batteries_level()[
                    agent_name]:
                    if dist_min == -1:
                        dist_min = estimated_arrival_cost
                        closest_station_name = s['name']

                    elif estimated_arrival_cost < dist_min:
                        dist_min = estimated_arrival_cost
                        closest_station_name = s['name']

        return closest_station_name, estimated_station_cost * self.move_consumption

    def search_nearest_available_station_to_agent(self, agent_pos, agent_name, discarded_stations):

        dist_min = -1
        closest_station_name = None
        estimated_station_cost = None
        ongoing_tasks = []

        for a in self.token['agents_to_tasks']:
            ongoing_tasks.append(self.token['agents_to_tasks'][a]['task_name'])

        for s in self.charging_stations:
            # stazione non assegnata a un altro agente
            if s['name'] not in ongoing_tasks and s['name'] not in discarded_stations:
                estimated_station_cost = self.admissible_heuristic(agent_pos, s['pos'])
                estimated_arrival_time = estimated_station_cost + self.simulation.get_time()

                # quando arrivo trovo la stazione libera? Mi basta la batteria per arrivarci?
                if self.token['charging_stations'][s['name']]['free_time'] < estimated_arrival_time \
                        and estimated_station_cost * self.move_consumption < self.simulation.get_batteries_level()[
                    agent_name]:
                    if dist_min == -1:
                        dist_min = estimated_station_cost
                        closest_station_name = s['name']

                    elif estimated_station_cost < dist_min:
                        dist_min = estimated_station_cost
                        closest_station_name = s['name']

        if closest_station_name is None:
            return None, None
        else:
            return closest_station_name, estimated_station_cost * self.move_consumption

    def compute_real_path(self, agent_name, agent_pos, closest_task, closest_task_name, all_idle_agents,
                          available_tasks, consumption_to_station):
        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)
        idle_obstacles_agents |= set(self.non_task_endpoints) - {tuple(agent_pos),
                                                                 tuple(closest_task[1])}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_task[0]}
        # penso sia l'unione di obstacles e idle_obastacles
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_task_start = self.search(cbs, agent_name, moving_obstacles_agents)

        if not path_to_task_start:
            print("Solution not found to task start for agent", agent_name, " idling at current position...")

        else:
            print("Solution found to task start for agent", agent_name, " searching solution to task goal...")
            cost1 = env.compute_solution_cost(path_to_task_start)
            consumption = self.predicted_consumption(path_to_task_start[agent_name])
            # Use cost - 1 because idle cost is 1
            moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], cost1 - 1)
            idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), cost1 - 1)
            idle_obstacles_agents |= set(self.non_task_endpoints) - {tuple(agent_pos),
                                                                     tuple(closest_task[1])}


            agent = {'name': agent_name, 'start': closest_task[0], 'goal': closest_task[1]}
            env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                              moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
            cbs = CBS(env)
            path_to_task_goal = self.search(cbs, agent_name, moving_obstacles_agents)
            if not path_to_task_goal:
                print("Solution not found to task goal for agent", agent_name, " idling at current position...")

            else:
                print("Solution found to task goal for agent", agent_name, " doing task...")
                cost2 = env.compute_solution_cost(path_to_task_goal)
                consumption += self.predicted_consumption(path_to_task_goal[agent_name])

                if self.simulation.get_batteries_level()[agent_name] >= consumption + consumption_to_station:
                    if agent_name not in self.token['agents_to_tasks']:
                        self.token['tasks'].pop(closest_task_name)
                        task = available_tasks.pop(closest_task_name)
                    # ramo else mai in teoria
                    else:
                        task = closest_task

                    last_step = path_to_task_goal[agent_name][-1]
                    self.update_ends(agent_pos)
                    self.token['path_ends'].add(tuple([last_step['x'], last_step['y']]))
                    self.token['agents_to_tasks'][agent_name] = {'task_name': closest_task_name, 'start': task[0],
                                                                 'goal': task[1], 'predicted_cost': cost1 + cost2}
                    self.token['agents'][agent_name] = []
                    for el in path_to_task_start[agent_name]:
                        self.token['agents'][agent_name].append([el['x'], el['y']])
                    # Don't repeat twice same step, elimino ultimo elemento
                    self.token['agents'][agent_name] = self.token['agents'][agent_name][:-1]
                    for el in path_to_task_goal[agent_name]:
                        self.token['agents'][agent_name].append([el['x'], el['y']])

                    # devo fare pop di available_tasks da quello vero?
                    return True,
                else:
                    return False

    def compute_real_path_station(self, agent_name, agent_pos, all_idle_agents, station_name):

        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)
        idle_obstacles_agents |= set(self.non_task_endpoints) - {tuple(agent_pos),
                                                                 tuple(self.token['charging_stations'][station_name]['pos'])}
        # cambiare goal
        agent = {'name': agent_name, 'start': agent_pos, 'goal': self.token['charging_stations'][station_name]['pos']}

        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_station = self.search(cbs, agent_name, moving_obstacles_agents)

        if not path_to_station:
            print("Solution not found to charging station for agent", agent_name, " idling at current position...")

        else:
            print("Solution found to charging station for agent", agent_name)
            cost1 = env.compute_solution_cost(path_to_station)
            consumption = self.predicted_consumption(path_to_station[agent_name])
            # Use cost - 1 because idle cost is 1

            if self.simulation.get_batteries_level()[agent_name] >= consumption:

                last_step = path_to_station[agent_name][-1]
                self.update_ends(agent_pos)
                self.token['path_ends'].add(tuple([last_step['x'], last_step['y']]))
                self.token['agents_to_tasks'][agent_name] = {'task_name': station_name,
                                                             'start': tuple([last_step['x'], last_step['y']]),
                                                             'goal': tuple([last_step['x'], last_step['y']]),
                                                             'predicted_cost': cost1}
                self.token['agents'][agent_name] = []
                for el in path_to_station[agent_name]:
                    self.token['agents'][agent_name].append([el['x'], el['y']])

                return True
            else:
                return False

    # calcolo il percorso escludendo i percorsi degli altri agenti e se sono in idle in posizioni scomode
    def compute_real_path_station_preemption(self, agent_name, agent_pos, station_name, all_idle_agents):

        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)
        idle_obstacles_agents |= set(self.non_task_endpoints) - {tuple(agent_pos),
                                                                 tuple(self.token['charging_stations'][station_name][
                                                                           'pos'])}
        # cambiare goal
        agent = {'name': agent_name, 'start': agent_pos, 'goal': self.token['charging_stations'][station_name]['pos']}
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                          {}, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_station = self.search(cbs, agent_name, {})

        if not path_to_station:
            return False, []

        else:
            consumption = self.predicted_consumption(path_to_station[agent_name])
            if self.simulation.get_batteries_level()[agent_name] >= consumption:
                return True, path_to_station[agent_name]
            else:
                return False, []

    def predicted_consumption(self, path):

        consumption = 0
        for i in range(len(path) - 1):
            if path[i]['x'] == path[i + 1]['x'] and path[i]['y'] == path[i + 1]['y']:
                consumption += self.move_consumption
            else:
                consumption += self.move_consumption

        return round(consumption, 2)

    # restituisco 2 insiemi di agenti con cui ho conflitti: agenti intoccabili (caricano o già pickup)
    # agenti eliminabili (non vanno a caricare)
    def find_conflicting_agents(self, my_path):

        untouchable_agents = []
        modifiable_agents = []
        going_to_goal_agents = []

        for agent in self.token['agents_to_tasks']:
            #k = self.simulation.get_time()
            # ottieni una lista di triple
            path = self.token['agents'][agent]
            for i in range(1, min(len(my_path) - 1, len(path) - 1)):

                # controllo su vertex conflict e transition conflict
                if (my_path[i]['x'] == path[i][0] and my_path[i]['y'] == path[i][1]) or \
                        (my_path[i]['x'] == path[i + 1][0] and my_path[i]['y'] == path[i + 1][1] and
                         my_path[i + 1]['x'] == path[i][0] and my_path[i + 1]['y'] == path[i][1]):

                    if self.token['agents_to_tasks'][agent]['task_name'] in self.token['charging_stations']:
                        untouchable_agents.append(agent)

                    elif self.token['agents_to_tasks'][agent]['task_name'] == 'safe_idle':
                        modifiable_agents.append(agent)

                    else:
                        # il secondo controllo da fare è se hanno già raggiungo lo start_task
                        task_start = self.token['agents_to_tasks'][agent]['start']
                        task_start = tuple(task_start)

                        # se task_start è nel path (lista di tuple) rimanente da percorrere allora non ho fatto pickup
                        if task_start in self.token['agents'][agent]:
                            modifiable_agents.append(agent)
                        else:
                            going_to_goal_agents.append(agent)
                    break

        return untouchable_agents, modifiable_agents, going_to_goal_agents

    def reset_path_and_task(self, agent_name):

        #task: 'task_name', 'start', 'goal'
        task = self.token['agents_to_tasks'][agent_name]
        self.token['agents_to_tasks'].pop(agent_name)
        #riaggiungo il task
        self.token['tasks'][task['task_name']] = [[task['start']],[task['goal']]]
        #dovrei ridurlo al singolo primo elemento
        self.token['agents'][agent_name] = self.token['agents'][agent_name][:1]

    # non rimetto il task tra quelli assegnabili
    def reset_path(self, agent_name):
        self.token['agents'][agent_name] = self.token['agents'][agent_name][:1]

    def compute_new_path_to_goal(self, all_idle_agents, agent_name, agent_pos):

        #forse usare x e y
        task = self.token['agents_to_tasks'][agent_name]

        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents'], 0)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)
        idle_obstacles_agents |= set(self.non_task_endpoints) - {tuple(agent_pos),
                                                                 tuple(task['goal'])}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': task['goal']}

        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_task_goal = self.search(cbs, agent_name, moving_obstacles_agents)

        if not path_to_task_goal:
            print('errore nel ricalcolo del percorso di ', agent_name)
        else:
            consumption = self.predicted_consumption(path_to_task_goal[agent_name])
            if self.simulation.get_batteries_level()[agent_name] >= consumption:

                #path_ends in teoria non lo cambio perché non è mai stato toccato
                #last_step = path_to_task_goal[agent_name][-1]
                #self.update_ends(agent_pos)
                #self.token['path_ends'].add(tuple([last_step['x'], last_step['y']]))
                #self.token['agents_to_tasks'][agent_name] = {'task_name': closest_task_name, 'start': task[0],
                #                                             'goal': task[1], 'predicted_cost': cost1 + cost2}
                self.token['agents'][agent_name] = []
                for el in path_to_task_goal[agent_name]:
                    self.token['agents'][agent_name].append([el['x'], el['y']])

                return True
            else:
                return False

    def time_forward(self):

        self.update_completed_tasks()
        self.collect_new_tasks()
        idle_agents = self.get_idle_agents()
        assigned = False

        while len(idle_agents) > 0:
            agent_name = random.choice(list(idle_agents.keys()))
            # agent_name = list(idle_agents.keys())[0]
            all_idle_agents = self.token['agents'].copy()
            all_idle_agents.pop(agent_name)
            agent_pos = idle_agents.pop(agent_name)[0]
            available_tasks = self.find_available_tasks(agent_pos)

            if agent_name in self.token['agents_to_tasks']:
                if not self.compute_new_path_to_goal(all_idle_agents, agent_name, agent_pos):
                    print("Errore nel calcolo del nuovo percorso verso il goal già assegnato")
                else:
                    print("Percorso dell'", agent_name, " verso il goal ricalcolato correttamente")
                #in teoria end_paths già ok, available_tasks anche perché non lo reinserivo

            elif len(available_tasks) > 0:
                assigned = False
                discarded_tasks = {}
                while not assigned and len(discarded_tasks) < len(available_tasks):
                    # prima c'era un if - else dovuto probabilmente a qualcosa dei delay
                    closest_task_name = self.get_closest_task_name(available_tasks, agent_pos, discarded_tasks)
                    closest_task = available_tasks[closest_task_name]

                    task_total_cost = self.admissible_heuristic(closest_task[0], agent_pos)
                    task_total_cost += self.admissible_heuristic(closest_task[1], closest_task[0])
                    task_total_consumption = task_total_cost * self.move_consumption

                    if task_total_consumption < self.simulation.get_batteries_level()[agent_name]:
                        # consumption to station si intende dal goal alla stazione più vicina
                        nearest_station, consumption_to_station = self.search_nearest_available_station(task_total_cost,closest_task[1],agent_name)

                        if nearest_station is not None:
                            assigned = self.compute_real_path(agent_name, agent_pos, closest_task, closest_task_name,
                                                              all_idle_agents,
                                                              available_tasks, consumption_to_station)
                            # mi turba il fatto che la pop di available_tasks funzioni fuori dalla funzione, ma ok

                            if not assigned:
                                discarded_tasks[closest_task_name] = closest_task
                        else:
                            discarded_tasks[closest_task_name] = closest_task
                    else:
                        discarded_tasks[closest_task_name] = closest_task

                # non posso fare nulla quindi vado a caricarmi
                if not assigned and len(available_tasks) > 0:
                    self.no_availableTasks_try_recharge(agent_name, agent_pos, all_idle_agents, idle_agents)

            # righe 13-14 alg
            elif self.check_safe_idle(agent_pos):
                nearest_station, consumption_to_station = self.search_nearest_available_station_to_agent(
                    agent_pos, agent_name, {})

                if nearest_station is not None and consumption_to_station == self.simulation.get_batteries_level()[agent_name]:
                    find_feasible_station = self.compute_real_path_station(agent_name, agent_pos,
                                                                           all_idle_agents, nearest_station)
                    if find_feasible_station:
                        print(agent_name, ' is moving to ', nearest_station, ' to recharge')
                    else:
                        #feasible_station è un bool
                        feasible_station, path_to_station_preemption = self.compute_real_path_station_preemption(
                            agent_name, agent_pos, nearest_station, all_idle_agents)

                        if feasible_station:
                            untouchable_agents, modifiable_agents, going_to_goal_agents = self.find_conflicting_agents(
                                path_to_station_preemption)
                            if len(modifiable_agents) + len(going_to_goal_agents) > 0:
                                for a in modifiable_agents:
                                    self.reset_path_and_task(a)
                                    idle_agents[a] = self.token['agents'][a]

                                for b in going_to_goal_agents:
                                    self.reset_path(b)
                                    idle_agents[b] = self.token['agents'][b]

                                # in teoria in moving obstacles ora non ci sono gli agenti problematici
                                find_feasible_station = self.compute_real_path_station(agent_name, agent_pos,
                                                                                       all_idle_agents,
                                                                                       nearest_station)
                                if not find_feasible_station:
                                    print(agent_name, " is dead")
                                    self.token['dead_agents'].append(agent_name)
                        else:
                            print(agent_name, " is dead")
                            self.token['dead_agents'].append(agent_name)

                # else:
                #    print(agent_name, ' will wait in idle')

            # righe 15-16 alg
            else:
                self.go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents)
