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


class TokenPassing(object):
    def __init__(self, agents, dimensions, obstacles, non_task_endpoints, charging_stations, simulation, goal_endpoints,
                 a_star_max_iter=4000, new_recovery=False):
        random.seed(3)
        self.agents = agents
        self.dimensions = dimensions
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
        self.goal_endpoints = goal_endpoints
        self.charging_stations = charging_stations
        self.move_consumption = self.simulation.get_move_consumption()
        self.wait_consumption = self.simulation.get_wait_consumption()
        self.chiamateCBS = 0
        self.init_token()
        # vedi sotto

    def init_token(self):
        self.token['agents'] = {}
        self.token['agents_preemption'] = {}
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
        self.token['preemption_ends'] = set()
        self.token['occupied_non_task_endpoints'] = set()
        # 0,1 = coordinates, 2 = t when the agent will finish its recharging process
        self.token['charging_stations'] = {}
        self.token['agent_at_end_path'] = []
        self.token['agent_at_end_path_pos'] = []
        self.token['agents_in_recovery_trial'] = []
        self.token['dead_agents'] = set()

        for a in self.agents:
            self.token['agents'][a['name']] = [a['start']]
            self.token['agents_preemption'][a['name']] = [a['start']]

            if tuple(a['start']) in self.non_task_endpoints:
                self.token['occupied_non_task_endpoints'].add(tuple(a['start']))
            else:
                self.token['path_ends'].add(tuple(a['start']))

        for c in self.charging_stations:
            # ogni stazione contiene un dizionario con posizione e booked
            self.token['charging_stations'][c['name']] = {}
            self.token['charging_stations'][c['name']]['pos'] = c['pos']
            self.token['charging_stations'][c['name']]['booked'] = False

    # in teoria agenti in idle hanno il path verso la loro posizione attuale (se sta caricando no)
    def get_idle_agents(self):
        agents = {}
        for name, path in self.token['agents'].items():
            if len(path) == 1 and not (name in self.token['agents_to_tasks'] and self.token['agents_to_tasks'][name][
                'task_name'] == 'recharging'):  # and name not in self.token['dead_agents']:
                agents[name] = path
        return agents

    def get_chiamateCBS(self):
        return self.chiamateCBS

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

    def get_moving_obstacles_agents(self, agents, time_start, agent_name):
        obstacles = {}
        for name, path in agents.items():  # agents.item ritorna un dizionario con nome agente e coordinate dello stesso

            if name != agent_name:
                if len(path) > time_start and len(path) > 1:
                    for i in range(int(time_start), len(path)):
                        k = i - time_start

                        if (path[i][0], path[i][1], k) in obstacles:
                            print('problema già inserito')

                        obstacles[(path[i][0], path[i][1], k)] = name
                        if i == len(path) - 1:
                            obstacles[(path[i][0], path[i][1], -k)] = name

        return obstacles

    def get_idle_obstacles_agents(self, agents_paths, time_start, agent_name):
        obstacles = set()

        for g in self.goal_endpoints:
            obstacles.add(tuple(g))
        for c in self.charging_stations:
            obstacles.add(tuple(c['pos']))

        for agent in agents_paths:

            if agent != agent_name:
                # quelli nelle stazioni non li segno come ostacoli and tuple(path[0]) not in charging_stations_pos
                if len(agents_paths[agent]) == 1:
                    obstacles.add((agents_paths[agent][0][0], agents_paths[agent][0][1]))
                # presumo agenti che finiranno il loro percorso e si fermeranno? Quindi metto ultima
                # loro posizione
                if 1 < len(agents_paths[agent]) <= time_start:
                    obstacles.add((agents_paths[agent][-1][0], agents_paths[agent][-1][1]))
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

    # agent_pos è da che posizione calcolare l'endpoint più vicino, old_non_task_endpoint è l'endpoint che (forse) occupi ora
    # preem_non_task_endpoint è l'endpoint che hai prenotato al momento
    def get_closest_non_task_endpoint(self, agent_pos, discarded_endpoints, old_non_task_endpoint,
                                      preem_non_task_endpoint):
        dist = -1
        res = -1
        for endpoint in self.non_task_endpoints:
            if ((endpoint not in self.token[
                'occupied_non_task_endpoints'] and endpoint not in discarded_endpoints and endpoint not in self.token[
                     'preemption_ends'])
                    or endpoint == tuple(agent_pos) or endpoint == tuple(old_non_task_endpoint) or endpoint == tuple(
                        preem_non_task_endpoint)):
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
            # exit(1)
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
        self.chiamateCBS += 1
        return path

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
                # self.token['agents_to_tasks'][agent_name]['task_name'] = 'recharging'
                estimated_time_to_recharge = (self.simulation.get_max_autonomies()[agent_name] -
                                              self.simulation.get_batteries_level()[
                                                  agent_name]) / 10
                estimated_time_to_recharge = math.ceil(estimated_time_to_recharge)

                # aggiungo al path dell'agente la posizione corrente fino a quando non finirà di caricarsi
                for i in range(estimated_time_to_recharge):
                    self.token['agents'][agent_name].append([pos['x'], pos['y']])

                self.token['agents_to_tasks'][agent_name]['task_name'] = 'recharging'

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

    # def change_other_paths(self, path_to_station_preemption, idle_agents, agent_name, agent_pos,
    #                        all_idle_agents, nearest_station):
    #
    #     untouchable_agents, modifiable_agents, going_to_goal_agents = self.find_conflicting_agents(
    #         path_to_station_preemption)
    #     find_feasible_station = False
    #
    #     if len(modifiable_agents) + len(going_to_goal_agents) > 0:
    #         for a in modifiable_agents:
    #             self.reset_path_and_task(a)
    #             idle_agents[a] = self.token['agents'][a]
    #
    #         for b in going_to_goal_agents:
    #             self.reset_path(b)
    #             idle_agents[b] = self.token['agents'][b]
    #
    #         # in teoria in moving obstacles ora non ci sono gli agenti problematici
    #         find_feasible_station = self.compute_real_path_station(agent_name, agent_pos,
    #                                                                all_idle_agents,
    #                                                                nearest_station, 0)
    #     if not find_feasible_station:
    #         self.check_if_dead(agent_pos, agent_name)
    #
    #     return find_feasible_station

    # devo andarmi a caricare
    # def try_preemption_recharge(self, agent_name, agent_pos, all_idle_agents, idle_agents):
    #
    #     # in questo caso parto dalla mia posizione (agent_pos) quindi 0 come costo base
    #     # quella restituita è sempre raggiungibile data la mia batteria oppure None
    #     discarded_stations = {}
    #     theoretically_ok_stations = []
    #     path_station = False
    #
    #     nearest_station, consumption_to_station = self.search_nearest_available_station_to_agent(
    #         agent_pos, agent_name, discarded_stations)
    #     while not path_station and nearest_station is not None:
    #
    #         # theoretically_ok_stations.append(nearest_station)
    #         path_station = self.compute_real_path_station(agent_name, agent_pos,
    #                                                       all_idle_agents, nearest_station, 0)
    #         # se ho True vuol dire che ho assegnato la stazione e sono felice
    #         # altrimenti devo vedere altre stazioni
    #         if not path_station:
    #             discarded_stations[nearest_station] = self.token['charging_stations'][nearest_station]
    #
    #             nearest_station, consumption_to_station = self.search_nearest_available_station_to_agent(
    #                 agent_pos, agent_name, discarded_stations)
    #
    #     # path station contiene il percorso reale per andare alla stazione oppure falso
    #     if not path_station:
    #         # qui ci andava la parte sul cambio di path agli altri agenti
    #         self.check_if_dead(agent_pos, agent_name)
    #     else:
    #         # gli metto -1,-1 tanto viene cancellato
    #         self.apply_path_preemption(agent_name, None, [{'t': -1, 'x': -1, 'y': -1}], path_station[agent_name], None,
    #                                    nearest_station)
    #         if self.predicted_consumption(path_station[agent_name]) == self.simulation.get_batteries_level()[
    #             agent_name]:
    #             self.use_preempted_path_to_station(agent_name)

    # restituisce il nome della stazione più vicina a cui è possibile andare a caricarsi una
    # volta completato il task
    # discarded_stations è un set di nomi di stazioni scartate
    def search_nearest_available_station(self, task_duration, task_final_pos, agent_name, discarded_stations):

        dist_min = -1
        closest_station_name = None
        estimated_to_station_duration = 0

        for s in self.charging_stations:
            # stazione non assegnata a un altro agente
            if s['name'] not in self.token['agents_to_tasks'] and s['name'] not in discarded_stations:
                estimated_to_station_duration = self.admissible_heuristic(task_final_pos, s['pos'])
                estimated_total_duration = task_duration + estimated_to_station_duration
                # estimated_arrival_time = estimated_total_duration + self.simulation.get_time()

                # quando arrivo trovo la stazione libera? Mi basta la batteria per arrivarci?
                # self.token['charging_stations'][s['name']]['free_time'] < estimated_arrival_time
                if ((self.token['charging_stations'][s['name']]['booked'] == False or
                     self.token['charging_stations'][s['name']]['booked'] == agent_name) and
                        estimated_total_duration * self.move_consumption < self.simulation.get_batteries_level()[
                            agent_name]):
                    if dist_min == -1:
                        dist_min = estimated_total_duration
                        closest_station_name = s['name']

                    elif estimated_total_duration < dist_min:
                        dist_min = estimated_total_duration
                        closest_station_name = s['name']

        return closest_station_name, estimated_to_station_duration * self.move_consumption

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

                if ((self.token['charging_stations'][s['name']]['booked'] is False or
                     self.token['charging_stations'][s['name']]['booked'] == agent_name) and
                        estimated_station_cost * self.move_consumption < self.simulation.get_batteries_level()[
                            agent_name]):

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

    def check_if_dead(self, agent_pos, agent_name):
        dist_min = -1
        estimated_station_cost = None

        for s in self.charging_stations:
            estimated_station_cost = self.admissible_heuristic(agent_pos, s['pos'])
            if dist_min == -1:
                dist_min = estimated_station_cost

            elif estimated_station_cost < dist_min:
                dist_min = estimated_station_cost

        if dist_min * self.move_consumption > self.simulation.get_batteries_level()[
            agent_name]:
            self.token['dead_agents'].add(agent_name)
            print(agent_name, 'is dead in position ', agent_pos, 'al timestep', self.simulation.get_time())
            return True
        else:
            return False

    # se ho solo un percorso utilizzo il due
    def apply_path(self, agent_name, agent_pos, path1, path2, task_name, start, goal, predicted_cost):
        last_step = path2[-1]
        self.update_ends(agent_pos)

        self.token['agents_to_tasks'][agent_name] = {'task_name': task_name, 'start': start,
                                                     'goal': goal, 'predicted_cost': predicted_cost}
        self.token['agents'][agent_name] = []

        # se path1 non è None, allora stiamo assegnando il percorso di un task e quindi metto in path_ends
        if path1 is not None:
            self.token['path_ends'].add(tuple([last_step['x'], last_step['y']]))
            for el in path1:
                self.token['agents'][agent_name].append([el['x'], el['y']])
            # Don't repeat twice same step, elimino ultimo elemento
            self.token['agents'][agent_name] = self.token['agents'][agent_name][:-1]

        for el in path2:
            self.token['agents'][agent_name].append([el['x'], el['y']])

    def apply_path_preemption(self, agent_name, path1, path2, path_preemption, estimated_time_to_recharge,
                              path_endpoint, station_name):
        # prima di resettare rimuovo l'ultimo elemento del path precedente dai finali
        self.token['preemption_ends'].discard(tuple(self.token['agents_preemption'][agent_name][-1]))
        for s in self.token['charging_stations']:
            if self.token['charging_stations'][s]['booked'] == agent_name:
                self.token['charging_stations'][s]['booked'] = False
                break

        self.token['agents_preemption'][agent_name] = []
        if path1 is not None:
            for el in path1:
                self.token['agents_preemption'][agent_name].append([el['x'], el['y']])
            # Don't repeat twice same step, elimino ultimo elemento
            self.token['agents_preemption'][agent_name] = self.token['agents_preemption'][agent_name][:-1]

        for el in path2:
            self.token['agents_preemption'][agent_name].append([el['x'], el['y']])

        self.token['agents_preemption'][agent_name] = self.token['agents_preemption'][agent_name][:-1]
        for el in path_preemption:
            self.token['agents_preemption'][agent_name].append([el['x'], el['y']])

        # tempo fermo a caricare
        el = path_preemption[-1]
        for i in range(estimated_time_to_recharge):
            self.token['agents_preemption'][agent_name].append([el['x'], el['y']])

        if path_endpoint is not None:
            self.token['agents_preemption'][agent_name] = self.token['agents_preemption'][agent_name][:-1]
            for el in path_endpoint:
                self.token['agents_preemption'][agent_name].append([el['x'], el['y']])

        self.token['charging_stations'][station_name]['booked'] = agent_name

        self.token['preemption_ends'].add(tuple(self.token['agents_preemption'][agent_name][-1]))

    def use_preempted_path_to_station(self, agent_name):

        station_name = 'Error'
        station_pos = None
        for s in self.charging_stations:
            if s['pos'] in self.token['agents_preemption'][agent_name]:
                station_name = s['name']
                station_pos = s['pos']
                break

        # copio solo la parte relativa alla stazione, non quella del NON task endpoint
        self.token['agents'][agent_name] = []
        i = 0
        while self.token['agents_preemption'][agent_name][i] != station_pos:
            self.token['agents'][agent_name].append(self.token['agents_preemption'][agent_name][i])
            i += 1
        self.token['agents'][agent_name].append(self.token['agents_preemption'][agent_name][i])

        last_step = self.token['agents'][agent_name][-1]
        self.update_ends(self.token['agents'][agent_name][0])
        self.token['path_ends'].add(tuple([last_step[0], last_step[1]]))

        self.token['agents_to_tasks'][agent_name] = {'task_name': station_name,
                                                     'start': tuple([last_step[0], last_step[1]]),
                                                     'goal': tuple([last_step[0], last_step[1]]),
                                                     'predicted_cost': len(self.token['agents'][agent_name]) - 1}

        self.token['charging_stations'][station_name]['booked'] = False

    def compute_real_path(self, agent_name, agent_pos, closest_task, closest_task_name, all_idle_agents,
                          available_tasks, predicted_consumption_to_station):
        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents_preemption'], 0, agent_name)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, 0, agent_name)
        idle_obstacles_agents |= set(self.non_task_endpoints)
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos), tuple(closest_task[1])}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_task[0]}
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_task_start = self.search(cbs, agent_name, moving_obstacles_agents)

        if not path_to_task_start:
            print("Solution not found to task start for agent", agent_name, " idling at current position...")
            return False, False, False, False
        else:
            print("Solution found to task start for agent", agent_name, " searching solution to task goal...")
            cost1 = env.compute_solution_cost(path_to_task_start)
            consumption = self.predicted_consumption(path_to_task_start[agent_name])
            # Use cost - 1 because idle cost is 1 (precursore) - secondo me perchè il primo step lo fai a tempo 0
            moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents_preemption'], cost1 - 1,
                                                                       agent_name)
            idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, cost1 - 1, agent_name)
            idle_obstacles_agents |= set(self.non_task_endpoints)
            idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos), tuple(closest_task[1])}

            agent = {'name': agent_name, 'start': closest_task[0], 'goal': closest_task[1]}
            env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                              moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
            cbs = CBS(env)
            path_to_task_goal = self.search(cbs, agent_name, moving_obstacles_agents)
            if not path_to_task_goal:
                print("Solution not found to task goal for agent", agent_name, " idling at current position...")
                return False, False, False, False
            else:
                print("Solution found to task goal for agent", agent_name, " doing task...")
                cost2 = env.compute_solution_cost(path_to_task_goal)
                consumption += self.predicted_consumption(path_to_task_goal[agent_name])

                if self.simulation.get_batteries_level()[agent_name] >= consumption + predicted_consumption_to_station:
                    return path_to_task_start, path_to_task_goal, cost1 + cost2 - 1, consumption
                else:
                    return False, False, False, False

    # agent_pos spesso è la posizione del goal appena terminato dall'agente
    # time_start è il tempo in cui l'agente termina il goal e poi va a caricarsi nel caso
    def compute_real_path_station(self, agent_name, agent_pos, all_idle_agents, station_name, time_start,
                                  task_total_consumption, battery):

        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents_preemption'], time_start,
                                                                   agent_name)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start, agent_name)
        idle_obstacles_agents |= set(self.non_task_endpoints)
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos),
                                                         tuple(self.token['charging_stations'][station_name]['pos'])}
        # cambiare goal
        agent = {'name': agent_name, 'start': agent_pos, 'goal': self.token['charging_stations'][station_name]['pos']}
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_station = self.search(cbs, agent_name, moving_obstacles_agents)

        if not path_to_station:
            print("Solution not found to charging station for agent", agent_name, " idling at current position...")
            return False, False, False

        else:
            # print("Solution found to charging station for agent", agent_name)
            cost1 = env.compute_solution_cost(path_to_station)
            consumption = self.predicted_consumption(path_to_station[agent_name]) + task_total_consumption
            # Use cost - 1 because idle cost is 1

            if battery >= consumption:
                return path_to_station, consumption, cost1
            else:
                return False, False, False

    # calcolo il percorso escludendo i percorsi degli altri agenti e se sono in idle in posizioni scomode
    def compute_real_path_station_preemption(self, agent_name, agent_pos, station_name, all_idle_agents):

        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, 0, agent_name)
        idle_obstacles_agents |= set(
            self.non_task_endpoints)
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos),
                                                         tuple(self.token['charging_stations'][station_name]['pos'])}

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

    def compute_new_path_to_goal(self, all_idle_agents, agent_name, agent_pos):

        task = self.token['agents_to_tasks'][agent_name]
        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents_preemption'], 0, agent_name)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, 0, agent_name)
        idle_obstacles_agents |= set(self.non_task_endpoints)  # - {tuple(agent_pos), tuple(task['goal'])}
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos), tuple(task['goal'])}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': task['goal']}
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_task_goal = self.search(cbs, agent_name, moving_obstacles_agents)

        if not path_to_task_goal:
            print('errore(agent path tolto) nel ricalcolo del percorso di ', agent_name)
        else:
            consumption = self.predicted_consumption(path_to_task_goal[agent_name])
            if self.simulation.get_batteries_level()[agent_name] >= consumption:

                self.token['agents'][agent_name] = []
                for el in path_to_task_goal[agent_name]:
                    self.token['agents'][agent_name].append([el['x'], el['y']])
                return True
            else:
                return False

    def compute_real_path_to_endpoint(self, all_idle_agents, agent_name, agent_pos, closest_endpoint,
                                      previous_consumption, time_start, autonomy):

        moving_obstacles_agents = self.get_moving_obstacles_agents(self.token['agents_preemption'], time_start,
                                                                   agent_name)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start, agent_name)

        # Avoid delivery endpoints unless it's the goal or current position
        idle_obstacles_agents |= set(self.non_task_endpoints)
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos), closest_endpoint}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_endpoint}
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents, moving_obstacles_agents,
                          a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_non_task_endpoint = self.search(cbs, agent_name, moving_obstacles_agents)
        if not path_to_non_task_endpoint:
            # print("Solution to non-task endpoint not found for ", agent_name, " instance is not well-formed.")
            return False, False
        else:
            consumption = self.predicted_consumption(path_to_non_task_endpoint[agent_name]) + previous_consumption
            if autonomy >= consumption:
                return path_to_non_task_endpoint, env.compute_solution_cost(path_to_non_task_endpoint)
            else:
                return False, False

    def predicted_consumption(self, path):

        consumption = 0

        if type(path[0]) is dict:
            for i in range(len(path) - 1):
                if path[i]['x'] == path[i + 1]['x'] and path[i]['y'] == path[i + 1]['y']:
                    consumption += self.move_consumption
                else:
                    consumption += self.move_consumption
        else:
            for i in range(len(path) - 1):
                if path[i][0] == path[i + 1][0] and path[i][1] == path[i + 1][1]:
                    consumption += self.move_consumption
                else:
                    consumption += self.move_consumption

        return round(consumption, 2)

    # restituisco 2 insiemi di agenti con cui ho conflitti: agenti intoccabili (caricano o già pickup)
    # agenti eliminabili (non vanno a caricare)
    # def find_conflicting_agents(self, my_path):
    #
    #     untouchable_agents = []
    #     modifiable_agents = []
    #     going_to_goal_agents = []
    #
    #     for agent in self.token['agents_to_tasks']:
    #         # k = self.simulation.get_time()
    #         # ottieni una lista di triple
    #         path = self.token['agents'][agent]
    #         for i in range(1, min(len(my_path) - 1, len(path) - 1)):
    #
    #             # controllo su vertex conflict e transition conflict
    #             if (my_path[i]['x'] == path[i][0] and my_path[i]['y'] == path[i][1]) or \
    #                     (my_path[i]['x'] == path[i + 1][0] and my_path[i]['y'] == path[i + 1][1] and
    #                      my_path[i + 1]['x'] == path[i][0] and my_path[i + 1]['y'] == path[i][1]):
    #
    #                 if self.token['agents_to_tasks'][agent]['task_name'] in self.token['charging_stations']:
    #                     untouchable_agents.append(agent)
    #
    #                 elif self.token['agents_to_tasks'][agent]['task_name'] == 'safe_idle':
    #                     modifiable_agents.append(agent)
    #
    #                 else:
    #                     # il secondo controllo da fare è se hanno già raggiungo lo start_task
    #                     task_start = self.token['agents_to_tasks'][agent]['start']
    #                     task_start = tuple(task_start)
    #
    #                     # se task_start è nel path (lista di tuple) rimanente da percorrere allora non ho fatto pickup
    #                     if task_start in self.token['agents'][agent]:
    #                         modifiable_agents.append(agent)
    #                     else:
    #                         going_to_goal_agents.append(agent)
    #                 break
    #
    #     return untouchable_agents, modifiable_agents, going_to_goal_agents
    def reset_path_and_task(self, agent_name):

        # task: 'task_name', 'start', 'goal'
        task = self.token['agents_to_tasks'][agent_name]
        self.token['agents_to_tasks'].pop(agent_name)
        # riaggiungo il task
        self.token['tasks'][task['task_name']] = [[task['start']], [task['goal']]]
        # dovrei ridurlo al singolo primo elemento
        self.token['agents'][agent_name] = self.token['agents'][agent_name][:1]

    # non rimetto il task tra quelli assegnabili
    def reset_path(self, agent_name):
        self.token['agents'][agent_name] = self.token['agents'][agent_name][:1]

    # se hai un percorso di ricarica, ma non hai la batteria uguale al minimo per andarci controlli
    # se anche al turno dopo avrai un percorso valido per andare. Se si cambi il preemption path con quello
    # altrimenti vai a caricarti con il path già deciso
    def find_new_path_to_station_plus_endpoint(self, agent_name, agent_pos, all_idle_agents, first_time):

        discarded_stations = {}
        path_station = False
        assigned = False

        nearest_station, consumption_to_station = self.search_nearest_available_station_to_agent(
            agent_pos, agent_name, discarded_stations)

        while not path_station and nearest_station is not None and not assigned:
            # time start = 1 perché devo considerare dal turno dopo
            battery = self.simulation.get_batteries_level()[agent_name]
            if first_time:
                time_start = 0
                previous_consumption = 0

            else:
                time_start = 1
                previous_consumption = self.simulation.get_wait_consumption()

            # gli passo il consumo di un turno di wait se non è il primo turno
            path_station, to_station_consumption, to_station_duration = self.compute_real_path_station(agent_name,
                                                                                                       agent_pos,
                                                                                                       all_idle_agents,
                                                                                                       nearest_station,
                                                                                                       time_start,
                                                                                                       previous_consumption,
                                                                                                       battery)

            # se ho True vuol dire che ho assegnato la stazione e sono felice
            # altrimenti devo vedere altre stazioni
            if not path_station:
                discarded_stations[nearest_station] = self.token['charging_stations'][nearest_station]

                nearest_station, consumption_to_station = self.search_nearest_available_station_to_agent(
                    agent_pos, agent_name, discarded_stations)
            # controllo del NON-task endpoint
            else:
                discarded_endpoints = set()
                assigned = False
                total_real_consumption = self.predicted_consumption(path_station[agent_name])
                estimated_time_to_recharge = (self.simulation.get_max_autonomies()[agent_name] -
                                              self.simulation.get_batteries_level()[
                                                  agent_name] + total_real_consumption) / 10
                estimated_time_to_recharge = math.ceil(estimated_time_to_recharge)

                # tra quanti timestep dovrei caricarmi
                total_real_duration = to_station_duration + estimated_time_to_recharge
                closest_non_task_endpoint = 0
                station_pos = tuple((path_station[agent_name][-1]['x'], path_station[agent_name][-1]['y']))

                while not assigned and closest_non_task_endpoint != -1:

                    closest_non_task_endpoint = self.get_closest_non_task_endpoint(station_pos,
                                                                                   discarded_endpoints, agent_pos,
                                                                                   self.token['agents_preemption'][
                                                                                       agent_name][-1])
                    if closest_non_task_endpoint != -1:
                        endpoint_duration = self.admissible_heuristic(closest_non_task_endpoint,
                                                                      station_pos)
                        endpoint_consume_heuristic = endpoint_duration * self.move_consumption
                        if endpoint_consume_heuristic < self.simulation.get_max_autonomies()[
                            agent_name]:

                            # real_cost è in step, non in consumo
                            # previous consumption è 0 perché mi sono appena caricato
                            # total_real_duration è il tempo totale che ci metto a fare tutto (c'è un meno 1 per la concatenazione)
                            path_endpoint, real_duration = self.compute_real_path_to_endpoint(
                                all_idle_agents, agent_name, station_pos,
                                closest_non_task_endpoint, 0, total_real_duration - 1 + time_start,
                                self.simulation.get_max_autonomies()[
                                    agent_name])

                            if path_endpoint is not False:

                                # nel calcolo del percorso reale verso la stazione ho già considerato il consumo in wait
                                # di conseguenza assegno come preemption path questo al posto del precedente
                                assigned = True
                                self.apply_path_preemption(agent_name, None,
                                                           [path_station[agent_name][0], {'t': -1, 'x': -1, 'y': -1}],
                                                           path_station[agent_name], estimated_time_to_recharge,
                                                           path_endpoint[agent_name], nearest_station)

                            else:
                                discarded_endpoints.add(closest_non_task_endpoint)
                        else:
                            # se già il più vicino non va bene col calcolo euristico, non ha senso andare avanti
                            # discarded_endpoints.add(closest_non_task_endpoint)
                            break

                if not assigned:
                    discarded_stations[nearest_station] = nearest_station
                    nearest_station, consumption_to_station = self.search_nearest_available_station_to_agent(
                        agent_pos, agent_name, discarded_stations)

        return assigned

    def decide_if_charge(self, agent_name, agent_pos, all_idle_agents, idle_agents):
        # controllo se ho un path per andarmi a caricare
        if agent_name in self.token['agents_preemption'] and len(self.token['agents'][agent_name]) != len(
                self.token['agents_preemption'][agent_name]):
            # se con la mia batteria arrivo giusto a caricarmi ci vado
            # if self.compute_consumption_to_station(agent_name) == self.simulation.get_batteries_level()[agent_name]:
            if self.last_time_to_charge(agent_name):
                self.use_preempted_path_to_station(agent_name)

            # altrimenti calcolo il percorso per andare a caricarmi a turno dopo e se esiste assegno quello al preem
            # se non esiste assegno quello già calcolato
            else:
                changed = self.find_new_path_to_station_plus_endpoint(agent_name, agent_pos, all_idle_agents, False)
                if not changed:
                    self.use_preempted_path_to_station(agent_name)

        else:
            # self.try_preemption_recharge(agent_name, agent_pos, all_idle_agents, idle_agents)
            changed = self.find_new_path_to_station_plus_endpoint(agent_name, agent_pos, all_idle_agents, True)
            if not changed:
                print("Errore ", agent_name, " non aveva un percorso per andare a caricarsi e non lo trova ora")

    # qui io non sono già in un NONTE
    def choose_NON_task_endpoint_and_station(self, agent_name, agent_pos, all_idle_agents):

        assigned = False
        discarded_endpoints1 = set()

        for s in self.token['charging_stations']:
            if self.token['charging_stations'][s]['booked'] == agent_name:
                self.token['charging_stations'][s]['booked'] = False
                break
        closest_non_task_endpoint = 1
        # fino a quando non assegno un task oppure li ho esclusi tutti
        while not assigned and closest_non_task_endpoint != -1:

            closest_non_task_endpoint = self.get_closest_non_task_endpoint(agent_pos, discarded_endpoints1, agent_pos,
                                                                           self.token['agents_preemption'][agent_name][
                                                                               -1])

            if closest_non_task_endpoint != -1:
                endpoint_duration = self.admissible_heuristic(closest_non_task_endpoint, agent_pos)

                endpoint_consume_heuristic = endpoint_duration * self.move_consumption
                if endpoint_consume_heuristic < self.simulation.get_batteries_level()[agent_name]:

                    nearest_station, consumption_to_station_heuristic = self.search_nearest_available_station(
                        endpoint_duration, closest_non_task_endpoint, agent_name, set())

                    if nearest_station is not None:

                        # real_cost è in step, non in consumo
                        # previous consumption in questo caso lo faccio valere come il costo per andare a caricarmi
                        path_endpoint, real_duration_endpoint = self.compute_real_path_to_endpoint(all_idle_agents,
                                                                                                   agent_name,
                                                                                                   agent_pos,
                                                                                                   closest_non_task_endpoint,
                                                                                                   consumption_to_station_heuristic,
                                                                                                   0,
                                                                                                   self.simulation.get_batteries_level()[
                                                                                                       agent_name])

                        endpoint_consumption = self.predicted_consumption(path_endpoint[agent_name])
                        # se il path reale con consumo euristico verso la stazione non esiste, non ha senso andare avanti
                        if path_endpoint is not False:
                            discarded_stations = set()
                            while nearest_station is not None and assigned is False:

                                # ora controllo pure il percorso reale della stazione
                                path_station, to_station_consumption, to_station_duration = self.compute_real_path_station(
                                    agent_name, closest_non_task_endpoint, all_idle_agents, nearest_station,
                                    real_duration_endpoint - 1, endpoint_consumption,
                                    self.simulation.get_batteries_level()[agent_name])

                                # se effettivamente ho un percorso per la stazione calcolato col consumo reale, allora
                                # posso controllare gli endpoint
                                if path_station is not False:
                                    discarded_endpoints2 = set()
                                    # assigned = False
                                    total_real_consumption = endpoint_consumption + to_station_consumption
                                    estimated_time_to_recharge = (self.simulation.get_max_autonomies()[agent_name] -
                                                                  self.simulation.get_batteries_level()[
                                                                      agent_name] + total_real_consumption) / 10

                                    estimated_time_to_recharge = math.ceil(estimated_time_to_recharge)
                                    # tra quanti timestep dovrei caricarmi
                                    total_real_duration = real_duration_endpoint + estimated_time_to_recharge + to_station_duration - 1
                                    closest_non_task_endpoint2 = 0

                                    while not assigned and closest_non_task_endpoint2 != -1:
                                        closest_non_task_endpoint2 = self.get_closest_non_task_endpoint(tuple(
                                            (path_station[agent_name][-1]['x'], path_station[agent_name][-1]['y'])),
                                            discarded_endpoints2,
                                            tuple((
                                                path_station[
                                                    agent_name][
                                                    -1][
                                                    'x'],
                                                path_station[
                                                    agent_name][
                                                    -1][
                                                    'y'])),
                                            self.token[
                                                'agents_preemption'][
                                                agent_name][
                                                -1])
                                        if closest_non_task_endpoint2 != -1:
                                            endpoint_duration2 = self.admissible_heuristic(closest_non_task_endpoint2,
                                                                                           tuple((path_station[
                                                                                                      agent_name][-1][
                                                                                                      'x'],
                                                                                                  path_station[
                                                                                                      agent_name][-1][
                                                                                                      'y'])))
                                            endpoint_consume_heuristic2 = endpoint_duration2 * self.move_consumption
                                            if endpoint_consume_heuristic2 < self.simulation.get_max_autonomies()[
                                                agent_name]:

                                                # real_cost è in step, non in consumo
                                                # previous consumption è 0 perché mi sono appena caricato
                                                path_endpoint2, real_cost2 = self.compute_real_path_to_endpoint(
                                                    all_idle_agents, agent_name, tuple((path_station[agent_name][-1][
                                                                                            'x'],
                                                                                        path_station[agent_name][-1][
                                                                                            'y'])),
                                                    closest_non_task_endpoint2, 0, total_real_duration - 1,
                                                    self.simulation.get_max_autonomies()[
                                                        agent_name])

                                                if path_endpoint2 is not False:
                                                    assigned = True
                                                    # ufficializzo il path fino al primo non task endpoint

                                                    self.apply_path(agent_name, agent_pos, None,
                                                                    path_endpoint[agent_name], 'safe_idle',
                                                                    agent_pos, closest_non_task_endpoint,
                                                                    endpoint_duration)
                                                    self.token['occupied_non_task_endpoints'].add(
                                                        closest_non_task_endpoint)
                                                    self.apply_path_preemption(agent_name, None,
                                                                               path_endpoint[agent_name],
                                                                               path_station[agent_name],
                                                                               estimated_time_to_recharge,
                                                                               path_endpoint2[agent_name],
                                                                               nearest_station)

                                                    print(agent_name, " si dirige verso il NON task endpoint",
                                                          closest_non_task_endpoint)
                                                else:
                                                    discarded_endpoints2.add(closest_non_task_endpoint2)
                                            else:
                                                # se già il più vicino non va bene col calcolo euristico, non ha senso andare avanti
                                                # discarded_endpoints.add(closest_non_task_endpoint)
                                                break

                                    if not assigned:
                                        discarded_stations.add(nearest_station)
                                else:
                                    discarded_stations.add(nearest_station)

                                nearest_station, consumption_to_station_heuristic = self.search_nearest_available_station(
                                    endpoint_duration, closest_non_task_endpoint, agent_name, set())

                            # se alla fine del ciclo delle stazioni non ho assegnato il task allora lo metto tra gli scartati
                            if not assigned:
                                discarded_endpoints1.add(closest_non_task_endpoint)
                        else:
                            discarded_endpoints1.add(closest_non_task_endpoint)
                        # qui non metto break perché magari ci sono task con distanza euristica minore, ma che hanno
                        # percorso reale meno esoso in termini di risorse

                    else:
                        break

                else:
                    break

        return assigned

    def choose_task_and_station(self, agent_name, agent_pos, all_idle_agents, available_tasks):

        assigned = False
        discarded_tasks = {}
        for s in self.token['charging_stations']:
            if self.token['charging_stations'][s]['booked'] == agent_name:
                self.token['charging_stations'][s]['booked'] = False
                break

        # fino a quando non assegno un task oppure li ho esclusi tutti
        while not assigned and len(discarded_tasks) < len(available_tasks):

            closest_task_name = self.get_closest_task_name(available_tasks, agent_pos, discarded_tasks)
            closest_task = available_tasks[closest_task_name]

            task_total_duration = self.admissible_heuristic(closest_task[0], agent_pos)
            task_total_duration += self.admissible_heuristic(closest_task[1], closest_task[0])
            task_total_consumption_heuristic = task_total_duration * self.move_consumption

            # in questo controllo considero il task più vicino ed il suo consumo con euristica (minimo)
            # se già questo non va bene, non ha senso andare avanti
            if task_total_consumption_heuristic < self.simulation.get_batteries_level()[agent_name]:
                # consumption to station si intende dal goal alla stazione più vicina
                nearest_station, consumption_to_station_heuristic = self.search_nearest_available_station(
                    task_total_duration - 1, closest_task[1], agent_name, set())

                # se già con i consumi euristici non arrivo alla stazione più vicina, non ha senso andare avanti
                if nearest_station is not None:
                    # cost non è il consumo, ma il numero di timestep
                    # c'è un controllo interno sul consumo totale inclusi start, goal e stazione (euristica)
                    path_start, path_goal, total_real_duration, task_total_consumption = self.compute_real_path(
                        agent_name, agent_pos, closest_task, closest_task_name,
                        all_idle_agents, available_tasks, consumption_to_station_heuristic)

                    # se il path reale con consumo euristico verso la stazione non esiste, non ha senso andare avanti
                    if path_start is not False:
                        discarded_stations = set()
                        while nearest_station is not None and assigned is False:

                            # ora controllo pure il percorso reale della stazione
                            path_station, to_station_consumption, to_station_duration = self.compute_real_path_station(
                                agent_name, closest_task[1],
                                all_idle_agents, nearest_station,
                                total_real_duration - 1,
                                task_total_consumption, self.simulation.get_batteries_level()[agent_name])

                            # se effettivamente ho un percorso per la stazione calcolato col consumo reale, allora
                            # posso controllare gli endpoint
                            if path_station is not False:
                                discarded_endpoints = set()
                                assigned = False
                                total_real_consumption = task_total_consumption + self.predicted_consumption(
                                    path_station[agent_name])
                                estimated_time_to_recharge = (self.simulation.get_max_autonomies()[agent_name] -
                                                              self.simulation.get_batteries_level()[
                                                                  agent_name] + total_real_consumption) / 10

                                estimated_time_to_recharge = math.ceil(estimated_time_to_recharge)
                                # tra quanti timestep dovrei caricarmi
                                total_real_duration = total_real_duration + estimated_time_to_recharge + to_station_duration - 1
                                closest_non_task_endpoint = 0

                                station_pos = tuple(
                                    (path_station[agent_name][-1]['x'], path_station[agent_name][-1]['y']))
                                while not assigned and closest_non_task_endpoint != -1:
                                    closest_non_task_endpoint = self.get_closest_non_task_endpoint(station_pos,
                                                                                                   discarded_endpoints,
                                                                                                   agent_pos,
                                                                                                   self.token[
                                                                                                       'agents_preemption'][
                                                                                                       agent_name][-1])
                                    if closest_non_task_endpoint != -1:
                                        endpoint_duration = self.admissible_heuristic(closest_non_task_endpoint,
                                                                                      station_pos)

                                        endpoint_consume_heuristic = endpoint_duration * self.move_consumption
                                        if endpoint_consume_heuristic < self.simulation.get_max_autonomies()[
                                            agent_name]:

                                            # real_cost è in step, non in consumo
                                            # previous consumption è 0 perché mi sono appena caricato
                                            path_endpoint, real_cost = self.compute_real_path_to_endpoint(
                                                all_idle_agents, agent_name, station_pos,
                                                closest_non_task_endpoint, 0, total_real_duration - 1,
                                                self.simulation.get_max_autonomies()[
                                                    agent_name])

                                            if path_endpoint is not False:
                                                assigned = True
                                                # ufficializzo il path fino al goal
                                                if agent_name not in self.token['agents_to_tasks']:
                                                    self.token['tasks'].pop(closest_task_name)

                                                self.apply_path(agent_name, agent_pos, path_start[agent_name],
                                                                path_goal[agent_name], closest_task_name,
                                                                closest_task[0], closest_task[1],
                                                                total_real_duration - estimated_time_to_recharge)

                                                self.apply_path_preemption(agent_name, path_start[agent_name],
                                                                           path_goal[agent_name],
                                                                           path_station[agent_name],
                                                                           estimated_time_to_recharge,
                                                                           path_endpoint[agent_name],
                                                                           nearest_station)

                                                print("Task ", closest_task_name, " assegnato all'agente ",
                                                      agent_name)
                                            else:
                                                discarded_endpoints.add(closest_non_task_endpoint)
                                        else:
                                            # se già il più vicino non va bene col calcolo euristico, non ha senso andare avanti
                                            # discarded_endpoints.add(closest_non_task_endpoint)
                                            break

                                if not assigned:
                                    discarded_stations.add(nearest_station)
                            else:
                                discarded_stations.add(nearest_station)

                            nearest_station, consumption_to_station_heuristic = self.search_nearest_available_station(
                                task_total_duration, closest_task[1], agent_name, discarded_stations)

                        # se alla fine del ciclo delle stazioni non ho assegnato il task allora lo metto tra gli scartati
                        if not assigned:
                            discarded_tasks[closest_task_name] = closest_task

                    # qui non metto break perché magari ci sono task con distanza euristica minore, ma che hanno
                    # percorso reale meno esoso in termini di risorse
                    else:
                        break
                else:
                    break
            else:
                break

        return assigned

    def compute_consumption_to_station(self, agent_name):

        path = self.token['agents_preemption'][agent_name]
        consumption = 0
        stations = set()
        for s in self.charging_stations:
            stations.add(tuple(s['pos']))

        # appena arrivo alla stazione smetto di contare il consumo
        for i in range(len(path) - 1):
            if tuple(path[i]) == tuple(path[i + 1]):
                consumption += self.wait_consumption
            else:
                consumption += self.move_consumption
            if tuple(path[i + 1]) in stations:
                break

        return consumption

    def last_time_to_charge(self, agent_name):

        if self.simulation.get_batteries_level()[agent_name] >= self.compute_consumption_to_station(agent_name) >= \
                self.simulation.get_batteries_level()[agent_name] - self.simulation.get_wait_consumption():
            return True
        else:
            return False

    def time_forward(self):

        self.update_completed_tasks()
        self.collect_new_tasks()
        idle_agents = self.get_idle_agents()
        for a in idle_agents:
            self.check_if_dead(idle_agents[a][0], a)
        for a in self.token['dead_agents']:
            del idle_agents[a]
        assigned = False

        while len(idle_agents) > 0:
            agent_name = random.choice(list(idle_agents.keys()))
            all_idle_agents = self.token['agents'].copy()
            all_idle_agents.pop(agent_name)
            agent_pos = idle_agents.pop(agent_name)[0]
            available_tasks = self.find_available_tasks(agent_pos)

            # se ho cancellato il path di un agente che ha già fatto pickup
            if agent_name in self.token['agents_to_tasks'] and self.token['agents_to_tasks'][agent_name][
                'task_name'] != 'charge_complete':
                if not self.compute_new_path_to_goal(all_idle_agents, agent_name, agent_pos):
                    print("Errore nel calcolo del nuovo percorso verso il goal già assegnato")
                else:
                    print("Percorso dell'", agent_name, " verso il goal ricalcolato correttamente")
                # in teoria end_paths già ok, available_tasks anche perché non lo reinserivo
            # se ci sono task disponibili provo ad assegnarli all'agente a patto che almeno a livello euristico poi possa andare
            # a caricarsi. Dopo il check con due euristiche per il task calcolo il percorso reale
            elif len(available_tasks) > 0:

                assigned = self.choose_task_and_station(agent_name, agent_pos, all_idle_agents, available_tasks)
                if not assigned and len(available_tasks) > 0:
                    # questo implica che in passato qualcuno ha già calcolato il mio path per andare a caricarmi
                    # self.decide_if_charge(agent_name, agent_pos, all_idle_agents, idle_agents)
                    if agent_name in self.token['agents_preemption'] and len(self.token['agents'][agent_name]) != len(
                            self.token['agents_preemption'][agent_name]):
                        self.use_preempted_path_to_station(agent_name)

                    # Se non ho un path per andarmi a caricare, lo calcolo. Caso specifico in cui probabilmente non mi sono
                    # mai mosso dal mio non task endpoint e quindi non ho mai calcolato un path per andarmi a caricare
                    else:
                        # self.try_preemption_recharge(agent_name, agent_pos, all_idle_agents, idle_agents)
                        if self.find_new_path_to_station_plus_endpoint(agent_name, agent_pos, all_idle_agents, True):
                            print("Nuovo percorso per andare a caricarsi calcolato correttamente - ", agent_name)
                            self.use_preempted_path_to_station(agent_name)
                        else:
                            print("Errore nel calcolo del percorso per andare a caricarsi dell'", agent_name,
                                  " che non aveva un path precedente")


            # righe 13-14 alg
            # se sono in safe_idle nel momento in cui ho la batteria minima per caricarmi secondo l'euristica vado a caricarmi
            # anche qui se serve cambio il path degli altri agenti
            elif self.check_safe_idle(agent_pos):
                # quindi ho la stazione prenotata
                self.decide_if_charge(agent_name, agent_pos, all_idle_agents, idle_agents)

            # righe 15-16 alg
            else:
                if agent_name in self.token['agents_preemption'] and len(self.token['agents'][agent_name]) != len(
                        self.token['agents_preemption'][agent_name]):

                    # se non ho un percorso per andarmi a caricare dopo che vado nel NON task endpoint, vado a caricarmi
                    if not self.choose_NON_task_endpoint_and_station(agent_name, agent_pos, all_idle_agents):
                        self.use_preempted_path_to_station(agent_name)

                # Sono in una posizione NON safe, ma non ho un percorso preemption... Impossibile
                else:
                    print("sono in una posizione NON safe, ma non ho un percorso preemption... Impossibile ")

                # self.go_to_closest_non_task_endpoint_or_charge(agent_name, agent_pos, all_idle_agents, idle_agents, 0)
                # self.go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents, 0)
