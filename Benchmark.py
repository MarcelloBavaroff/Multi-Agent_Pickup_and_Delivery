import argparse
import yaml
import json
import os
# import random
from Simulation.TP_battery_3 import TokenPassing
import RoothPath
from Simulation.tasks_and_delays_maker import *
from Simulation.simulation_3 import Simulation
from Simulation.TP_battery_1 import TokenPassing as TP1
from Simulation.simulation_1 import Simulation as Sim1
import ast


def read_tasks():
    data_list = []
    with open('HardcodedTasks/sovrascriveMovingObs', 'r') as file:
        for line in file:
            try:
                # Valuta la stringa come un dizionario Python
                line_data = ast.literal_eval(line.strip())

                # Verifica che il dizionario abbia i campi richiesti
                if all(key in line_data for key in ['start_time', 'start', 'goal', 'task_name']):
                    data_list.append(line_data)
                else:
                    print(f"Errore: La riga '{line.strip()}' non ha tutti i campi richiesti.")
            except SyntaxError:
                print(f"Errore nella lettura della riga: {line.strip()}")

    return data_list


def parameters(random_seed):

    parser = argparse.ArgumentParser()
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=5000, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=1, type=int)  # default=1
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true',
                        default=False)

    args = parser.parse_args()

    with open(os.path.join(RoothPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    args.param = os.path.join(RoothPath.get_root(), os.path.join(config['input_path'], config['input_name']))
    args.output = os.path.join(RoothPath.get_root(), 'output.yaml')

    # Read from input file, metto tutto dentro param
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimensions = param['map']['dimensions']
    obstacles = param['map']['obstacles']
    non_task_endpoints = param['map']['non_task_endpoints']
    agents = param['agents']
    charging_stations = param['map']['charging_stations']

    if args.not_rand:
        tasks = read_tasks()
    else:
        # Generate random tasks and delays
        tasks = gen_tasks(param['map']['start_locations'], param['map']['goal_locations'],
                          param['n_tasks'], param['task_freq'], random_seed)

    # batteria casuale tra 80 e 100
    autonomies = []
    for i in range(len(agents)):
        autonomies.append(round(random.uniform(80, 100), 2))

    print(autonomies)
    # autonomies = [92.58, 82.74, 82.21, 89.83, 80.19, 91.86, 85.0, 90.43, 87.11, 85.03, 82.95, 97.61, 99.7, 88.11, 92.78, 95.78, 85.4, 96.53, 81.9, 96.15]
    param['autonomies'] = autonomies
    # assegno i tasks generati così poi li vado a scrivere
    param['tasks'] = tasks

    return tasks, agents, autonomies, charging_stations, dimensions, obstacles, non_task_endpoints, param['map'][
        'goal_locations'], args.a_star_max_iter

def print_comparison(version, completed_tasks, n_tasks, dead_agents, makespan, average_service_time, cbs_calls, index_run, random_seed=1234):
    with open('Comparison3/test13.txt', 'a') as file:
        file.write("\n\n" + str(index_run) + " " + version + " " + str(random_seed) + "\n")
        s_completed_tasks = "Number of completed tasks: ", completed_tasks, "/", n_tasks
        s_dead_agents = "Number of dead agents: ", dead_agents
        s_makespan = "Makespan: ", makespan

        s_average_service_time = "Average service time: ", average_service_time
        s_cbs_calls = "Chiamate a CBS: ", cbs_calls

        file.write(str(s_completed_tasks) + '\n' + str(s_dead_agents) + '\n' + str(s_makespan) + '\n' + str(
            s_average_service_time) + '\n' + str(s_cbs_calls))

def single_run(index_run, random_seed):
    tasks, agents, autonomies, charging_stations, dimensions, obstacles, non_task_endpoints, goal_locations, max_iter = parameters(random_seed)

    move_consumption = 0.5
    move_heavy_consumption = move_consumption
    wait_consumption = 0.01

    # Simulate
    simulation = Simulation(tasks, agents, autonomies, charging_stations, move_consumption, wait_consumption, move_heavy_consumption)
    tp = TokenPassing(agents, dimensions, obstacles, non_task_endpoints, charging_stations, simulation,
                      goal_locations, a_star_max_iter=max_iter, new_recovery=True)
    while tp.get_completed_tasks() != len(tasks) and simulation.get_time() < 2500:
        simulation.time_forward(tp)

    completed_tasks = tp.get_completed_tasks()
    n_tasks = len(tasks)
    dead_agents = len(tp.get_token()['dead_agents'])
    makespan = simulation.get_time()

    service_time = 0
    for a in tp.get_token()['completed_tasks_times']:
        service_time += tp.get_token()['completed_tasks_times'][a] - tp.get_token()['start_tasks_times'][a]
    average_service_time = service_time / len(tp.get_token()['completed_tasks_times'])
    cbs_calls = tp.get_chiamateCBS()

    print_comparison("VersionePreem", completed_tasks, n_tasks, dead_agents, makespan, average_service_time, cbs_calls, index_run, random_seed)

    # ---------------------------------------------------------

    # Simulate
    simulation = Sim1(tasks, agents, autonomies, charging_stations, move_consumption, wait_consumption, move_heavy_consumption)
    tp = TP1(agents, dimensions, obstacles, non_task_endpoints, charging_stations, simulation,
             goal_locations, a_star_max_iter=max_iter, new_recovery=True)
    while tp.get_completed_tasks() != len(tasks) and simulation.get_time() < 2000:
        simulation.time_forward(tp)

    completed_tasks2 = tp.get_completed_tasks()
    n_tasks2 = len(tasks)
    dead_agents2 = len(tp.get_token()['dead_agents'])
    makespan2 = simulation.get_time()

    service_time = 0
    for a in tp.get_token()['completed_tasks_times']:
        service_time += tp.get_token()['completed_tasks_times'][a] - tp.get_token()['start_tasks_times'][a]
    average_service_time2 = service_time / len(tp.get_token()['completed_tasks_times'])
    cbs_calls2 = tp.get_chiamateCBS()

    print_comparison("VersioneBase", completed_tasks2, n_tasks2, dead_agents2, makespan2, average_service_time2, cbs_calls2, index_run, random_seed)

    return completed_tasks, n_tasks, dead_agents, makespan, average_service_time, cbs_calls, completed_tasks2, n_tasks2, dead_agents2, makespan2, average_service_time2, cbs_calls2

if __name__ == '__main__':
    # random.seed(1234)

    #1 preemption
    #2 no preemption

    run_complete1 = 0
    run_complete2 = 0
    sum_completed_tasks1 = 0
    sum_completed_tasks2 = 0
    sum_makespan1 = 0
    sum_makespan2 = 0
    sum_service_time1 = 0
    sum_service_time2 = 0
    sum_cbs_calls1 = 0
    sum_cbs_calls2 = 0

    for i in range(20):
        print("Run numero: ", i+1)
        random_seed = random.randint(0, 100000)
        completed_tasks, n_tasks, dead_agents, makespan, average_service_time, cbs_calls, completed_tasks2, n_tasks2, dead_agents2, makespan2, average_service_time2, cbs_calls2 = single_run(i, random_seed)

        if completed_tasks == n_tasks:
            run_complete1 += 1
            sum_makespan1 += makespan
            sum_service_time1 += average_service_time
            sum_cbs_calls1 += cbs_calls
        sum_completed_tasks1 += completed_tasks

        if completed_tasks2 == n_tasks2:
            run_complete2 += 1
            sum_makespan2 += makespan2
            sum_service_time2 += average_service_time2
            sum_cbs_calls2 += cbs_calls2
        sum_completed_tasks2 += completed_tasks2

    print("\nVersionePreem")
    print("Numero di run completate: ", run_complete1)
    print("Numero medio di task completati: ", sum_completed_tasks1/20)
    print("Makespan medio: ", sum_makespan1/run_complete1)
    print("Tempo medio di servizio: ", sum_service_time1/run_complete1)
    print("Chiamate a CBS: ", sum_cbs_calls1/run_complete1)

    print("\nVersioneBase")
    print("Numero di run completate: ", run_complete2)
    print("Numero medio di task completati: ", sum_completed_tasks2 / 20)
    print("Makespan medio: ", sum_makespan2/run_complete2)
    print("Tempo medio di servizio: ", sum_service_time2/run_complete2)
    print("Chiamate a CBS: ", sum_cbs_calls2/run_complete2)
