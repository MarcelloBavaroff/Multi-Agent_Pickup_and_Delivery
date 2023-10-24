import argparse
import yaml
import json
import os
import random
from Simulation.TP_battery_2 import TokenPassing
import RoothPath
from Simulation.tasks_and_delays_maker import *
from Simulation.simulation_2 import Simulation
import subprocess
import sys
import ast


def read_tasks():
    data_list = []
    with open('Modified/bugAgentLoop', 'r') as file:
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


if __name__ == '__main__':
    # random.seed(1234)
    parser = argparse.ArgumentParser()
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=5000, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=1, type=int)  # default=1
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true', default=True)

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
                          param['n_tasks'], param['task_freq'])

    # batteria casuale tra 80 e 100
    autonomies = []
    for i in range(len(agents)):
        # autonomies.append(round(random.uniform(80, 100), 2))
        autonomies.append(round(random.uniform(80, 100), 2))

    print(autonomies)

    autonomies = [89.09, 96.18, 94.99, 86.41, 88.19, 86.61, 92.29, 91.93, 95.79, 84.55, 83.32, 83.57, 85.72, 95.36, 89.43, 90.13, 83.22, 84.42, 95.44, 83.03]


    param['autonomies'] = autonomies

    # assegno i tasks generati così poi li vado a scrivere
    param['tasks'] = tasks

    with open(args.param + config['visual_postfix'], 'w') as param_file:
        yaml.safe_dump(param, param_file)

    # Simulate
    simulation = Simulation(tasks, agents, autonomies, charging_stations)
    tp = TokenPassing(agents, dimensions, obstacles, non_task_endpoints, charging_stations, simulation, param['map']['goal_locations'],
                      a_star_max_iter=args.a_star_max_iter, new_recovery=True)
    while tp.get_completed_tasks() != len(tasks) and simulation.get_time() < 1000:
        simulation.time_forward(tp)

    print("Number of completed tasks: ", tp.get_completed_tasks(), "/", len(tasks))
    print("Number of dead agents: ", len(tp.get_token()['dead_agents']))

    for a in tp.get_token()['dead_agents']:
        print("Agent ", a, " died")

    service_time = 0
    for a in tp.get_token()['completed_tasks_times']:
        service_time += tp.get_token()['completed_tasks_times'][a] - tp.get_token()['start_tasks_times'][a]
    print("Average service time: ", service_time / len(tp.get_token()['completed_tasks_times']))

    cost = 0
    for path in simulation.actual_paths.values():
        cost = cost + len(path)
    output = {'schedule': simulation.actual_paths, 'cost': cost,
              'completed_tasks_times': tp.get_completed_tasks_times()}
    # 'n_replans': tp.get_n_replans()}
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)

    # legge dal file di output
    create = [sys.executable, '-m', 'Utils.Visualization.visualize', '-slow_factor', str(args.slow_factor)]
    subprocess.call(create)
