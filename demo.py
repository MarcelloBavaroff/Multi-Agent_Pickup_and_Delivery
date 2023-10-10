import argparse
import yaml
import json
import os
import random
from Simulation.TP_with_recovery import TokenPassingRecovery
import RoothPath
from Simulation.tasks_and_delays_maker import *
from Simulation.simulation_new_recovery import SimulationNewRecovery
import subprocess
import sys
import ast

def read_tasks():
    data_list = []
    with open('Modified/Hardcoded_Tasks', 'r') as file:
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
    #random.seed(1234)
    parser = argparse.ArgumentParser()
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=5000, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=3, type=int) #default=1
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true', default=True)

    args = parser.parse_args()

    # if args.k is None:
    #     args.k = 0
    # if args.p is None:
    #     args.p = 1

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

    #batteria casuale tra 80 e 100
    autonomies = []
    for i in range(len(agents)):
        autonomies.append(round(random.uniform(80, 100), 2))

    param['autonomies'] = autonomies

    #assegno i tasks generati così poi li vado a scrivere
    param['tasks'] = tasks

    with open(args.param + config['visual_postfix'], 'w') as param_file:
        yaml.safe_dump(param, param_file)

    # Simulate
    simulation = SimulationNewRecovery(tasks, agents, autonomies)
    tp = TokenPassingRecovery(agents, dimensions, obstacles, non_task_endpoints, charging_stations, simulation,
                              a_star_max_iter=args.a_star_max_iter, new_recovery=True)
    while tp.get_completed_tasks() != len(tasks) and simulation.get_time() < 400:
        simulation.time_forward(tp)

    cost = 0
    for path in simulation.actual_paths.values():
        cost = cost + len(path)
    output = {'schedule': simulation.actual_paths, 'cost': cost,
              'completed_tasks_times': tp.get_completed_tasks_times(),
              'n_replans': tp.get_n_replans()}
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)

    #legge dal file di output
    create = [sys.executable, '-m', 'Utils.Visualization.visualize', '-slow_factor', str(args.slow_factor)]
    subprocess.call(create)
