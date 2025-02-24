import argparse
import yaml
import json
import os
#from Simulation.G_TP.G_TP import TokenPassing
#from Simulation.G_TP.simulation_G_TP import Simulation
from Simulation.A_TP.A_TP import TokenPassing
from Simulation.A_TP.simulation_A_TP import Simulation
#from Simulation.Versione_Queue.TP_battery_Queue import TokenPassing
#from Simulation.Versione_Queue.simulation_Queue import Simulation
#from Simulation.TP_battery_Queue_Long import TokenPassing
#from Simulation.simulation_Queue_Long import Simulation

import RoothPath
from Simulation.tasks_maker import *

import subprocess
import sys
import ast


def read_tasks():
    data_list = []
    with open('HardcodedTasks/ErroreTest8-Base', 'r') as file:
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
    #seed = random.randint(0, 100000)
    seed = 95708
    random.seed(seed)
    print("Seed: ", seed)
    parser = argparse.ArgumentParser()
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=15000, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=5, type=int)  # default=1
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true', default=False)

    args = parser.parse_args()
    with open(os.path.join(RoothPath.get_root(), 'config_demo.json'), 'r') as json_file:
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
    if type(obstacles[0]) is not tuple:
        obstacles = [tuple((obstacle[0], obstacle[1])) for obstacle in obstacles]

    non_task_endpoints = param['map']['non_task_endpoints']
    if type(non_task_endpoints[0]) is not tuple:
        non_task_endpoints = [tuple((non_task_endpoint[0], non_task_endpoint[1])) for non_task_endpoint in
                              non_task_endpoints]

    agents = param['agents']
    charging_stations = param['map']['charging_stations']

    if args.not_rand:
        tasks = read_tasks()
    else:
        # Generate random tasks and delays
        tasks = gen_tasks(param['map']['start_locations'], param['map']['goal_locations'],
                          param['n_tasks'], param['task_freq'], random_seed=seed)

    # batteria casuale tra 80 e 100
    autonomies = []
    for i in range(len(agents)):
        autonomies.append(round(random.uniform(80, 100), 2))
    print(autonomies)
    # autonomies = [88.32, 80.15, 80.98, 91.02, 81.92, 87.65, 99.62, 95.54, 82.78, 97.65, 81.06, 90.2, 95.29, 82.58, 89.13, 81.58, 97.27, 96.99, 95.43, 81.79]

    param['autonomies'] = autonomies
    # assegno i tasks generati così poi li vado a scrivere
    param['tasks'] = tasks
    with open(args.param + config['visual_postfix'], 'w') as param_file:
        yaml.safe_dump(param, param_file)

    # Simulate
    simulation = Simulation(tasks, agents, autonomies, charging_stations, 0.1, 0.1, 0.1)
    tp = TokenPassing(agents, dimensions, obstacles, non_task_endpoints, charging_stations, simulation,
                      param['map']['goal_locations'], a_star_max_iter=args.a_star_max_iter, new_recovery=True)
    while tp.get_completed_tasks() != len(tasks) and simulation.get_time() < 20000:
        simulation.time_forward(tp)

    print("Number of completed tasks: ", tp.get_completed_tasks(), "/", len(tasks))
    print("Number of dead agents: ", len(tp.get_token()['dead_agents']))

    for a in tp.get_token()['dead_agents']:
        print("Agent ", a, " died")

    delta_times = []
    for a in tp.get_token()['completed_tasks_times']:
        delta_times.append(tp.get_token()['completed_tasks_times'][a] - tp.get_token()['start_tasks_times'][a])

    service_time = sum(delta_times)
    average_service_time = service_time / len(tp.get_token()['completed_tasks_times'])
    variance = sum((x - average_service_time) ** 2 for x in delta_times) / len(tp.get_token()['completed_tasks_times'])
    std_dev = math.sqrt(variance)

    print("Chiamate a CBS per ricaricarsi: ", tp.get_chiamateCBS_recharge())
    print("Chiamate a CBS: ", tp.get_chiamateCBS())
    print("Espansioni media A*: ", tp.get_avg_espansioniA())
    # print("Numero medio passi/steps per task: ",
    #       tp.get_totalePassiTasks() / len(tp.get_token()['completed_tasks_times']))

    # print all delta times
    # i = 0
    # for a in tp.get_token()['completed_tasks_times']:
    #     print(str(tp.get_token()['completed_tasks_times'][a]) + '-' + str(delta_times[i]))
    #     i += 1

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
