import argparse
import yaml
import json
import os
# import random

import RoothPath
from Simulation.tasks_and_delays_maker import *
#from Simulation.Versione_Queue.TP_battery_Queue import TokenPassing
#from Simulation.Versione_Queue.simulation_Queue import Simulation
#from Simulation.Versione_Preemption.TP_battery_Preem import TokenPassing
#from Simulation.Versione_Preemption.simulation_Preem import Simulation
#from Simulation.Versione_Change.TP_battery_Change2 import TokenPassing
#from Simulation.Versione_Change.simulation_Change2 import Simulation
#from Simulation.TP_battery_Queue_Long import TokenPassing
#from Simulation.simulation_Queue_Long import Simulation


def parameters(seed):
    random.seed(seed)
    parser = argparse.ArgumentParser()
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=15000, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=1, type=int)  # default=1
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true', default=False)
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
    if type(obstacles[0]) is not tuple:
        obstacles = [tuple((obstacle[0], obstacle[1])) for obstacle in obstacles]

    non_task_endpoints = param['map']['non_task_endpoints']
    if type(non_task_endpoints[0]) is not tuple:
        non_task_endpoints = [tuple((non_task_endpoint[0], non_task_endpoint[1])) for non_task_endpoint in
                              non_task_endpoints]
    agents = param['agents']
    charging_stations = param['map']['charging_stations']

    if args.not_rand:
        tasks = None
    else:
        # Generate random tasks and delays
        #tasks = gen_tasks(param['map']['start_locations'], param['map']['goal_locations'],param['n_tasks'], param['task_freq'], random_seed)
        tasks = gen_tasks(param['map']['start_locations'], param['map']['goal_locations'], 2000, 0.3, random_seed)

    # batteria casuale tra 80 e 100
    autonomies = []
    for i in range(len(agents)):
        autonomies.append(round(random.uniform(80, 100), 2))

    print(autonomies)
    param['autonomies'] = autonomies
    param['tasks'] = tasks

    # with open('Comparisons/seeds2.txt', 'a') as file:
    #     file.write(str(seed) + " ")

    return tasks, agents, autonomies, charging_stations, dimensions, obstacles, non_task_endpoints, param['map'][
        'goal_locations'], args.a_star_max_iter


def print_comparison(version, completed_tasks, n_tasks, dead_agents, makespan, average_service_time, std_dev,cbs_calls,
                     index_run, cbs_calls_recharge, random_seed=1234, file_name='Comparisons/Comp1/test3.txt'):
    with open(file_name, 'a') as file:
        file.write("\n\n" + str(index_run) + " " + version + " " + str(random_seed) + "\n")
        s_completed_tasks = "Number of completed tasks: ", completed_tasks, "/", n_tasks
        s_dead_agents = "Number of dead agents: ", dead_agents
        s_makespan = "Makespan: ", makespan

        s_average_service_time = "Average service time: ", average_service_time
        s_std_dev = "Standard deviation: ", std_dev
        s_cbs_calls_recharge = "Chiamate a CBS per stazioni di ricarica: ", cbs_calls_recharge
        s_cbs_calls = "Chiamate a CBS: ", cbs_calls

        file.write(str(s_completed_tasks) + '\n' + str(s_dead_agents) + '\n' + str(s_makespan) + '\n' + str(
            s_average_service_time) + '\n' + str(s_std_dev) + '\n' + str(s_cbs_calls_recharge) + '\n' + str(s_cbs_calls))


def single_run(index_run, random_seed, file_name, move_consumption=1.0, move_heavy_consumption=1.0, wait_consumption=1.0):
    tasks, agents, autonomies, charging_stations, dimensions, obstacles, non_task_endpoints, goal_locations, max_iter = parameters(
        random_seed)

    # Simulate
    simulation = Simulation(tasks, agents, autonomies, charging_stations, move_consumption, wait_consumption,
                            move_heavy_consumption)
    tp = TokenPassing(agents, dimensions, obstacles, non_task_endpoints, charging_stations, simulation,
                      goal_locations, a_star_max_iter=max_iter, new_recovery=True)
    while tp.get_completed_tasks() != len(tasks) and simulation.get_time() < 20000:
        simulation.time_forward(tp)

    completed_tasks = tp.get_completed_tasks()
    n_tasks = len(tasks)
    dead_agents = len(tp.get_token()['dead_agents'])
    makespan = simulation.get_time()

    delta_times = []
    for a in tp.get_token()['completed_tasks_times']:
        delta_times.append(tp.get_token()['completed_tasks_times'][a] - tp.get_token()['start_tasks_times'][a])

    service_time = sum(delta_times)
    average_service_time = service_time / len(tp.get_token()['completed_tasks_times'])
    variance = sum((x - average_service_time) ** 2 for x in delta_times) / len(tp.get_token()['completed_tasks_times'])
    std_dev = math.sqrt(variance)

    cbs_calls = tp.get_chiamateCBS()
    cbs_calls_recharge = tp.get_chiamateCBS_recharge()

    print_comparison("VersioneQueue", completed_tasks, n_tasks, dead_agents, makespan, average_service_time, std_dev,cbs_calls,
                     index_run, cbs_calls_recharge, random_seed, file_name)

    return completed_tasks, n_tasks, dead_agents, makespan, average_service_time, std_dev, cbs_calls, cbs_calls_recharge  # , completed_tasks2, n_tasks2, dead_agents2, makespan2, average_service_time2, cbs_calls2, cbs_calls_recharge2


if __name__ == '__main__':

    run_complete1 = 0
    sum_completed_tasks1 = 0
    sum_makespan1 = 0
    sum_service_time1 = 0
    sum_std_dev1 = 0
    sum_cbs_calls1 = 0
    sum_cbs_calls_recharge1 = 0
    sum_dead_agents1 = 0

    file_name = 'Comparisons/ForLong/Change/3.txt'
    move_consumption = 1
    move_heavy_consumption = move_consumption*2
    wait_consumption = 0.1

    with open('Comparisons/seeds2.txt', 'r') as file:
        # inserisci ogni riga in una lista
        seeds = file.read()
    seeds = seeds.split(" ")

    for i in range(20):
        print("Run numero: ", i + 1)
        # random_seed = random.randint(0, 100000)
        random_seed = int(seeds[i])
        completed_tasks, n_tasks, dead_agents, makespan, average_service_time, std_dev, cbs_calls, cbs_calls_recharge = single_run(
            i, random_seed, file_name, move_consumption, move_heavy_consumption, wait_consumption)

        if completed_tasks == n_tasks:
            run_complete1 += 1
            sum_makespan1 += makespan
            sum_service_time1 += average_service_time
            sum_std_dev1 += std_dev
            sum_cbs_calls1 += cbs_calls
            sum_cbs_calls_recharge1 += cbs_calls_recharge

        sum_completed_tasks1 += completed_tasks
        sum_dead_agents1 += dead_agents

    # print("\nVersioneChange")
    print("move_consumption: ", move_consumption)
    print("move_heavy_consumption: ", move_heavy_consumption)
    print("wait_consumption: ", wait_consumption)
    print("Numero di run completate: ", run_complete1)
    print("Numero medio di task completati: ", sum_completed_tasks1 / 20)
    print("Numero medio di agenti morti: ", sum_dead_agents1 / 20)
    try:
        print("Makespan medio: ", sum_makespan1 / run_complete1)
        print("Tempo medio di servizio: ", sum_service_time1 / run_complete1)
        print("Deviazione standard media: ", sum_std_dev1 / run_complete1)
        print("Chiamate a CBS per stazioni di ricarica: ", sum_cbs_calls_recharge1 / run_complete1)
        print("Chiamate a CBS totali: ", sum_cbs_calls1 / run_complete1)
    except:
        print("0 run completate")

    with open(file_name, 'a') as file:
        file.write("\n\n" + "Numero di run completate: " + str(run_complete1) + "\n")
        file.write("Numero medio di task completati: " + str(sum_completed_tasks1 / 20) + "\n")
        file.write("Numero medio di agenti morti: " + str(sum_dead_agents1 / 20) + "\n")
        try:
            file.write("Makespan medio: " + str(sum_makespan1 / run_complete1) + "\n")
            file.write("Tempo medio di servizio: " + str(sum_service_time1 / run_complete1) + "\n")
            file.write("Deviazione standard media: " + str(sum_std_dev1 / run_complete1) + "\n")
            file.write("Chiamate a CBS per stazioni di ricarica: " + str(sum_cbs_calls_recharge1 / run_complete1) + "\n")
            file.write("Chiamate a CBS totali: " + str(sum_cbs_calls1 / run_complete1) + "\n")
        except:
            file.write("0 run completate")

