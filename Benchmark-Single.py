import argparse
import yaml
import json
import os
# import random
from Simulation.TP_battery_Queue import TokenPassing
import RoothPath
from Simulation.tasks_and_delays_maker import *
from Simulation.simulation_Queue import Simulation
import ast


def parameters(seed):
    random.seed(seed)
    parser = argparse.ArgumentParser()
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm', default=5000, type=int)
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
    non_task_endpoints = param['map']['non_task_endpoints']
    agents = param['agents']
    charging_stations = param['map']['charging_stations']

    if args.not_rand:
        tasks = None
    else:
        # Generate random tasks and delays
        tasks = gen_tasks(param['map']['start_locations'], param['map']['goal_locations'],
                          param['n_tasks'], param['task_freq'], random_seed)

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

def print_comparison(version, completed_tasks, n_tasks, dead_agents, makespan, average_service_time, cbs_calls,
                     index_run, cbs_calls_recharge, random_seed=1234):
    with open('Comparisons/Comp1/test2.txt', 'a') as file:
        file.write("\n\n" + str(index_run) + " " + version + " " + str(random_seed) + "\n")
        s_completed_tasks = "Number of completed tasks: ", completed_tasks, "/", n_tasks
        s_dead_agents = "Number of dead agents: ", dead_agents
        s_makespan = "Makespan: ", makespan

        s_average_service_time = "Average service time: ", average_service_time
        s_cbs_calls_recharge = "Chiamate a CBS per stazioni di ricarica: ", cbs_calls_recharge
        s_cbs_calls = "Chiamate a CBS: ", cbs_calls

        file.write(str(s_completed_tasks) + '\n' + str(s_dead_agents) + '\n' + str(s_makespan) + '\n' + str(
            s_average_service_time) + '\n' + str(s_cbs_calls_recharge) + '\n' + str(s_cbs_calls))

def single_run(index_run, random_seed):
    tasks, agents, autonomies, charging_stations, dimensions, obstacles, non_task_endpoints, goal_locations, max_iter = parameters(
        random_seed)

    move_consumption = 0.5
    move_heavy_consumption = move_consumption
    wait_consumption = 0.5

    # Simulate
    simulation = Simulation(tasks, agents, autonomies, charging_stations, move_consumption, wait_consumption,
                            move_heavy_consumption)
    tp = TokenPassing(agents, dimensions, obstacles, non_task_endpoints, charging_stations, simulation,
                      goal_locations, a_star_max_iter=max_iter, new_recovery=True)
    while tp.get_completed_tasks() != len(tasks) and simulation.get_time() < 3000:
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
    cbs_calls_recharge = tp.get_chiamateCBS_recharge()

    print_comparison("VersioneQueue", completed_tasks, n_tasks, dead_agents, makespan, average_service_time, cbs_calls,
                     index_run, cbs_calls_recharge, random_seed)
    # ---------------------------------------------------------

    # # Simulate
    # simulation = Sim1(tasks, agents, autonomies, charging_stations, move_consumption, wait_consumption, move_heavy_consumption)
    # tp = TP1(agents, dimensions, obstacles, non_task_endpoints, charging_stations, simulation,
    #          goal_locations, a_star_max_iter=max_iter, new_recovery=True)
    # while tp.get_completed_tasks() != len(tasks) and simulation.get_time() < 2000:
    #     simulation.time_forward(tp)
    #
    # completed_tasks2 = tp.get_completed_tasks()
    # n_tasks2 = len(tasks)
    # dead_agents2 = len(tp.get_token()['dead_agents'])
    # makespan2 = simulation.get_time()
    #
    # service_time = 0
    # for a in tp.get_token()['completed_tasks_times']:
    #     service_time += tp.get_token()['completed_tasks_times'][a] - tp.get_token()['start_tasks_times'][a]
    # average_service_time2 = service_time / len(tp.get_token()['completed_tasks_times'])
    # cbs_calls2 = tp.get_chiamateCBS()
    # cbs_calls_recharge2 = tp.get_chiamateCBS_recharge()
    #
    # print_comparison("VersioneBase", completed_tasks2, n_tasks2, dead_agents2, makespan2, average_service_time2, cbs_calls2, index_run, cbs_calls_recharge2, random_seed)
    return completed_tasks, n_tasks, dead_agents, makespan, average_service_time, cbs_calls, cbs_calls_recharge  # , completed_tasks2, n_tasks2, dead_agents2, makespan2, average_service_time2, cbs_calls2, cbs_calls_recharge2

if __name__ == '__main__':

    run_complete1 = 0
    sum_completed_tasks1 = 0
    sum_makespan1 = 0
    sum_service_time1 = 0
    sum_cbs_calls1 = 0
    sum_cbs_calls_recharge1 = 0
    sum_dead_agents1 = 0

    with open('Comparisons/seeds2.txt', 'r') as file:
        # inserisci ogni riga in una lista
        seeds = file.read()
    seeds = seeds.split(" ")

    for i in range(20):
        print("Run numero: ", i + 1)
        #random_seed = random.randint(0, 100000)
        random_seed = int(seeds[i])
        completed_tasks, n_tasks, dead_agents, makespan, average_service_time, cbs_calls, cbs_calls_recharge = single_run(i, random_seed)

        if completed_tasks == n_tasks:
            run_complete1 += 1
            sum_makespan1 += makespan
            sum_service_time1 += average_service_time
            sum_cbs_calls1 += cbs_calls
            sum_cbs_calls_recharge1 += cbs_calls_recharge
        sum_completed_tasks1 += completed_tasks
        sum_dead_agents1 += dead_agents

    #print("\nVersioneChange")
    print("Numero di run completate: ", run_complete1)
    print("Numero medio di task completati: ", sum_completed_tasks1 / 20)
    print("Numero medio di agenti morti: ", sum_dead_agents1 / 20)
    print("Makespan medio: ", sum_makespan1 / run_complete1)
    print("Tempo medio di servizio: ", sum_service_time1 / run_complete1)
    print("Chiamate a CBS per stazioni di ricarica: ", sum_cbs_calls_recharge1 / run_complete1)
    print("Chiamate a CBS totali: ", sum_cbs_calls1 / run_complete1)

