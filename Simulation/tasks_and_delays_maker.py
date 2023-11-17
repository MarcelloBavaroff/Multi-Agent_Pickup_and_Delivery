import random
import math

def gen_tasks(starts, goals, n_tasks, task_freq, random_seed):
    arrival_time = 0
    tasks = []

    random.seed(random_seed)
    for i in range(n_tasks):
        # Get the next probability value from Uniform(0,1)
        p = random.random()
        # Plug it into the inverse of the CDF of Exponential(task_freq)
        inter_arrival_time = -math.log(1.0 - p) / task_freq
        # Add the inter-arrival time to the running sum
        arrival_time = arrival_time + inter_arrival_time
        # Generate task
        single_task = {'start_time': int(arrival_time), 'start': random.choice(starts), 'goal': random.choice(goals),
                      'task_name': 'task' + str(i)}
        tasks.append(single_task)
        print(single_task)

    return tasks
