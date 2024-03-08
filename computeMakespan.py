import re
import statistics

# Leggi il file e ottieni le righe che contengono il makespan
with open('Comparisons/BigSeparate/change/test17.txt', 'r') as file:
    makespan_lines = [line for line in file if 'Makespan:' in line]

# Estrai i valori di makespan dalle righe usando espressioni regolari, escludendo i valori pari a 30000
makespan_values = []
for line in makespan_lines:
    try:
        makespan_value = re.search(r'\d+', line)
        makespan_value = int(makespan_value.group())
        if makespan_value != 30000:
            makespan_values.append(makespan_value)
    except AttributeError:
        pass  # Ignora le righe che non contengono il valore Makespan

# Calcola la media e la deviazione standard del makespan
makespan_mean = statistics.mean(makespan_values)
makespan_std_dev = statistics.stdev(makespan_values)

print("Media del makespan (esclusi i valori pari a 30000):", makespan_mean)
print("Deviazione standard del makespan (esclusi i valori pari a 30000):", makespan_std_dev)
print("Numero di valori di makespan (esclusi i valori pari a 30000):", len(makespan_values))



