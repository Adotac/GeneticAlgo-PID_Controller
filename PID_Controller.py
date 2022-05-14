import numpy as np
import matplotlib.pyplot as plt

import itertools
import statistics as st
import pure_lineTracker as tracker

import random

POP_MAX = 5 # Population size
KSIZE = 3 # P, I, D
KLIMIT = 5 # Maximum of PID values
PLIMIT = 40  # Maximum value of coordinate values / value of x

LIMIT_RANGE = 0 # size of K limit arrays
for row in np.arange(0, PLIMIT, 0.1):
    LIMIT_RANGE += 1
MIN_LR = (LIMIT_RANGE*0.5)-((LIMIT_RANGE*0.5)*(PLIMIT/100)) # minimum plot points range

CLOSEST_PID = () # ( [PID], generation, distance, plots )
cumulative_distance = np.array([0.0]) # to get the mean and margin of error of distances

def distance(coordinates, sum=False):
    dx = []
    dy = []
    for index in range(len(coordinates[0])):
        if index < LIMIT_RANGE:
            dx.append(cgraph[0][index] - coordinates[0][index])
            dy.append(cgraph[1][index] - coordinates[1][index])

    if sum:
        return np.sum(np.hypot(dx, dy))
    else:
        return np.hypot(dx, dy)

def fitness_score(individual): # array of x & y points
    rear_states = tracker.fitness_simulation(individual) # generated plot points

    score_distance = distance(rear_states, sum=True)
    return score_distance, rear_states

def gen_populate():
    pop = []
    for i in range(POP_MAX):
        new = [float(round(np.random.uniform(0, KLIMIT),2)) for i in range(KSIZE)] # random floats
        pop.append(new)
    return pop

def selection(population):
    parents = []
    pick = random.uniform(0, LIMIT_RANGE)
    current = 0
    for ind in population:
        # select parents with roulette wheel selection
        FS = fitness_score(ind)[0]
        current += FS
        if current > pick:
            parents.append(ind)

    return parents

def crossover(parents):
    MIXING_PAIR = 2
    cross_point = random.randint(0, KSIZE-1)
    offsprings = []
    combination = list(itertools.combinations(parents, MIXING_PAIR)) # 2d array of pairs
    for comb in combination:
        parent1_right = comb[0][cross_point:]
        parent2_right = comb[1][:3-cross_point]
        comb[0][cross_point:] = parent2_right
        comb[1][:3-cross_point] = parent1_right
        for i in comb:
            offsprings.append(i)

    return offsprings

def mutation(offspring):
    MUTATION_RATE = 0.2
    for i in range(KSIZE):
        if random.random() < MUTATION_RATE:
            offspring[i] = float(round(np.random.uniform(0, KLIMIT),2))

    return offspring

def evolve(population):

    parents = selection(population)

    offsprings = crossover(parents)
    offsprings = list(map(mutation, offsprings))
    new_gen = offsprings
    for ind in population:
        new_gen.append(ind)

    new_gen = sorted(new_gen, key=lambda x: fitness_score(x)[0], reverse=False)

    return new_gen[:POP_MAX]

def check_FS(population, generation):
    global CLOSEST_PID
    global cumulative_distance

    cumulative_distance = np.hstack((cumulative_distance,
                        np.array(distance(fitness_score(population[0])[1])))) # only get the lowest distance in the population

    # z value * stdev / sample
    merr = 2.58 * (st.stdev(cumulative_distance) / POP_MAX)  # 99% confidence interval

    for ind in population:
        SD, r_points = fitness_score(ind)
        LRS = len(r_points[0])

        print(f'{ind} | G.{generation} | Distance from path:{SD} | N# of plots:{LRS}')
        if (SD <= np.sum(cumulative_distance)*merr) and LRS >= MIN_LR:
            CLOSEST_PID = (list(ind), generation, SD, LRS)

            hideOnly_selectedGraph(population, cgraph, index=generation)
            print(f'Closest Config Found!')
            return True

        if SD == 0: # one in a lifetime miracle perfect result
            print(f'Config Found!')
            return True

    return False

def main():
    print(LIMIT_RANGE)
    population = gen_populate()
    print(f'starts at population: {population}')
    generation = 0
    plt.cla()

    while True:
        # if (generation >= GEN_MAX):
        #     break

        population = evolve(population)
        hideOnly_selectedGraph(population, cgraph, start=4, index=generation)


        if check_FS(population, generation):
            break
        print(f'Generation: {generation}')

        generation += 1
        #print(f'main loop CLOSEST PID {CLOSEST_PID}')



    print(f'\nThe closest path is PID: {CLOSEST_PID[0]}'
          f'\nDistance is {CLOSEST_PID[2]}'
          f'\nN of Plots: {CLOSEST_PID[3]}'
          f'\nfrom G.{CLOSEST_PID[1]}')

    # SHOW THE PID THAT RESULTS CLOSEST TO ORIGINAL PATH #
    gg_x, gg_y = tracker.simulation(CLOSEST_PID[0], show=True)
    plt.figure(None)
    plt.plot(cgraph[0], cgraph[1], ".r", label="path")
    plt.plot(gg_x, gg_y, c='b', label=f"{CLOSEST_PID[0]}")
    plt.legend()
    plt.title(f"Closest PID control with D:{CLOSEST_PID[2]},"
              f"\n No. of plots:{CLOSEST_PID[3]}"
              f"\nFrom Generation {CLOSEST_PID[1]}")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.grid(True)
    # -------------------------------------------------- #

    plt.show()

def hideOnly_selectedGraph(population, cgraph, start=None, index=None):

    if (start is not None) and (index <= start):

        plt.figure(index)
        plt.plot(cgraph[0], cgraph[1], ".r", label="path")
        for i in population:
            col = (random.random(), random.random(), random.random())
            ss_x, ss_y = tracker.simulation(i)
            plt.plot(ss_x, ss_y, c=col, label=f"{i}")

        plt.legend()
        plt.title(f"Generation {index}")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.grid(True)
    elif index is not None and start is None:
        plt.figure(index)
        plt.plot(cgraph[0], cgraph[1], ".r", label="path")
        for i in population:
            col = (random.random(), random.random(), random.random())
            ss_x, ss_y = tracker.simulation(i)
            plt.plot(ss_x, ss_y, c=col, label=f"{i}")

        plt.legend()
        plt.title(f"Generation {index}")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.grid(True)

if __name__ == "__main__":
    cgraph = tracker.graph()

    main()