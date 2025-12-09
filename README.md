# Vehicle Routing Problem (VRP) Solver

## Description

This project solves the **Vehicle Routing Problem** using a **genetic algorithm**. The goal is to find optimal routes to deliver to customers with limited capacity vehicles, minimizing the total distance traveled.

## Constraints

- Number of customers: 5 ≤ n ≤ 20 (including the depot)
- Each customer's demand ≤ vehicle capacity
- Total demand of a route ≤ vehicle capacity
- Each vehicle returns to the depot after serving all its customers
- Each customer can only be served once
- Time limit: 10 seconds

## Algorithm Overview

### 1. Genetic Algorithm

The solver uses a genetic algorithm with the following components:

- **Population** : 100 chromosomes (solutions)
- **Selection**: The top 10 chromosomes are kept at each generation
- **Mutation**: 4 types of random mutations:
    - Swap customers between two routes
    - Move a customer to another route
    - Create a new route for a customer
    - Swap positions of two customers in the same route

### 2. Fitness Function

The fitness is calculated as the **total distance** of all routes (Euclidean distance). The objective is to minimize this value.

### 3. Route Generation

Initial routes are created pseudo-randomly while respecting capacity constraints, with a probability to continue filling a route or create a new one.

## Code Structure

- `Point`: Represents a customer or the depot
- `Route`: Contains a list of customers and the current load
- `Chromosome`: Represents a complete solution (set of routes)
- `solve_vrp()`: Main function that executes the genetic algorithm

## Input Format
```
n capacity
index_depot x_depot y_depot demand_depot
index1 x1 y1 demand1
index2 x2 y2 demand2
```

## Output Format
Routes are printed as semicolon-separated lists of customer indices, with each route represented as space-separated indices.
```
1 3 5;2 4;6 7 8
```

## Adjustable Parameters
- POPULATION_SIZE: Number of chromosomes in the population
- BEST_SELECTION_SIZE: Number of the best chromosomes to retain each generation
- MAX_ATTEMPTS: Maximum attempts to generate a valid mutation for one chromosome