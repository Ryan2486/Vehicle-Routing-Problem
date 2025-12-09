#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>

using namespace std;

/**
 *  Constraint:
 *  Number of clients: 5 <= n <= 20 ( including depot )
 *  Client demand <= vehicle capacity
 *  Total demand of clients in a route <= vehicle capacity
 *  Return to depot after all clients in a route are served
 *  Can only serve each client once
 *  Time to found a solution is limited to 10 second ( 10000 ms )
 */

static mt19937 gen(random_device{}());
constexpr int POPULATION_SIZE = 100;
constexpr int BEST_SELECTION_SIZE = POPULATION_SIZE * 0.1;
constexpr int MAX_ATTEMPTS = 50;

static vector<vector<double>> distance_cache;
static bool cache_initialized = false;

struct Point{
    int index;
    int x;
    int y;
    int demand;
};

struct Route{
    vector<Point> clients;
    int load = 0;

    void addClient(const Point& client) {
        clients.push_back(client);
        load += client.demand;
    };

    void addClientAt(const Point& client,const int index) {
        clients.insert(clients.begin() + index, client);
        load += client.demand;
    };

    void removeClientAt(const int index) {
        load -= clients[index].demand;
        clients.erase(clients.begin() + index);
    };

    void swapPositions(const int indexA, const int indexB) {
        swap(clients[indexA], clients[indexB]);
    };

    [[nodiscard]] int clientCount() const {
        return static_cast<int>(clients.size());
    }
};

struct Chromosome{
    vector<Route> routes;
    double fitness = 0;

    void deleteEmptyRoutes() {
        routes.erase(ranges::remove_if(routes,
                                       [](const Route& route) { return route.clients.empty(); }).begin(),
                     routes.end());
    }

    void moveClientRoute(const int fromRouteIdx, const int toRouteIdx, const int clientIdx) {
        const Point client = routes[fromRouteIdx].clients[clientIdx];
        routes[fromRouteIdx].removeClientAt(clientIdx);
        routes[toRouteIdx].addClient(client);
    }

    void moveClientToNewRoute(const int fromRouteIdx, const int clientIdx) {
        const Point client = routes[fromRouteIdx].clients[clientIdx];
        routes[fromRouteIdx].removeClientAt(clientIdx);
        Route newRoute;
        newRoute.addClient(client);
        newRoute.load = client.demand;
        routes.push_back(newRoute);
    };

    void swapClientsBetweenRoutes(const int routeAIdx, const int clientAIdx, const int routeBIdx, const int clientBIdx) {
        const Point clientA = routes[routeAIdx].clients[clientAIdx];
        const Point clientB = routes[routeBIdx].clients[clientBIdx];

        routes[routeAIdx].clients[clientAIdx] = clientB;
        routes[routeAIdx].load = routes[routeAIdx].load - clientA.demand + clientB.demand;

        routes[routeBIdx].clients[clientBIdx] = clientA;
        routes[routeBIdx].load = routes[routeBIdx].load - clientB.demand + clientA.demand;
    }

    void swapClientsWithinRoute(const int routeIdx, const int clientAIdx, const int clientBIdx) {
        routes[routeIdx].swapPositions(clientAIdx, clientBIdx);
    }
};

void initDistanceCache(const vector<Point>& clients, const Point& depot) {
    int max_index = depot.index;
    for (const auto& client : clients) {
        max_index = max(max_index, client.index);
    }
    distance_cache.assign(max_index + 1, vector<double>(max_index + 1, -1.0));
    cache_initialized = true;
}

double distance(const Point& a, const Point& b) {
    if (cache_initialized && distance_cache[a.index][b.index] >= 0.0) return distance_cache[a.index][b.index];
    const double dist = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    if (cache_initialized) {
        distance_cache[a.index][b.index] = dist;
        distance_cache[b.index][a.index] = dist;
    }
    return dist;
}

double calculate_route_distance(const Route& route, const Point& depot) {
    double total_distance = 0.0;
    Point previous = depot;
    for (const Point& client : route.clients) {
        total_distance += distance(previous, client);
        previous = client;
    }
    total_distance += distance(previous, depot);
    return total_distance;
}

double score(const vector<Route>& routes, const Point& depot) {
    double total_distance = 0.0;
    for (const Route& route : routes) {
        total_distance += calculate_route_distance(route, depot);
    }
    return total_distance;
}

void apply2Opt(Route& route, const Point& depot) {
    const int n = route.clientCount();
    if (n < 4) return;

    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 2; j < n; j++) {
                const Point& before_i = (i == 0) ? depot : route.clients[i - 1];
                const Point& after_j = (j == n - 1) ? depot : route.clients[j + 1];

                double old_edges = distance(before_i, route.clients[i]) +
                                  distance(route.clients[j], after_j);

                double new_edges = distance(before_i, route.clients[j]) +
                                  distance(route.clients[i], after_j);

                if (new_edges < old_edges) {
                    reverse(route.clients.begin() + i, route.clients.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}


string format_output(const vector<Route>& routes) {
    string output;
    for (int i = 0; i < routes.size(); i++) {
        for (int j = 0; j < routes[i].clients.size(); j++) {
            output += to_string(routes[i].clients[j].index);
            if (j < routes[i].clients.size() - 1) {
                output += " ";
            }
        }
        if (i < routes.size() - 1) {
            output += ";";
        }
    }
    return output;
}

vector<Point> shuffle_clients(const vector<Point>& clients) {
    vector<Point> shuffled_clients = clients;
    ranges::shuffle(shuffled_clients, gen);
    return shuffled_clients;
}


vector<Route> create_routes(const vector<Point>& clients, int capacity) {
    uniform_real_distribution<> dis(0.0, 1.0);

    vector<Route> routes;
    Route current_route;
    int current_load = 0;

    for (const Point& client : clients) {
        if (current_load + client.demand <= capacity) {
            double fill_ratio = static_cast<double>(current_load) / capacity;
            double continue_probability = 0.1 + (fill_ratio * 0.4);

            if (dis(gen) < continue_probability || current_route.clients.empty()) {
                current_route.addClient(client);
                current_load += client.demand;
            } else {
                routes.push_back(current_route);
                current_route = Route();
                current_route.addClient(client);
                current_load = client.demand;
            }
        } else {
            if (!current_route.clients.empty()) {
                routes.push_back(current_route);
            }
            current_route = Route();
            current_route.addClient(client);
            current_load = client.demand;
        }
    }
    if (!current_route.clients.empty()) {
        current_route.load = current_load;
        routes.push_back(current_route);
    }
    return routes;
}

vector<Chromosome> initialize_population(const vector<Point>& clients, const Point& depot, const int capacity) {
    vector<Chromosome> population;
    for (int i = 0; i < POPULATION_SIZE; i++) {
        Chromosome chromosome;
        vector<Point> shuffled_clients = shuffle_clients(clients);
        chromosome.routes = create_routes(shuffled_clients, capacity);
        chromosome.fitness = score(chromosome.routes, depot);
        for (Route& route : chromosome.routes) {
            apply2Opt(route, depot);
        }
        population.push_back(chromosome);
    }
    return population;
}

vector<Chromosome> best_selection(const vector<Chromosome>& population, const int num_best) {
    vector<Chromosome> sorted_population = population;
    ranges::sort(sorted_population, [](const Chromosome& a, const Chromosome& b) {
        return a.fitness < b.fitness;
    });
    sorted_population.resize(num_best);
    return sorted_population;
}

void swapClientsBetweenRoutes(Chromosome& chromosome, const int capacity, const int routesCount) {
    int attempts = 0;
    uniform_int_distribution<> route_dis(0, routesCount - 1);
    while (attempts < MAX_ATTEMPTS) {
        attempts++;
        const int routeAIdx = route_dis(gen);
        int routeBIdx = route_dis(gen);
        while (routeBIdx == routeAIdx) {
            routeBIdx = route_dis(gen);
        }
        if (!chromosome.routes[routeAIdx].clients.empty() && !chromosome.routes[routeBIdx].clients.empty()) {
            uniform_int_distribution<> clientA_dis(0, chromosome.routes[routeAIdx].clientCount() - 1);
            uniform_int_distribution<> clientB_dis(0, chromosome.routes[routeBIdx].clientCount() - 1);
            const int clientAIdx = clientA_dis(gen);
            const int clientBIdx = clientB_dis(gen);

            const Point& clientA = chromosome.routes[routeAIdx].clients[clientAIdx];
            const Point& clientB = chromosome.routes[routeBIdx].clients[clientBIdx];

            if (chromosome.routes[routeAIdx].load - clientA.demand + clientB.demand > capacity) continue;
            if (chromosome.routes[routeBIdx].load - clientB.demand + clientA.demand > capacity) continue;

            chromosome.swapClientsBetweenRoutes(routeAIdx, clientAIdx, routeBIdx, clientBIdx);
            break;
        }
    }
}

void moveClientToRoute(Chromosome& chromosome, const int capacity, const int routesCount) {
    int attempts = 0;
    while (attempts < MAX_ATTEMPTS) {
        attempts++;
        uniform_int_distribution<> route_dis(0, routesCount - 1);
        const int fromRouteIdx = route_dis(gen);
        int toRouteIdx = route_dis(gen);
        while (toRouteIdx == fromRouteIdx) {
            toRouteIdx = route_dis(gen);
        }
        if (!chromosome.routes[fromRouteIdx].clients.empty()) {
            uniform_int_distribution<> client_dis(0, chromosome.routes[fromRouteIdx].clientCount() - 1);
            const int clientIdx = client_dis(gen);

            if (const Point& client = chromosome.routes[fromRouteIdx].clients[clientIdx]; chromosome.routes[toRouteIdx].load + client.demand > capacity) continue;

            chromosome.moveClientRoute(fromRouteIdx, toRouteIdx, clientIdx);
            break;
        }
    }
}

void mutate_chromosome(Chromosome& chromosome, const Point& depot, int capacity) {
    uniform_int_distribution<> dis(0, 2);
    const int mutation_type = dis(gen);
    if (const int routesCount = static_cast<int>(chromosome.routes.size()); mutation_type == 0 && routesCount >= 2) swapClientsBetweenRoutes(chromosome, capacity, routesCount);
    else if (mutation_type == 1 && routesCount >= 2) moveClientToRoute(chromosome, capacity, routesCount);
    else if (mutation_type == 2) {
        uniform_int_distribution<> route_dis(0, routesCount - 1);
        if (int fromRouteIdx = route_dis(gen); !chromosome.routes[fromRouteIdx].clients.empty()) {
            uniform_int_distribution<> client_dis(0, chromosome.routes[fromRouteIdx].clientCount() - 1);
            int clientIdx = client_dis(gen);
            chromosome.moveClientToNewRoute(fromRouteIdx, clientIdx);
        }
    }

    chromosome.deleteEmptyRoutes();
    for (Route& route : chromosome.routes) {
        apply2Opt(route, depot);
    }
    chromosome.fitness = score(chromosome.routes, depot);
}

// take the best, create they kids and mutate them to create the next generation
vector<Chromosome> generateNextGeneration(const vector<Chromosome>& parents, const Point& depot, int capacity) {
    vector<Chromosome> next_generation;
    for (const auto & parent : parents) {
        next_generation.push_back(parent);
    }

    while (next_generation.size() < POPULATION_SIZE) {
        for (const Chromosome& parent : parents) {
            Chromosome child = parent;
            mutate_chromosome(child, depot, capacity);
            next_generation.push_back(child);
            if (next_generation.size() >= POPULATION_SIZE) break;
        }
    }
    return next_generation;
}

string solve_vrp(const vector<Point>& clients, const Point& depot, int capacity) {
    const auto start_time = chrono::high_resolution_clock::now();
    const auto time_limit = chrono::milliseconds(9500);
    initDistanceCache(clients, depot);

    vector<Chromosome> population = initialize_population(clients, depot, capacity);

    int generation = 0;
    while (true) {
        auto current_time = chrono::high_resolution_clock::now();
        if (auto elapsed = chrono::duration_cast<chrono::milliseconds>(current_time - start_time); elapsed >= time_limit) break;
        population = generateNextGeneration(best_selection(population, BEST_SELECTION_SIZE), depot, capacity);
        generation++;
    }

    const auto best_it = ranges::min_element(population, [](const Chromosome& a, const Chromosome& b) {
        return a.fitness < b.fitness;
    });


    cerr << "Generations: " << generation << ", Best fitness: " << best_it->fitness << endl;

    return format_output(best_it->routes);
}

int main()
{
    int n;
    cin >> n; cin.ignore();
    int capacity;
    cin >> capacity; cin.ignore();
    vector<Point> clients;
    Point depot;
    cin >> depot.index >> depot.x >> depot.y >> depot.demand; cin.ignore();
    for (int i = 0; i < n-1; i++) {
        Point c;
        cin >> c.index >> c.x >> c.y >> c.demand; cin.ignore();
        clients.push_back(c);
    }
    string soluction = solve_vrp(clients, depot, capacity);
    cout << soluction << endl;

}
