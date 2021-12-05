import datetime
import pprint

import googlemaps
import pytz

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import secrets
import inputs


def get_departure_time(hour, minute):
    today = datetime.date.today()  # current local date
    departure_time = datetime.datetime(year=today.year, month=today.month, day=today.day, hour=hour, minute=minute)
    return pytz.timezone('US/Eastern').localize(departure_time)


def get_data(addresses, departure_time):
    gmaps = googlemaps.Client(key=secrets.googlemaps_api_key)
    # set origins / destinations
    origins = addresses
    destinations = addresses
    # get distance matrix
    return gmaps.distance_matrix(origins=origins, destinations=destinations, mode='driving',
                                 traffic_model='best_guess', departure_time=departure_time)


def build_cost_matrix(response, cost_measure):
    cost_matrix = []
    for row in response['rows']:
        row_list = [row['elements'][j][cost_measure]['value'] for j in range(len(row['elements']))]
        cost_matrix.append(row_list)
    return cost_matrix


def get_distance_matrix(addresses, depature_time):
    api_response = get_data(addresses, depature_time)
    # Instantiate the data problem.
    return build_cost_matrix(api_response, 'duration_in_traffic')


def create_data_model(distance_matrix, start, end):
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = 1
    data['starts'] = [start]
    data['ends'] = [end]
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Time of the route: {}s\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)


def get_solution(addresses, depart_time, start, end):
    # Instantiate the data problem.
    distance_matrix = get_distance_matrix(addresses, depart_time)
    pprint.pprint(distance_matrix)
    data = create_data_model(distance_matrix, start, end)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['starts'],
                                           data['ends'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print('No solution found!')


if __name__ == '__main__':
    # Inputs to the program are addresses, leave/return time, and the start/end indices
    # addresses is an array of strings complying with google distance matrix api format
    # leave and return time are datetime objects that specify the departure time. Must be in the future
    # start/end index specify which location in addresses the route should start and end
    # TODO: add command line args for all this. For now this is fine though
    pprint.pprint(inputs.addresses)
    # Print solution on console.
    print('Solution on the way there:')
    get_solution(inputs.addresses, inputs.leave_time, inputs.start_index, inputs.end_index)
    print('Solution on the way back:')
    get_solution(inputs.addresses, inputs.return_time, inputs.end_index, inputs.start_index)
