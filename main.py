from flask import Flask, jsonify, request
from flask_cors import CORS
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import googlemaps
import numpy as np
import config

apikey = config.api_key

app = Flask(__name__)
CORS(app, origins="http://localhost:3000", always_send=False)
app.config['CORS_HEADERS'] = 'Content-Type'

gmaps = googlemaps.Client(key=apikey)

def create_data_model(distance_matrix , num_vehicles):
    """Stores the data for the problem."""
    data = {}
    data["distance_matrix"] = distance_matrix
    data["num_vehicles"] = num_vehicles
    data["depot"] = 0
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    output = []
    dis = 0
    max_route_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        print(f"Route for vehicle {vehicle_id}:\n")
        plan_output = []
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output.append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        # plan_output += f"{manager.IndexToNode(index)}\n"
        # plan_output += f"Distance of the route: {route_distance} km\n"
        print(plan_output)
        dis += route_distance
        output.append(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print(f"Maximum of the route distances: {max_route_distance} km")
    return output , dis


def calculate_distance_matrix(warehouses, delivery_locations):
    distance_matrix = []
    for warehouse in warehouses:
        warehouse_distances = []
        for delivery_location in delivery_locations:
            warehouse_coords = (warehouse['lat'], warehouse['lng'])
            delivery_coords = (delivery_location['lat'], delivery_location['lng'])
            distance = gmaps.distance_matrix(warehouse_coords, delivery_coords, mode='driving')
            distance_meters = distance['rows'][0]['elements'][0]['distance']['value']

            # Convert distance to kilometers
            # distance_kms = distance_meters / 1000.0

            warehouse_distances.append(distance_meters)
        distance_matrix.append(warehouse_distances)

    return distance_matrix

def calculate_mat(arr) :
     n = len(arr)
     matrix = []
     for i in range(n) :
         curr = []
         for j in range(n) :
             if i== j :
                curr.append(0)
             elif j > i :
                 chord1 = (arr[i]['lat'] , arr[i]['lng'])
                 chord2 = (arr[j]['lat'] , arr[j]['lng'])
                 distance = gmaps.distance_matrix(chord1, chord2, mode='driving')
                 distance_meters = distance['rows'][0]['elements'][0]['distance']['value']

            # Convert distance to kilometers
                 distance_kms = int(distance_meters)
                 curr.append(distance_kms)
             else :
                curr.append(matrix[j][i])

         matrix.append(curr)
                

             
     return matrix




@app.route('/', methods=['POST'])
def handle_post():
    data = request.get_json()
    depots = data.get('markers')
    service_points = data.get('users')
    anspaths = []
    ansdis = 0
    distance_matrix = calculate_distance_matrix(depots , service_points)
    print(distance_matrix)
    assignments = np.argmin(distance_matrix, axis=0)
    path_matrix = []

    for i in range(len(depots)) :
        curr = []
        curr.append(depots[i])
        path_matrix.append(curr)


    for i in range(len(service_points)):
        path_matrix[assignments[i]].append(service_points[i])

    for i in range(len(path_matrix)) :
        if len(path_matrix[i]) >=2 :
            mat = calculate_mat(path_matrix[i])
            print(mat)
            num_vehicles = 1
            data = create_data_model(mat , num_vehicles)
            manager = pywrapcp.RoutingIndexManager( len(data["distance_matrix"]), data["num_vehicles"], data["depot"] )
            # Create Routing Model.
            routing = pywrapcp.RoutingModel(manager)

            # Create and register a transit callback.
            def distance_callback(from_index, to_index):
                """Returns the distance between the two nodes."""
                # Convert from routing variable Index to distance matrix NodeIndex.
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return data["distance_matrix"][from_node][to_node]

            transit_callback_index = routing.RegisterTransitCallback(distance_callback)

            # Define cost of each arc.
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

            # Add Distance constraint.
            dimension_name = "Distance"
            routing.AddDimension(
                transit_callback_index,
                0,  # no slack
                30000000,  # vehicle maximum travel distance
                True,  # start cumul to zero
                dimension_name,
            )
            distance_dimension = routing.GetDimensionOrDie(dimension_name)
            distance_dimension.SetGlobalSpanCostCoefficient(100)

            # Setting first solution heuristic.
            search_parameters = pywrapcp.DefaultRoutingSearchParameters()
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
            )

            # Solve the problem.
            solution = routing.SolveWithParameters(search_parameters)

            # Print solution on console.
            if solution:
                output , dis = print_solution(data, manager, routing, solution)
                ansdis += dis
                for j in range(len(output)) :
                    curr_path = []
                    for k in range(len(output[j])) :
                        curr_path.append(path_matrix[i][output[j][k]])
                    anspaths.append(curr_path)
            else:
                print("No solution found !")
    return jsonify({"path" :anspaths , "dist" : ansdis})
            


# main driver function
if __name__ == '__main__':
    app.run(debug=True, port=8080)
