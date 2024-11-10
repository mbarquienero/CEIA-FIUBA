import tracemalloc
import time
from hanoi_states import StatesHanoi, ProblemHanoi
from tree_hanoi import NodeHanoi
from search import (
    breadth_first_tree_search,
    breadth_first_graph_search,
    depth_first_tree_search,
    limited_depth_first_tree_search,
    a_star_search
)

def main():
    """
    Función principal que resuelve el problema de la Torre de Hanoi y genera los JSON para el simulador.
    """
    # Define el estado inicial y el estado objetivo del problema
    initial_state = StatesHanoi([5, 4, 3, 2, 1], [], [], max_disks=5)
    goal_state = StatesHanoi([], [], [5, 4, 3, 2, 1], max_disks=5)

    # Crea una instancia del problema de la Torre de Hanoi
    problem_hanoi = ProblemHanoi(initial=initial_state, goal=goal_state)

    # Menú para seleccionar el algoritmo
    print("Seleccione el algoritmo para resolver el problema de las Torres de Hanoi:")
    print("1. Búsqueda en anchura (sin visitados) <- Catedra IIA")
    print("2. Búsqueda en anchura (con visitados) <- Catedra IIA")
    print("3. Búsqueda en profundidad (con visitados) <- Equipo")
    print("4. Búsqueda en profundidad limitada (con visitados) <- Equipo")
    print("5. A* con colas de prioridad en coste heurística <- Equipo")

    choice = input("Seleccione el algoritmo: ")

    match choice:
        case "1":
            print("Ha seleccionado el algoritmo: Búsqueda en anchura (sin visitados) <- Catedra IIA")
        case "2":
            print("Ha seleccionado el algoritmo: Búsqueda en anchura (con visitados) <- Catedra IIA")
        case "3":
            print("Ha seleccionado el algoritmo: Búsqueda en profundidad (con visitados) <- Equipo")
        case "4":
            print("Ha seleccionado el algoritmo: Búsqueda en profundidad limitada (con visitados) <- Equipo")
        case "5":
            print("Ha seleccionado el algoritmo: A* con colas de prioridad en coste heurística <- Equipo")
        case _:
            print("Opción no válida. Por favor, seleccione un número entre 1 y 5.")

    num_limit = 0
    if choice == "4":
        num_limit = int(input("Selecciono DFS limitada - ingrese la restriccion de limite: "))    
    # Solicitar el número de ejecuciones
    num_executions = int(input("Ingrese el número de ejecuciones: "))

    # Inicializar variables para calcular el promedio de tiempo y memoria
    total_elapsed_time = 0
    total_memory_peak = 0

    for _ in range(num_executions):
        # Para medir tiempo consumido
        start_time = time.perf_counter()
        # Para medir memoria consumida (usamos el pico de memoria)
        tracemalloc.start()

        # Selecciona y ejecuta el algoritmo según la elección del usuario
        if choice == "1":
            last_node = breadth_first_tree_search(problem_hanoi)
        elif choice == "2":
            last_node = breadth_first_graph_search(problem_hanoi, display=True)
        elif choice == "3":
            last_node = depth_first_tree_search(problem_hanoi, display=True)
        elif choice == "4":
            last_node = limited_depth_first_tree_search(problem_hanoi, max_depth=num_limit, display=True)
        elif choice == "5":
            last_node = a_star_search(problem_hanoi, display=True)
        else:
            print("Opción no válida.")
            return

        # Calcular memoria y tiempo usados
        _, memory_peak = tracemalloc.get_traced_memory()
        memory_peak /= 1024 * 1024  # Convertir a MB
        tracemalloc.stop()

        end_time = time.perf_counter()
        elapsed_time = end_time - start_time

        # Sumar los valores actuales al total
        total_elapsed_time += elapsed_time
        total_memory_peak += memory_peak

    # Calcular promedios
    average_elapsed_time = total_elapsed_time / num_executions
    average_memory_peak = total_memory_peak / num_executions

    # Imprime las métricas medidas
    print(f"Tiempo promedio que demoró: {average_elapsed_time:.4f} [s]")
    print(f"Máxima memoria ocupada promedio: {average_memory_peak:.2f} [MB]")

    if isinstance(last_node, NodeHanoi):
        # Imprime la longitud del camino de la solución encontrada
        print(f'Longitud del camino de la solución: {last_node.state.accumulated_cost}')

        # Genera los JSON para el simulador
        last_node.generate_solution_for_simulator()

    else:
        print(last_node)
        print("No se encuentra solución")


# Sección de ejecución del programa
if __name__ == "__main__":
    main()