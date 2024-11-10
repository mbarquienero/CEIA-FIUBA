from collections import deque
from hanoi_states import ProblemHanoi, StatesHanoi
import tree_hanoi
import hanoi_states
import copy
import heapq

# TP_1 - Ejercicio 4) Busqueda en Profundidad Iterativa con conjunto de visitados
def limited_depth_first_tree_search(problem: hanoi_states.ProblemHanoi, max_depth: int, display: bool = False):
    problem_copy = copy.deepcopy(problem)
    for curr_depth in range(max_depth):
        explored = set()  # Set para estados visitados <<==== Chequear VISITADOS !!!
        frontier = [(tree_hanoi.NodeHanoi(problem.initial), 0)]         
        while frontier:
            node, depth = frontier.pop()  # Extrae el nodo más reciente y su profundidad
            if node.state in explored:
                continue  # Salta si ya fue explorado

            explored.add(node.state)  # Marca como explorado

            if problem.goal_test(node.state):  # Chequea si es el objetivo
                if display:
                    print(len(explored), "caminos se expandieron y", len(frontier), "caminos quedaron en la frontera")
                return node

            # Expande el nodo si la profundidad actual es menor que el máximo permitido
            # Añade cada hijo con una profundidad aumentada en 1
            if depth < curr_depth:
                frontier.extend((child, depth + 1) for child in node.expand(problem)
                            if child.state not in explored and (child, depth) not in frontier)
        problem = copy.deepcopy(problem_copy)
    return None  # Retorna None si no encuentra solución

# TP_1 - Ejercicio 4) Busqueda en Profundidad con conjunto de visitados
def depth_first_tree_search(problem: hanoi_states.ProblemHanoi, display: bool = False):
    """
    Realiza una búsqueda en profundidad para encontrar una solución a un problema de Hanoi.
    Esta función no chequea si un estado se visitó, por lo que puede entrar en bucles infinitos
    muy fácilmente. No usarla con más de 3 discos.

    Parameters:
        problem (hanoi_states.ProblemHanoi): El problema de la Torre de Hanoi a resolver.

    Returns:
        tree_hanoi.NodeHanoi: El nodo que contiene la solución encontrada.
    """
    explored = set() 
    frontier = [tree_hanoi.NodeHanoi(problem.initial)]  # Creamos una pila LIFO con el nodo inicial
    while frontier:
        node = frontier.pop()  # Extraemos el último nodo de la pila
        explored.add(node.state)
        if problem.goal_test(node.state):  # Comprobamos si hemos alcanzado el estado objetivo
            if display:
                print(len(explored), "caminos se expandieron y", len(frontier), "caminos quedaron en la frontera")
            return node
        # frontier.extend(node.expand(problem))  # Agregamos a la pila todos los nodos sucesores del nodo actual
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and child not in frontier)

    return None

def breadth_first_tree_search(problem: hanoi_states.ProblemHanoi):
    """
    Realiza una búsqueda en anchura para encontrar una solución a un problema de Hanoi.
    Esta función no chequea si un estado se visito, por lo que puede entrar en Loop infinitos muy fácilmente. No
    usarla con más de 3 discos.

    Parameters:
        problem (hanoi_states.ProblemHanoi): El problema de la Torre de Hanoi a resolver.

    Returns:
        tree_hanoi.NodeHanoi: El nodo que contiene la solución encontrada.
    """
    frontier = deque([tree_hanoi.NodeHanoi(problem.initial)])  # Creamos una cola FIFO con el nodo inicial
    while frontier:
        node = frontier.popleft()  # Extraemos el primer nodo de la cola
        if problem.goal_test(node.state):  # Comprobamos si hemos alcanzado el estado objetivo
            return node
        frontier.extend(node.expand(problem))  # Agregamos a la cola todos los nodos sucesores del nodo actual

    return None


def breadth_first_graph_search(problem: hanoi_states.ProblemHanoi, display: bool = False):
    """
    Realiza una búsqueda en anchura para encontrar una solución a un problema de Hanoi. Pero ahora si recuerda si ya
    paso por un estado e ignora seguir buscando en ese nodo para evitar recursividad.

    Parameters:
        problem (hanoi_states.ProblemHanoi): El problema de la Torre de Hanoi a resolver.
        display (bool, optional): Muestra un mensaje de cuantos caminos se expandieron y cuantos quedaron sin expandir.
                                  Por defecto es False.

    Returns:
        tree_hanoi.NodeHanoi: El nodo que contiene la solución encontrada.
    """

    frontier = deque([tree_hanoi.NodeHanoi(problem.initial)])  # Creamos una cola FIFO con el nodo inicial

    explored = set()  # Este set nos permite ver si ya exploramos un estado para evitar repetir indefinidamente
    while frontier:
        node = frontier.popleft()  # Extraemos el primer nodo de la cola

        # Agregamos nodo al set. Esto evita guardar duplicados, porque set nunca tiene elementos repetidos, esto sirve
        # porque heredamos el método __eq__ en tree_hanoi.NodeHanoi de aima.Node
        explored.add(node.state)

        if problem.goal_test(node.state):  # Comprobamos si hemos alcanzado el estado objetivo
            if display:
                print(len(explored), "caminos se expandieron y", len(frontier), "caminos quedaron en la frontera")
            return node
        # Agregamos a la cola todos los nodos sucesores del nodo actual que no haya visitados
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and child not in frontier)

    return None

def a_star_search(problem: ProblemHanoi, display: bool = False):
    frontier = [(0, tree_hanoi.NodeHanoi(problem.initial))]
    heapq.heapify(frontier)
    explored = set()  # Conjunto para almacenar estados ya evaluados
    tree = {}  # Diccionario para almacenar el árbol de búsqueda
    costs = {}  # Diccionario para almacenar los costos `g(n)` y `f(n)` de cada nodo
    initial_node = frontier[0][1]  # Guardamos el nodo inicial
    costs[initial_node] = {'g': 0, 'f': hanoi_heuristic(initial_node.state, problem.goal)}

    expanded_nodes_count = 0  # Contador de nodos expandidos

    while frontier:
        _, node = heapq.heappop(frontier)

        # Se verifica si se  alcanzo el objetivo
        if problem.goal_test(node.state):
            if display:
                """
                Si se desea ver por pantalla los caminos que fue tomando el algoritmo se debe descomentar las dos siguientes lineas
                solution_path = get_solution_path(node)
                print_tree(tree, initial_node, solution_path, costs)
                """
                # Obtén el camino real hacia la solución 
                #solution_path = get_solution_path(node)
                #print_tree(tree, initial_node, solution_path, costs)
                return node

        # Se expande el nodo si su estado no ha sido evaluado
        if node.state not in explored:
            explored.add(node.state)  # Marcar este estado como evaluado
            expanded_nodes_count += 1  # Incrementar el contador de expansiones

            # Si el nodo aún no está en el árbol, lo agregamos
            if node not in tree:
                tree[node] = []  # Inicializamos la lista de hijos para el nodo actual en el árbol

            # Expandir y registrar los nodos hijos
            for action in problem.actions(node.state):
                child = node.child_node(problem, action)
                g_cost = child.path_cost
                f_cost = g_cost + hanoi_heuristic(child.state, problem.goal)
                
                # Solo añadimos el hijo si su estado no ha sido explorado aún
                if child.state not in explored:
                    costs[child] = {'g': g_cost, 'f': f_cost}  # Guardar los costos `g(n)` y `f(n)` para el hijo
                    heapq.heappush(frontier, (f_cost, child))
                    tree[node].append((action, child))  # Registramos el hijo en el árbol

    print("No se encuentra solución")
    print(f"Total de caminos expandidos: {expanded_nodes_count}")
    return None

def get_solution_path(node):
    """
    Retrocede desde el nodo solución hasta el nodo raíz para obtener el camino completo de la solución.
    """
    path = []
    while node:
        path.append(node)
        node = node.parent
    return list(reversed(path))

def print_tree(tree, initial_node, solution_path, costs):
    """
    Imprime el árbol de búsqueda completo y marca únicamente el camino elegido, incluyendo los costos.

    Args:
        tree (dict): Diccionario que almacena las relaciones de nodo padre-hijo.
        initial_node (tree_hanoi.NodeHanoi): El nodo inicial del árbol.
        solution_path (list): Lista de nodos que representan el camino exacto tomado hasta la solución.
        costs (dict): Diccionario que almacena los costos `g(n)` y `f(n)` de cada nodo.
    """
    print("Árbol de búsqueda completo:")
    print(f">>>>>>>> Nodo raíz: {initial_node.state}, Costo g(n): {costs[initial_node]['g']}, Costo total f(n): {costs[initial_node]['f']}")
    for parent, children in tree.items():
        # Marca el nodo padre solo si está en el camino de la solución
        prefix = ">>>>>>>> " if parent in solution_path else ""
        print(f"{prefix}└─ Nodo padre: {parent.state}, Costo g(n): {costs[parent]['g']}, Costo total f(n): {costs[parent]['f']}")
        for action, child in children:
            # Los nodos hijos no se marcan
            print(f"    └─ Nodo hijo: {child.state}, Costo g(n): {costs[child]['g']}, Costo total f(n): {costs[child]['f']}")
    
    # Imprime el nodo final si está en el camino de solución
    if solution_path[-1] not in tree:
        final_node = solution_path[-1]
        print(f">>>>>>>> Nodo objetivo: {final_node.state}, Costo g(n): {costs[final_node]['g']}, Costo total f(n): {costs[final_node]['f']}")

def hanoi_heuristic(state: StatesHanoi, goal: StatesHanoi) -> int:
    """
    Heurística para el problema de las Torres de Hanoi.
    Calcula cuántos discos están fuera de la varilla de destino.

    Args:
        state (StatesHanoi): Estado actual.
        goal (StatesHanoi): Estado objetivo.

    Returns:
        int: Número de discos fuera de la varilla de destino.
    """
    current_state = state.get_state()
    
    # Cuenta cuántos discos no están en la varilla de destino
    disks_out_of_place = sum(len(rod) for rod in current_state[:-1]) 
    
    return disks_out_of_place