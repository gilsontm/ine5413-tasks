import sys
import heapq

class Grafo:
    """
        Classe que representa um grafo ponderado não-dirigido.

        Utiliza tanto a representação de matriz de adjacência,
        quanto a representação de vetor de adjacência.

        Desse modo, o grafo ocupa mais espaço de armazenamento,
        mas todas as operações são O(1).

        IMPORTANTE: os vértices no arquivo de entrada devem ser
        indexados em 1, enquanto as estruturas internas da classe
        são indexadas em 0.
    """

    def __init__(self, filename):
        """
            Construtor da classe, recebe como parâmetro
            o nome do arquivo de entrada que contém as
            informações sobre o grafo.
        """
        self._labels = []
        self._INFINITY = sys.maxsize
        with open(filename, "r") as file:
            n = int(file.readline().split()[-1])
            self._nVertices = n
            for _ in range(n):
                self._labels.append(file.readline().split()[-1])

            # Constrói uma matriz n por n
            self._matrix = [[self._INFINITY] * n for _ in range(n)]
            # Constrói uma lista (tamanho n) de listas vazias
            self._vector = [[] for _ in range(n)]

            # Pula a linha que contém '*edges'
            file.readline()
            # Lê todas as linhas subsequentes
            edges = file.readlines()
            self._nEdges = len(edges)
            for edge in edges:
                a, b, weight = edge.split()
                a = int(a)-1; b = int(b)-1; weight = float(weight)
                self._matrix[a][b] = self._matrix[b][a] = weight
                self._vector[a].append((b+1, weight))
                self._vector[b].append((a+1, weight))

    def qtdVertices(self):
        """ Retorna o número de vértices """
        return self._nVertices

    def qtdArestas(self):
        """ Retorna o número de arestas """
        return self._nEdges

    def grau(self, v):
        """ Retorna o número de vizinhos do vértice v """
        return len(self._vector[v-1])

    def rotulo(self, v):
        """ Retorna o rótulo (label) do vértice v """
        return self._labels[v-1]

    def vizinhos(self, v):
        """
            Retorna uma lista com os vizinhos do vértice v

            IMPORTANTE: como o grafo é ponderado, a lista
            retornada contém tuplas (vértice, peso).

            IMPORTANTE: os índices dos vértices nas tuplas
            (vértice, peso) são indexados em 1.
        """
        return self._vector[v-1]

    def haAresta(self, u, v):
        """
            Retorna verdadeiro caso haja uma aresta entre
            os vértices u e v, e falso caso contrário
        """
        return self._matrix[u-1][v-1] != self._INFINITY

    def peso(self, u, v):
        """
            Retorna o peso da aresta u-v, se existir;
            caso contrário, retorna infinito positivo.
        """
        return self._matrix[u-1][v-1]

# Busca em largura
def busca(graph, index):
    """
        / graph: Grafo de entrada
        / index: índice do vértice de entrada
    """
    depth = 0
    to_visit = [index]
    visited = [index]
    while to_visit:
        print(f'{depth}: {",".join(str(v) for v in to_visit)}')
        neighbors = []
        for v in to_visit:
            u = [x[0] for x in graph.vizinhos(v)]
            for vertex in u:
                if vertex not in visited:
                    neighbors += [vertex]
                    visited += [vertex]
        to_visit = neighbors
        depth += 1

# Floyd-Warshall
def floyd_warshall(graph):
    """
        / graph: Grafo de entrada
    """
    matrix = []
    for vertex in range(graph.qtdVertices()):
        line = []
        for intern in range(graph.qtdVertices()):
            if vertex == intern:
                line.append(0)
            else:
                line.append(graph.peso(vertex,intern))
        matrix.append(line)

    for k in range(graph.qtdVertices()):
        new_matrix = []
        for u in range(graph.qtdVertices()):
            new_line = []
            for v in range(graph.qtdVertices()):
                new_line.append(min(matrix[u][v],matrix[u][k]+matrix[k][v]))
            new_matrix.append(new_line)
        matrix = new_matrix

    # Print
    count = 1
    for v in range(graph.qtdVertices()):
        print(f'{count}: {",".join(str(u) for u in matrix[v])}')
        count += 1

    return matrix

# Dijkstra
def dijkstra(graph, s):
    """
        / graph: Grafo de entrada
        / s: índice do vértice de entrada
    """
    # Constantes
    DISTANCE, INDEX, VALID = range(3)
    # Inicializa vetor de distâncias
    distances = [sys.maxsize] * graph.qtdVertices()
    distances[s-1] = 0
    # Inicializa o vetor de ancestrais
    ancestors = [None] * graph.qtdVertices()
    ancestors[s-1] = s
    # Inicializa heap com campos [DISTANCE, INDEX, VALID]
    heap = [[0, s, True]] + [[distances[i], i, True] for i in range(len(distances)) if i != (s-1)]
    while heap:
        # Remove da heap o elemento de menor distância
        vertex = heapq.heappop(heap)
        # Se o elemento tiver sido marcado como inválido, desconsidere-o
        if not vertex[VALID]: continue
        # Para cada vizinho do elemento selecionado...
        neighbors = graph.vizinhos(vertex[INDEX])
        for neighbor in neighbors:
            new_distance = distances[vertex[INDEX]-1] + graph.peso(vertex[INDEX], neighbor[0])
            # Se a distância passando pelo elemento selecionado for melhor que a anterior...
            if distances[neighbor[0]-1] > new_distance:
                # Atualiza a distância no vetor de distâncias
                distances[neighbor[0]-1] = new_distance
                ancestors[neighbor[0]-1] = vertex[INDEX]
                for i in range(len(heap)):
                    if heap[i][INDEX] == neighbor[0]-1:
                        # Invalida o registro antigo deste vizinho na heap
                        heap[i][VALID] = False
                        # Insere-o novamente, com a distância atualizada
                        heapq.heappush(heap, [new_distance, neighbor[0], True])
                        break

    # Print
    for i in range(len(distances)):
        path = []
        if ancestors[i] is not None:
            path.append(str(i+1))
            if (i+1) != s:
                index = i
                while True:
                    path.append(str(ancestors[index]))
                    if ancestors[index] == s: break
                    index = ancestors[index] - 1
            path.reverse()
        print(f"{i+1}: {','.join(path)}; d={distances[i]}")

    return distances, ancestors

# Ciclo Euleriano
def eulerian_circuit(g):
    """
        / g: Grafo de entrada
    """
    v = 1

    c = [[True] * g.qtdVertices() for _ in range(g.qtdVertices())]


    for i in range(0,g.qtdVertices()):
        for j in range(0,g.qtdVertices()):
            if g.haAresta(i+1,j+1):
                c[i][j] = False

    (is_circuit,circuit) = eulerian_circuit_search(g, v, c)

    if is_circuit == False:
        print(0)
    else:
        if g.qtdArestas() != (len(circuit)-1):
            print(0)
            return
        # Se é um ciclo e todas as arestas estão no ciclo
        print(1)
        print(f'{",".join(str(v) for v in circuit)}')

# Recursão Ciclo Euleriano
def eulerian_circuit_search(g, v, c):
    circuit = [v]
    v = v - 1
    v0 = v
    # Adiciona uma trilha ao ciclo até que...
    while True:
        i = 0
        u = -1
        for a in c[v]:
            if a == False and g.haAresta(v+1,i+1):
                u = i
                break
            i = i + 1
        # Ou o vértice no fim da trilha não tenha arestas ainda não visitadas;
        if u == -1:
            return (False, None)
        else: # (Quando um vértice é visitado...)
            c[v][u] = True
            c[u][v] = True
            v = u
            # (Adiciona o vértice na trilha)
            circuit.append(v+1)
        # Ou o vértice de partida é o mesmo que o atual (fechando um subciclo).
        if v == v0:
            break

    # Se há um subciclo

    # Para cada vértice desse subciclo...
    for j in range(0, len(circuit)):
        x = circuit[j]
        vizinhos = g.vizinhos(x)
        for a in vizinhos:
            if c[x-1][a[0]-1] == False:
            # que tenha alguma aresta não visitada...
                # Busque um novo subciclo.
                (is_circuit, sub_circuit) = eulerian_circuit_search(g, x, c)
                if is_circuit == False:
                    return (False, None)
                # Se encontrado, adiciona o novo subciclo no lugar do vértice.
                circuit[j:j+1] = sub_circuit
    # Retorne o ciclo.
    return (True, circuit)

if __name__ == "__main__":
    # No terminal, execute:
    # python graph.py ARQUIVO_DE_ENTRADA
    grafo = Grafo(sys.argv[1])
    print("Busca em largura")
    busca(grafo, 1)
    print("\nFloyd-Washall")
    floyd_warshall(grafo)
    print("\nDijkstra")
    dijkstra(grafo, 1)
    print("\nCiclo euleriano")
    eulerian_circuit(grafo)
