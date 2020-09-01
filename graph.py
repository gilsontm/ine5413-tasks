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
def busca(file,index):
    graph = Grafo(file)
    depth = 0
    to_visit = [index]
    visited = []
    while to_visit:
        print(f'{depth}: {",".join(str(v) for v in to_visit)}')
        neighbors = []
        for v in to_visit:
            visited += [v]
            u = [x[0] for x in graph.vizinhos(v)]
            for vertex in u:
                if vertex not in visited:
                    neighbors += [vertex]
                    visited += [vertex]
        to_visit = neighbors
        depth += 1

# Floyd-Warshall
def floyd_warshall(graph):
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

    #printa
    count = 1
    for v in range(graph.qtdVertices()):
        print(f'{count}: {",".join(str(u) for u in new_matrix[v])}')
        count += 1

    return new_matrix

# Dijkstra
def dijkstra(file, s):
    # Constantes
    DISTANCE, INDEX, VALID = range(3)
    graph = Grafo(file)
    # Inicializa vetor de distâncias
    distances = [0] + ([sys.maxsize] * (graph.qtdVertices() - 1))
    # Inicializa heap com campos [DISTANCE, INDEX, VALID]
    heap = [[distance, index, True] for index, distance in enumerate(distances)]
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
                for i in range(len(heap)):
                    if heap[i][INDEX] == neighbor[0]-1:
                        # Invalida o registro antigo deste vizinho na heap
                        heap[i][VALID] == False
                        # Insere-o novamente, com a distância atualizada
                        heapq.heappush(heap, [new_distance, neighbor[0], True])
                        break
    return distances

if __name__ == "__main__":
    # No terminal, execute:
    # python graph.py ARQUIVO_DE_ENTRADA
    # grafo = Grafo(sys.argv[1])
    # busca(sys.argv[1],61)
    # floyd_warshall(grafo)
    data = dijkstra(sys.argv[1], 1)