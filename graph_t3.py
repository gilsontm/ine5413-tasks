import sys
from collections import deque


class Grafo:
    """
        Classe que representa um grafo ponderado.

        Utiliza tanto a representação de matriz de adjacência,
        quanto a representação de vetor de adjacência.

        Desse modo, o grafo ocupa mais espaço de armazenamento,
        mas todas as operações são O(1).

        IMPORTANTE: os vértices no arquivo de entrada devem ser
        indexados em 1, enquanto as estruturas internas da classe
        são indexadas em 0.
    """

    def __init__(self, filename, infinity=sys.maxsize):
        """
            Construtor da classe, recebe como parâmetro
            o nome do arquivo de entrada que contém as
            informações sobre o grafo.
        """
        self._labels = []
        self._INFINITY = infinity
        with open(filename, "r") as file:
            n = int(file.readline().split()[-1])
            self._nVertices = n
            for _ in range(n):
                line = file.readline()
                self._labels.append(line[line.index(" ")+1:-1])

            # Constrói uma matriz n por n
            self._matrix = [[self._INFINITY] * n for _ in range(n)]
            # Constrói uma lista (tamanho n) de listas vazias
            self._vector = [[] for _ in range(n)]

            # Lê a linha que contém '*edges' ou '*arcs'
            self._directed = file.readline() == "*arcs\n"
            # Lê todas as linhas subsequentes
            edges = file.readlines()
            self._nEdges = len(edges)
            for edge in edges:
                a, b, weight = edge.split()
                a = int(a)-1; b = int(b)-1; weight = float(weight)
                self._matrix[a][b] = weight
                self._vector[a].append((b+1, weight))
                if not self._directed:
                    self._matrix[b][a] = weight
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
            Quando o grafo é dirigido retorna N+(v)

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

    def transposto(self):
        """
            Cria e retorna um novo grafo que corresponde
            ao grafo inicial transposto.
        """
        graph = Grafo.__new__(Grafo)
        graph._nVertices = self._nVertices
        graph._nEdges = self._nEdges
        graph._INFINITY = self._INFINITY
        graph._labels = self._labels.copy()
        graph._directed = self._directed

        # transpõe a matriz de adjacências
        matrix = [[0] * self._nVertices for _ in range(self._nVertices)]
        for i in range(self._nVertices):
            for j in range(self._nVertices):
                matrix[j][i] = self._matrix[i][j]
        graph._matrix = matrix

        # transpõe o vetor de adjacências
        vector = [[] for _ in range(self._nVertices)]
        for v in range(self._nVertices):
            for (u, weight) in self._vector[v]:
                vector[u-1].append((v+1, weight))
        graph._vector = vector

        return graph

class RedeFluxo(Grafo):
    """
        Especialização da classe Grafo; representa uma rede de fluxo.
    """
    def __init__(self, filename):
        super(RedeFluxo, self).__init__(filename, infinity=0)

# fluxo máximo resultante 
def edmonds_karp(flow_network, s, t):
    n = flow_network.qtdVertices()
    flows = [[0] * n for _ in range(n)]
    while True:
        path = breadth_first_search_edmonds_karp(flow_network, s, t, flows)
        if path is None:
            break
        flow = sys.maxsize
        for i in range(len(path)-1):
            u = path[i]; v = path[i+1]
            flow = min(flow, flow_network.peso(u, v) - flows[u-1][v-1])
        for i in range(len(path)-1):
            u = path[i]; v = path[i+1]
            if flow_network.haAresta(u, v):
                flows[u-1][v-1] += flow
            else:
                flows[v-1][u-1] -= flow
    max_flow = 0
    for row in flows:
        max_flow += row[t-1]
    print(f"Fluxo máximo resultante: {max_flow}")

# busca em largura para o algoritmo de Edmonds-Karp
def breadth_first_search_edmonds_karp(flow_network, s, t, flows):
    n = flow_network.qtdVertices()
    C = [False] * n
    A = [None] * n
    C[s-1] = True
    queue = deque([s])
    while queue:
        u = queue.popleft()
        neighbours = flow_network.vizinhos(u)
        for (v, capacity) in neighbours:
            if not C[v-1] and capacity > flows[u-1][v-1]:
                C[v-1] = True
                A[v-1] = u
                if v == t:
                    p = [t]
                    w = t
                    while w != s:
                        w = A[w-1]
                        p.append(w)
                    p.reverse()
                    return p
                queue.append(v)
    return None

if __name__ == "__main__":
    filename = sys.argv[1]

    print("Edmonds-Karp")
    flow_network = RedeFluxo(filename)
    edmonds_karp(flow_network, 1, 5)

    print("\nHopcraft-Karp")
    # ...

    print("\nColoração de vértices")
    # ...
