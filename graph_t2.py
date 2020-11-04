import sys
from itertools import product


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
                line = file.readline()
                self._labels.append(line[line.index(" ")+1:-1])

            # Constrói uma matriz n por n
            self._matrix = [[self._INFINITY] * n for _ in range(n)]
            # Constrói uma lista (tamanho n) de listas vazias
            self._vector = [[] for _ in range(n)]

            # Pula a linha que contém '*edges'
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

# componentes fortemente conexas
def strongly_connected_components(graph):
    C, T, A, F = depth_first_search(graph)
    graph_transposed = graph.transposto()
    Ct, Tt, At, Ft = depth_first_search(graph_transposed, ordered_by=F)

    # imprimir componentes
    components = {}
    for i in range(len(At)):
        if At[i] is None:
            if (i+1) not in components:
                components[i+1] = [i+1]
        else:
            index = i
            while At[index] is not None:
                index = At[index] - 1
            if (index+1) not in components:
                components[index+1] = [index+1]
            components[index+1].append(i+1)
    for c in components:
        components[c].sort()
        print(",".join(map(str, components[c])))
    return At

def depth_first_search(graph, ordered_by=None):
    n = graph.qtdVertices()
    C = [False] * n
    T = [sys.maxsize] * n
    F = [sys.maxsize] * n
    A = [None] * n
    tempo = 0

    # suporte ao algoritmo DFS adaptado, usado pelo algoritmo 15 da apostila
    if ordered_by is not None:
        vertices = [(ordered_by[i-1], i) for i in range(1, n+1)]
        vertices.sort(reverse=True)
        vertices = [u for (_, u) in vertices]
    else:
        vertices = range(1, n+1)

    for u in vertices:
        if not C[u-1]:
            tempo = depth_first_search_visit(graph, u, C, T, A, F, tempo)
    return (C, T, A, F)

def depth_first_search_visit(graph, u, C, T, A, F, tempo):
    C[u-1] = True
    tempo += 1
    T[u-1] = tempo
    vizinhos = graph.vizinhos(u)
    for (v, _) in vizinhos:
        if not C[v-1]:
            A[v-1] = u
            tempo = depth_first_search_visit(graph, v, C, T, A, F, tempo)
    tempo += 1
    F[u-1] = tempo
    return tempo

#ordenacao topologica
def topological_sorting(g):
    #Lista ordenada
    if g._directed == False:
        print("Grafo não dirigido")
        return

    s = [None] * g.qtdVertices()
    i = g.qtdVertices()-1
    #Lista que indica os vértices já visitados
    c = [False] * g.qtdVertices()

    for v in range(1, g.qtdVertices()+1):
        if c[v-1] == False:
            i = topological_sorting_visit(g, v, c, s, i)

    print(f'{"->".join(v for v in s)}')

def topological_sorting_visit(g, v, c, s, i):
    """
        Concatena em s a ordem topológica a partir de v
        considerando os vertices ainda não visitados
    """
    #Indica que o vértice foi visitado
    c[v-1] = True

    for u in g.vizinhos(v):
        if c[u[0]-1] == False:
            """
                Recursivamente visita e coloca os vizinhos
                ainda não ordenados no topo da ordenação
            """
            i = topological_sorting_visit(g, u[0], c, s, i)

    #Coloca v no topo da ordenação
    s[i] = g.rotulo(v)

    i = i - 1

    return i

#algoritmo de kruskal
def kruskal(graph):

    s = {}
    #para cada vértice do grafo cria uma chave no dicionário
    for v in range(1, graph.qtdVertices()+1):
        s[v] = {v}

    #cria uma lista com todas as arestas do grafo
    e = [(a,b) for a,b in product(range(1, graph.qtdVertices()+1),repeat=2) if graph.haAresta(a,b)]
    #deixa a lista ordenada pela ordem crescente do peso das arestas
    e = sorted(e,key= lambda k: graph.peso(k[0],k[1]))

    replacement = []
    for u,v in e:
        if s[u] != s[v]:
            x = (s[u] | s[v])
            for y in x:
                s[y] = x
            replacement += [(u,v)]

    #prints
    to_print = []
    result = 0
    for x,y in replacement:
        result += graph.peso(x,y)
        to_print += [f'{x}-{y}']

    print(result)
    print(', '.join(to_print))


if __name__ == "__main__":
    # No terminal, execute:
    # python graph.py ARQUIVO_DE_ENTRADA
    grafo = Grafo(sys.argv[1])

    print("Componentes fortemente conexas")
    strongly_connected_components(grafo)

    print("\nOrdenação topológica")
    topological_sorting(grafo)

    print("\nKruskal")
    kruskal(grafo)
