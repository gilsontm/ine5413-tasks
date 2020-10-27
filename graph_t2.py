import sys
import heapq
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
                self._labels.append(file.readline().split()[-1])

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
                if self._directed == False:
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

#ordenacao topologica
def topological_sorting(g):
    #Lista ordenada
    if g._directed == False:
        print("Grafo não dirigido")
        return

    s = DLList()
    #Lista que indica os vértices já visitados
    c = [False] * g.qtdVertices()

    for v in range(1, g.qtdVertices()+1):
        if c[v-1] == False:
            topological_sorting_visit(g, v, c, s)

    print(f'{"->".join(v for v in s.to_list())}')

def topological_sorting_visit(g, v, c, s):
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
            topological_sorting_visit(g, u[0], c, s)

    #Coloca v no topo da ordenação
    s.prepend(g.rotulo(v))

#algoritmo de kruskal
def kruskal(graph):

    a = set()
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
            a |= {u,v}
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

class node:
    def __init__(self, value, next, prev):
        self._value = value
        self._next = next
        self._prev = prev
    def replace(self,dllist):
        (self._prev).next = dllist._begin
        (self._next).prev = dllist._end

class DLList:
    def __init__(self):
        """
            O primeiro valor coloquei um vazio pra não mudar
            o que já tinha implementado. No t1 essa DLList precisava
            de um valor pra ser inicializado
        """
        self._begin = node(None,None,None)
        self._end = None
        self._size = 0
    def append(self, value):
        if self._size == 0:
            self._end = node(value, None, self._begin)
            (self._begin)._next = self._end
        else:
            temp = self._end
            self._end = node(value, None, temp)
            temp._next = self._end
        self._size = self._size + 1
    def prepend(self, value):
        if self._size == 0:
            self._end = node(value, None, self._begin)
            (self._begin)._next = self._end
        else:
            temp = (self._begin)._next
            new = node(value, temp, self._begin)
            (self._begin)._next = new
            temp._prev = new
        self._size = self._size + 1
    def to_list(self):
        alist = []
        node = (self._begin)._next #pula o primeir q é o vazio
        while node != None:
            alist.append(node._value)
            node = node._next
        return alist


if __name__ == "__main__":
    # No terminal, execute:
    # python graph.py ARQUIVO_DE_ENTRADA
    grafo = Grafo(sys.argv[1])
    # print("Ordenação Topológica")
    # topological_sorting(grafo)
    kruskal(grafo)
