import sys
import heapq

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

    def __init__(self, filename, is_directed):
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
                self._matrix[a][b] = weight
                self._vector[a].append((b+1, weight))
                if is_directed == False:
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
    s = []
    #Lista que indica os vértices já visitados
    c = [False] * g.qtdVertices()

    for v in range(1, g.qtdVertices()+1):
        if c[v-1] == False:
            s = topological_sorting_visit(g, v, c, s)

    print(f'{"->".join(v for v in s)}')

def topological_sorting_visit(g, v, c, s):
    """
        Concatena em s a ordem topológica a partir de v
        considerando os vertices ainda não visitados
    """
    #Indica que o vérice foi visitado
    c[v-1] = True

    for u in g.vizinhos(v):
        if c[u[0]-1] == False:
            """
                Recursivamente visita e coloca os vizinhos 
                ainda não ordenados no topo da ordenação
            """
            s = topological_sorting_visit(g, u[0], c, s)

    #Coloca v no topo da ordenação
    return [g.rotulo(v)]+s

if __name__ == "__main__":
    # No terminal, execute:
    # python graph.py ARQUIVO_DE_ENTRADA
    grafoDirigido = Grafo(sys.argv[1],True)
    print("Ordenação Topológica")
    topological_sorting(grafoDirigido)
