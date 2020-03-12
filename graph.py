
class Graph:
    def __init__(self, n, vertices, edges):
        pass

    @staticmethod
    def build_graph(filename):
        with open(filename, "r") as file:

            n = file.read().split()[-1]


if __name__ == "__main__":
    Graph.build_graph(input())
