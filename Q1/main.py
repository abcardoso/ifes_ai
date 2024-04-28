
import random
import time 
import heapq

from collections import deque

from sympy import true
from viewer import MazeViewer
from math import inf, sqrt

## Ok

def gera_labirinto(n_linhas, n_colunas, inicio, goal):
    # cria labirinto vazio
    labirinto = [[0] * n_colunas for _ in range(n_linhas)]

    # adiciona celulas ocupadas em locais aleatorios de
    # forma que 40% do labirinto esteja ocupado
    numero_de_obstaculos = int(0.40 * n_linhas * n_colunas)
    for _ in range(numero_de_obstaculos):
        linha = random.randint(0, n_linhas-1)
        coluna = random.randint(0, n_colunas-1)
        labirinto[linha][coluna] = 1

    # remove eventuais obstaculos adicionados na posicao
    # inicial e no goal
    labirinto[inicio.y][inicio.x] = 0
    labirinto[goal.y][goal.x] = 0

    return labirinto


class Celula:
    def __init__(self, y, x, anterior):
        self.y = y
        self.x = x
        self.anterior = anterior
        self.distancia = 0
        self.custo = 0
    
    # Define menor-que para heapq 
    def __lt__(self, other):
        return (self.y, self.x) < (other.y, other.x)


def distancia(celula_1, celula_2):
    dx = celula_1.x - celula_2.x
    dy = celula_1.y - celula_2.y
    return sqrt(dx ** 2 + dy ** 2)


def esta_contido(lista, celula):
    for elemento in lista:
        if (elemento.y == celula.y) and (elemento.x == celula.x):
            return True
    return False


def custo_caminho(caminho):
    if len(caminho) == 0:
        return inf

    custo_total = 0
    for i in range(1, len(caminho)):
        custo_total += distancia(caminho[i].anterior, caminho[i])

    return custo_total


def obtem_caminho(goal):
    caminho = []

    celula_atual = goal
    while celula_atual is not None:
        caminho.append(celula_atual)
        celula_atual = celula_atual.anterior

    # o caminho foi gerado do final para o
    # comeco, entao precisamos inverter.
    caminho.reverse()

    return caminho


def celulas_vizinhas_livres(celula_atual, labirinto):
    # generate vizinhos of the current state
    vizinhos = [
        Celula(y=celula_atual.y-1, x=celula_atual.x-1, anterior=celula_atual),
        Celula(y=celula_atual.y+0, x=celula_atual.x-1, anterior=celula_atual),
        Celula(y=celula_atual.y+1, x=celula_atual.x-1, anterior=celula_atual),
        Celula(y=celula_atual.y-1, x=celula_atual.x+0, anterior=celula_atual),
        Celula(y=celula_atual.y+1, x=celula_atual.x+0, anterior=celula_atual),
        Celula(y=celula_atual.y+1, x=celula_atual.x+1, anterior=celula_atual),
        Celula(y=celula_atual.y+0, x=celula_atual.x+1, anterior=celula_atual),
        Celula(y=celula_atual.y-1, x=celula_atual.x+1, anterior=celula_atual),
    ]

    # seleciona as celulas livres
    vizinhos_livres = []
    for v in vizinhos:
        # verifica se a celula esta dentro dos limites do labirinto
        if (v.y < 0) or (v.x < 0) or (v.y >= len(labirinto)) or (v.x >= len(labirinto[0])):
            continue
        # verifica se a celula esta livre de obstaculos.
        if labirinto[v.y][v.x] == 0:
            vizinhos_livres.append(v)

    return vizinhos_livres


def breadth_first_search(labirinto, inicio, goal, viewer):
    # nos gerados e que podem ser expandidos (vermelhos)
    fronteira = deque()
    # nos ja expandidos (amarelos)
    expandidos = set()

    # adiciona o no inicial na fronteira
    fronteira.append(inicio)

    # variavel para armazenar o goal quando ele for encontrado.
    goal_encontrado = None

    # Repete enquanto nos nao encontramos o goal e ainda
    # existem para serem expandidos na fronteira. Se
    # acabarem os nos da fronteira antes do goal ser encontrado,
    # entao ele nao eh alcancavel.
    while (len(fronteira) > 0) and (goal_encontrado is None):

        # seleciona o no mais antigo para ser expandido
        no_atual = fronteira.popleft()

        # busca os vizinhos do no
        vizinhos = celulas_vizinhas_livres(no_atual, labirinto)

        # para cada vizinho verifica se eh o goal e adiciona na
        # fronteira se ainda nao foi expandido e nao esta na fronteira
        for v in vizinhos:
            if v.y == goal.y and v.x == goal.x:
                goal_encontrado = v
                # encerra o loop interno
                break
            else:
                if (not esta_contido(expandidos, v)) and (not esta_contido(fronteira, v)):
                    fronteira.append(v)

        expandidos.add(no_atual)

        if viewer:
            viewer.update(generated=fronteira, expanded=expandidos)
            #viewer.pause()


    caminho = obtem_caminho(goal_encontrado)
    custo   = custo_caminho(caminho)

    return caminho, custo, expandidos


def depth_first_search(labirinto, inicio, goal, viewer):
    # nós gerados e que podem ser expandidos, já inicializado apendando com [inicio]
    fronteira = deque([inicio])
    # nós já expandidos
    expandidos = set()

    # Goal
    goal_encontrado = None

    while (len(fronteira) > 0) and (goal_encontrado is None):
        # Seleciona o nó adicionado mais recentemente para expandir (comportamento da pilha)
        no_atual = fronteira.pop()  # Usando pop para remover da direita emulando pilha

        # verifica se atingimos o Goal
        if no_atual.y == goal.y and no_atual.x == goal.x:
            goal_encontrado = no_atual
            break

        # se ainda nao foi expandido
        if not esta_contido(expandidos, no_atual):
            # marca o nó atual como expandido
            expandidos.add(no_atual)

            # celulas vizinhas livres
            vizinhos = celulas_vizinhas_livres(no_atual, labirinto)

            # Adicione vizinhos não visitados e que não estão na fronteira a pilha
            for v in vizinhos:
                if not esta_contido(expandidos, v) and not esta_contido(fronteira, v):
                    fronteira.append(v)  # usando append

            if viewer:
                # Atualiza o viewer 
                viewer.update(generated=fronteira, expanded=expandidos)
                #viewer.pause()

    # retrona as infos
    caminho = obtem_caminho(goal_encontrado)
    custo = custo_caminho(caminho)

    return caminho, custo, expandidos


""" def a_star_search(labirinto, inicio, goal, viewer):
    # nos gerados e que podem ser expandidos (vermelhos)
    fronteira = []
    # nos ja expandidos (amarelos)
    expandidos = []

    # adiciona o no inicial na fronteira
    fronteira.append(inicio)

    # variavel para armazenar o goal quando ele for encontrado.
    goal_encontrado = None

    # Repete enquanto nos nao encontramos o goal e ainda
    # existem para serem expandidos na fronteira. Se
    # acabarem os nos da fronteira antes do goal ser encontrado,
    # entao ele nao eh alcancavel.
    while (len(fronteira) > 0) and (goal_encontrado is None):
        fronteira = sorted(fronteira, key=lambda x: x.distancia)
        # seleciona o no mais antigo para ser expandido
        no_atual = fronteira.pop(0)
        expandidos.append(no_atual)

        # busca os vizinhos do no
        vizinhos = celulas_vizinhas_livres(no_atual, labirinto)

        # para cada vizinho verifica se eh o goal e adiciona na
        # fronteira se ainda nao foi expandido e nao esta na fronteira
        for v in vizinhos:

            if v.y == goal.y and v.x == goal.x:
                if expandidos:
                    cost = custo_caminhoAS(expandidos[0], expandidos[1:])
                    if cost != inf:
                        goal_encontrado = v
                        break
                else:
                    goal_encontrado = v
                    break

            if v not in expandidos:
                distance = distancia(v, goal)
                v.distance = distance
                fronteira.append(v)

        if no_atual not in expandidos:
            expandidos.append(no_atual)

        if viewer:
            viewer.update(generated=fronteira,
                        expandidos=expandidos)
            #viewer.pause()


    caminho = obtem_caminho(goal_encontrado)
    custo   = custo_caminhoAS(caminho)

    return caminho, custo, expandidos


def custo_caminhoAS(caminho):
    if len(caminho) == 0:
        return inf

    custo_total = 0
    for i in range(1, len(caminho)):
        custo_total += distancia(caminho[i].anterior, caminho[i]) + heuristic(caminho[i].anterior, caminho[i])

    return custo_total

def obtem_caminhoAS(goal, origin):
    #construi o caminho do goal para a origem. 
    caminho = []
    step = goal
    while step is not None:
        caminho.append(step)
        step = origin.get(step)
    caminho.reverse()  #caminho gerado do goal para a origem, então precisamos inverter
    return caminho """


def a_star_search(labirinto, inicio, goal, viewer):
    # Configuração inicial: cria uma lista de prioridades para a fronteira de busca
    open_set = []
    heapq.heappush(open_set, (heuristic(inicio, goal), inicio))
    in_open_set = {inicio}
    origin = {inicio: None}
    g_score = {inicio: 0}  # Custo do caminho do início até o nó atual
    f_score = {inicio: heuristic(inicio, goal)}  # Custo estimado do início até o objetivo passando pelo nó

    expandidos = set()  # Conjunto de nós já expandidos

    while open_set:
        current_f, current = heapq.heappop(open_set)
        in_open_set.remove(current)

        # Atualiza a janela, se fornecido
        if viewer:
            # Converte os itens da fila de prioridade em lista para visualização
            fronteira_vis = [node for _, node in open_set]
            viewer.update(generated=fronteira_vis, expanded=list(expandidos))
            #viewer.pause()

        # Verifica se o nó atual é o objetivo
        if current.y == goal.y and current.x == goal.x:
            path = reconstruct_path(current, origin)
            return path, custo_caminho(path), expandidos

        expandidos.add(current)

        # Explora os vizinhos do nó atual
        for vizinho in celulas_vizinhas_livres(current, labirinto):
            if vizinho in expandidos:
                continue

            # Calcula o custo provisório g do caminho até o vizinho
            tentative_g_score = g_score[current] + distancia(current, vizinho)

            # Atualiza o custo do vizinho se for menor que o registrado
            if vizinho not in g_score or tentative_g_score < g_score[vizinho]:
                origin[vizinho] = current
                g_score[vizinho] = tentative_g_score
                f_score[vizinho] = tentative_g_score + heuristic(vizinho, goal)
                if vizinho not in in_open_set:
                    heapq.heappush(open_set, (f_score[vizinho], vizinho))
                    in_open_set.add(vizinho)

    return [], inf, expandidos

def reconstruct_path(current, origin):
    path = []
    while current in origin:
        path.append(current)
        current = origin[current]
    path.reverse()  # Inverte
    return path


def heuristic(celula_1, celula_2):
    # Heurística de distância de Manhattan
    return abs(celula_1.x - celula_2.x) + abs(celula_1.y - celula_2.y)


def main():
    for _ in range(10):
        SEED = 21  # coloque None no lugar do 42 para deixar aleatorio
        #random.seed(SEED)
        N_LINHAS  = 10 #10
        N_COLUNAS = 20 #20
        RENDER = true
        INICIO = Celula(y=0, x=0, anterior=None)
        GOAL   = Celula(y=N_LINHAS-1, x=N_COLUNAS-1, anterior=None)


        """
        O labirinto sera representado por uma matriz (lista de listas)
        em que uma posicao tem 0 se ela eh livre e 1 se ela esta ocupada.
        """
        labirinto = gera_labirinto(N_LINHAS, N_COLUNAS, INICIO, GOAL)

        if RENDER:
            viewer = MazeViewer(labirinto, INICIO, GOAL,
                                step_time_miliseconds=1, zoom=10)
        else:
            viewer = None #norender para não gastar tempo renderizando
        
        #----------------------------------------
        # BFS Search
        #----------------------------------------
        
        if viewer:
            viewer._figname = "BFS" 
        
        bfs_start_time = time.time()
        
        caminho, custo_total, expandidos = \
                breadth_first_search(labirinto, INICIO, GOAL, viewer)
        bfs_time = time.time() - bfs_start_time
        
        if len(caminho) == 0:
            print("Goal é inalcançavel neste labirinto.")

        print(
            f"BFS:"
            f"\tTempo: {bfs_time}.\n"
            f"\tCusto total do caminho: {custo_total}.\n"
            f"\tNumero de passos: {len(caminho)-1}.\n"
            f"\tNumero total de nos expandidos: {len(expandidos)}.\n\n"

        )

        if viewer:
            viewer.update(path=caminho)
            viewer.pause()


        #----------------------------------------
        # DFS Search
        #----------------------------------------
        if viewer:
            viewer._figname = "DFS" 
        
        dfs_start_time = time.time()
        caminho, custo_total, expandidos = \
                depth_first_search(labirinto, INICIO, GOAL, viewer)
        dfs_time = time.time() - dfs_start_time
        
        if len(caminho) == 0:
            print("Goal é inalcançavel neste labirinto.")

        print(
            f"DFS:"
            f"\tTempo: {dfs_time}.\n"
            f"\tCusto total do caminho: {custo_total}.\n"
            f"\tNumero de passos: {len(caminho)-1}.\n"
            f"\tNumero total de nos expandidos: {len(expandidos)}.\n\n"

        )

        if viewer:
            viewer.update(path=caminho)
            viewer.pause()

        #----------------------------------------
        # A-Star Search
        #----------------------------------------
        if viewer:
            viewer._figname = "A_Star" 
        
        as_start_time = time.time()
        caminho, custo_total, expandidos = \
                a_star_search(labirinto, INICIO, GOAL, viewer)
        as_time = time.time() - as_start_time
        
        if len(caminho) == 0:
            print("Goal é inalcançavel neste labirinto.")

        print(
            f"A-Star:"
            f"\tTempo: {as_time}.\n"
            f"\tCusto total do caminho: {custo_total}.\n"
            f"\tNumero de passos: {len(caminho)-1}.\n"
            f"\tNumero total de nos expandidos: {len(expandidos)}.\n\n"

        )

        if viewer:
            viewer.update(path=caminho)
            viewer.pause()
        #----------------------------------------
        # Uniform Cost Search (Obs: opcional)
        #----------------------------------------




    print("OK! Pressione alguma tecla pra finalizar...")
    input()


if __name__ == "__main__":
    main()
