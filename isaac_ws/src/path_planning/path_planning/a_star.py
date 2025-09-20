#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy
import heapq # Usaremos heapq para uma fila de prioridade mais eficiente

# Classe Node adaptada para o A*
class AStarNode:
    def __init__(self, position, parent=None):
        self.parent = parent
        self.position = position  # Tupla (x, y) nas coordenadas do grid

        self.g = 0  # Custo do início até o nó atual
        self.h = 0  # Heurística (custo estimado até o final)
        self.f = 0  # Custo total (g + h)

    # Métodos de comparação para a fila de prioridade (heapq)
    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.position == other.position

    # Hash para permitir adicionar o nó a um conjunto (set)
    def __hash__(self):
        return hash(self.position)

class AStarRosPlanner(Node):
    def __init__(self):
        super().__init__("a_star_planner_node")
        self.get_logger().info("Nó de planejamento A* iniciado.")

        # --- Posição Inicial Fixa ---
        # Defina aqui a posição (x, y) inicial do robô no frame do mapa
        self.start_pose = Pose()
        self.start_pose.position.x = -6.0
        self.start_pose.position.y = 1.5
        self.start_pose.orientation.w = 1.0
        self.get_logger().info(f"Posição inicial definida em: ({self.start_pose.position.x}, {self.start_pose.position.y})")
        
        # --- Parâmetros ---
        self.map_ = None
        
        # --- Configuração de QoS para o Mapa ---
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- Subscribers e Publishers ---
        # Subscriber do mapa Occupancy Grid
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            "/map", 
            self.map_callback, 
            map_qos
        )

        # Subscriber do 2D Goal Pose
        self.goal_sub = self.create_subscription(
            PoseStamped, 
            "/goal_pose", 
            self.goal_callback, 
            10
        )

        # Publisher do caminho planejado
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

    def map_callback(self, msg: OccupancyGrid):
        """Callback para receber e armazenar o mapa de ocupação."""
        if self.map_ is None:
            self.map_ = msg
            self.get_logger().info(f"Mapa recebido com sucesso! Resolução: {self.map_.info.resolution} m/pixel.")

    def goal_callback(self, msg: PoseStamped):
        """Callback acionado ao receber um novo objetivo do RViz."""
        if self.map_ is None:
            self.get_logger().warn("Aguardando o mapa para iniciar o planejamento...")
            return

        self.get_logger().info("Novo objetivo recebido! Iniciando o planejamento A*...")
        
        # O objetivo é a pose recebida na mensagem
        goal_pose = msg.pose
        
        # Executa o planejamento
        path = self.plan_path(self.start_pose, goal_pose)

        if path:
            self.get_logger().info("Caminho encontrado! Publicando no tópico /planned_path.")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("Não foi possível encontrar um caminho até o objetivo.")

    def plan_path(self, start_pose: Pose, goal_pose: Pose):
        """Implementação principal do algoritmo A*."""
        # Converte as poses do mundo (metros) para coordenadas do grid (pixels)
        start_node = self.world_to_grid(start_pose.position)
        goal_node = self.world_to_grid(goal_pose.position)

        # Verifica se os pontos de início e fim são válidos
        if not self.is_valid(start_node) or not self.is_valid(goal_node):
            self.get_logger().error("Posição inicial ou final está em um obstáculo ou fora do mapa.")
            return None

        open_list = []
        closed_set = set()

        heapq.heappush(open_list, start_node)
        closed_set.add(start_node.position)
        
        # Define os movimentos possíveis (8 direções, incluindo diagonais)
        movements = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        while open_list:
            current_node = heapq.heappop(open_list)

            # Se chegamos ao objetivo, reconstrua o caminho
            if current_node == goal_node:
                return self.reconstruct_path(current_node)

            # Explora os vizinhos
            for move in movements:
                neighbor_pos = (current_node.position[0] + move[0], current_node.position[1] + move[1])
                neighbor_node = AStarNode(neighbor_pos, current_node)

                if neighbor_node.position in closed_set:
                    continue
                
                if self.is_valid(neighbor_node):
                    move_cost = 1.414 if abs(move[0]) == 1 and abs(move[1]) == 1 else 1.0 # Custo diagonal vs. reto
                    neighbor_node.g = current_node.g + move_cost
                    neighbor_node.h = self.heuristic(neighbor_node, goal_node)
                    neighbor_node.f = neighbor_node.g + neighbor_node.h
                    
                    # Verifica se o vizinho já está na lista aberta com um custo maior
                    if any(n for n in open_list if n == neighbor_node and n.g < neighbor_node.g):
                        continue

                    heapq.heappush(open_list, neighbor_node)
                    closed_set.add(neighbor_node.position)
        
        return None # Nenhum caminho encontrado

    def heuristic(self, node1: AStarNode, node2: AStarNode):
        """Calcula a distância Euclidiana como heurística."""
        return math.sqrt((node1.position[0] - node2.position[0])**2 + (node1.position[1] - node2.position[1])**2)

    def is_valid(self, node: AStarNode):
        """Verifica se um nó está dentro dos limites do mapa e não é um obstáculo."""
        px, py = node.position
        width = self.map_.info.width
        height = self.map_.info.height
        
        if not (0 <= px < width and 0 <= py < height):
            return False # Fora dos limites
        
        # O mapa é um array 1D, então convertemos (px, py) para um índice
        map_index = py * width + px
        
        # Valores de OccupancyGrid: 0-100. Valores > 50 são considerados obstáculos. -1 é desconhecido.
        if self.map_.data[map_index] > 50 or self.map_.data[map_index] == -1:
            return False # É um obstáculo ou área desconhecida

        return True

    def world_to_grid(self, world_pos):
        """Converte uma posição do mundo (em metros) para coordenadas do grid."""
        origin = self.map_.info.origin.position
        resolution = self.map_.info.resolution
        grid_x = int((world_pos.x - origin.x) / resolution)
        grid_y = int((world_pos.y - origin.y) / resolution)
        return AStarNode((grid_x, grid_y))

    def grid_to_world(self, grid_node: AStarNode):
        """Converte coordenadas do grid de volta para uma pose no mundo."""
        origin = self.map_.info.origin.position
        resolution = self.map_.info.resolution
        pose = Pose()
        # Centraliza o ponto no meio da célula do grid
        pose.position.x = grid_node.position[0] * resolution + origin.x + resolution / 2
        pose.position.y = grid_node.position[1] * resolution + origin.y + resolution / 2
        pose.orientation.w = 1.0
        return pose
        
    def reconstruct_path(self, goal_node: AStarNode):
        """Gera a mensagem nav_msgs/Path a partir do nó objetivo."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_.header.frame_id

        current = goal_node
        while current is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose = self.grid_to_world(current)
            path_msg.poses.append(pose_stamped)
            current = current.parent
        
        path_msg.poses.reverse() # O caminho é reconstruído de trás para frente
        return path_msg

def main(args=None):
    rclpy.init(args=args)
    node = AStarRosPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()