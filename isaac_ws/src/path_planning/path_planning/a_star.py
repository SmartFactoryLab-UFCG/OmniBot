#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import heapq

# A classe AStarNode continua a mesma
class AStarNode:
    def __init__(self, position, parent=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __lt__(self, other):
        return self.f < other.f
    def __eq__(self, other):
        return self.position == other.position
    def __hash__(self):
        return hash(self.position)

class AStarCostmapPlanner(Node):
    def __init__(self):
        super().__init__("a_star_costmap_planner_node")
        self.get_logger().info("Nó de planejamento A* (baseado no Costmap) iniciado.")

        # Posição inicial (pode ser obtida de um tópico como /initialpose no futuro)
        self.start_pose = Pose()
        self.start_pose.position.x = 0.0
        self.start_pose.position.y = 0.0
        self.start_pose.orientation.w = 1.0
        self.get_logger().info(f"Posição inicial definida em: ({self.start_pose.position.x}, {self.start_pose.position.y})")
        
        # --- [ALTERAÇÃO] Parâmetros ---
        self.costmap_ = None
        # Valor máximo de custo para uma célula ser considerada "transitável"
        # 253 é o valor padrão no Nav2 para células com inflação inscrita
        self.lethal_cost_ = 100.0 
        # Fator de ponderação para o custo da célula
        self.cost_penalty_factor = 10.0

        # --- [ALTERAÇÃO] QoS para o Costmap ---
        # O costmap publica com durabilidade TRANSIENT_LOCAL
        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # --- [ALTERAÇÃO] Subscribers e Publishers ---
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, 
            "/costmap", # <-- MUDAMOS O TÓPICO DE /map PARA /costmap
            self.costmap_callback, 
            costmap_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, 
            "/goal_pose", 
            self.goal_callback, 
            10
        )
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

    def costmap_callback(self, msg: OccupancyGrid):
        """Callback para receber e armazenar o costmap."""
        if self.costmap_ is None:
            self.costmap_ = msg
            self.get_logger().info(f"Costmap recebido com sucesso! Resolução: {self.costmap_.info.resolution} m/pixel.")

    def goal_callback(self, msg: PoseStamped):
        """Callback acionado ao receber um novo objetivo do RViz."""
        if self.costmap_ is None:
            self.get_logger().warn("Aguardando o costmap para iniciar o planejamento...")
            return

        self.get_logger().info("Novo objetivo recebido! Iniciando o planejamento A* no costmap...")
        goal_pose = msg.pose
        path = self.plan_path(self.start_pose, goal_pose)
        if path:
            self.get_logger().info("Caminho encontrado! Publicando no tópico /planned_path.")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("Não foi possível encontrar um caminho até o objetivo.")

    def plan_path(self, start_pose: Pose, goal_pose: Pose):
        """Implementação principal do algoritmo A* usando o costmap."""
        start_node = self.world_to_grid(start_pose.position)
        goal_node = self.world_to_grid(goal_pose.position)

        if not self.is_valid(start_node) or not self.is_valid(goal_node):
            self.get_logger().error("Posição inicial ou final está em um obstáculo ou fora do mapa.")
            return None

        open_list = []
        closed_set = set()
        heapq.heappush(open_list, start_node)
        closed_set.add(start_node.position)
        
        movements = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node == goal_node:
                return self.reconstruct_path(current_node)

            for move in movements:
                neighbor_pos = (current_node.position[0] + move[0], current_node.position[1] + move[1])
                neighbor_node = AStarNode(neighbor_pos, current_node)

                if neighbor_node.position in closed_set or not self.is_valid(neighbor_node):
                    continue
                
                # --- [ALTERAÇÃO] Cálculo do Custo de Movimento ---
                distance_cost = 1.414 if abs(move[0]) == 1 and abs(move[1]) == 1 else 1.0
                cell_cost = self.get_cost(neighbor_node)
                
                # O custo agora inclui a penalidade da célula do costmap
                move_cost = distance_cost + self.cost_penalty_factor * (cell_cost / self.lethal_cost_)

                neighbor_node.g = current_node.g + move_cost
                neighbor_node.h = self.heuristic(neighbor_node, goal_node)
                neighbor_node.f = neighbor_node.g + neighbor_node.h
                
                if any(n for n in open_list if n == neighbor_node and n.f < neighbor_node.f):
                    continue

                heapq.heappush(open_list, neighbor_node)
                closed_set.add(neighbor_node.position)
        
        return None

    def heuristic(self, node1: AStarNode, node2: AStarNode):
        return math.sqrt((node1.position[0] - node2.position[0])**2 + (node1.position[1] - node2.position[1])**2)

    def get_cost(self, node: AStarNode):
        """Retorna o valor de custo de uma célula específica do costmap."""
        px, py = node.position
        map_index = py * self.costmap_.info.width + px
        # Retorna 0 se o custo for negativo (o que não deveria acontecer)
        return max(0, self.costmap_.data[map_index])

    def is_valid(self, node: AStarNode):
        """Verifica se um nó é transitável no costmap."""
        px, py = node.position
        width = self.costmap_.info.width
        height = self.costmap_.info.height
        
        if not (0 <= px < width and 0 <= py < height):
            return False
        
        cost = self.get_cost(node)
        
        # --- [ALTERAÇÃO] A verificação agora usa o valor de custo ---
        # Células com custo >= LETHAL_COST (254) são consideradas obstáculos intransponíveis.
        if cost >= self.lethal_cost_:
            return False
        return True

    def world_to_grid(self, world_pos):
        origin = self.costmap_.info.origin.position
        resolution = self.costmap_.info.resolution
        grid_x = int((world_pos.x - origin.x) / resolution)
        grid_y = int((world_pos.y - origin.y) / resolution)
        return AStarNode((grid_x, grid_y))

    def grid_to_world(self, grid_node: AStarNode):
        origin = self.costmap_.info.origin.position
        resolution = self.costmap_.info.resolution
        pose = Pose()
        pose.position.x = grid_node.position[0] * resolution + origin.x + resolution / 2
        pose.position.y = grid_node.position[1] * resolution + origin.y + resolution / 2
        pose.orientation.w = 1.0
        return pose
        
    def reconstruct_path(self, goal_node: AStarNode):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.costmap_.header.frame_id
        current = goal_node
        while current is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose = self.grid_to_world(current)
            path_msg.poses.append(pose_stamped)
            current = current.parent
        path_msg.poses.reverse()
        return path_msg

# ... (função main continua a mesma) ...

def main(args=None):
    rclpy.init(args=args)
    node = AStarCostmapPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()