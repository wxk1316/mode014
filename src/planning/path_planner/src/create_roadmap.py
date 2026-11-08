#!/usr/bin/env python3
"""
随机路网生成器 - 100节点，最小间距5米
边距离：随机5-15米
中心点在原点(0,0)
"""

import pickle
import random
import math
import networkx as nx
import matplotlib.pyplot as plt

def generate_random_roadmap(num_nodes=100, min_dist=5.0, max_edge_dist=15.0):
    """
    生成随机路网，中心点在原点
    边的距离为 min_dist 到 max_edge_dist 之间的随机值
    """
    G = nx.Graph()
    nodes = []
    
    # 计算地图半径（约60米）
    map_radius = int(math.sqrt(num_nodes * min_dist * min_dist)) // 2 + 30
    print(f"生成 {num_nodes} 个节点（节点最小间距: {min_dist}m）...")
    print(f"边距离范围: {min_dist}-{max_edge_dist}m")
    print(f"地图范围: -{map_radius} 到 {map_radius}m（半径 {map_radius}m）")
    
    # 1. 生成节点（保证最小间距）
    for i in range(num_nodes):
        attempts = 0
        max_attempts = 5000
        
        while attempts < max_attempts:
            # 在原点周围随机生成坐标
            x = random.uniform(-map_radius, map_radius)
            y = random.uniform(-map_radius, map_radius)
            
            # 检查与现有节点的距离
            valid = True
            for node_x, node_y, _ in nodes:
                dist = math.hypot(x - node_x, y - node_y)
                if dist < min_dist:
                    valid = False
                    break
            
            if valid:
                node = (round(x, 2), round(y, 2), 0.0)
                G.add_node(node, pos=(x, y))
                nodes.append((x, y, node))
                break
            attempts += 1
        
        if (i + 1) % 20 == 0:
            print(f"  进度: {i+1}/{num_nodes}")
    
    print(f"  实际节点数: {G.number_of_nodes()}")
    
    # 2. 连接邻近节点（边距离随机5-15米）
    print("连接节点...")
    connection_radius = max_edge_dist + 2  # 连接半径稍大于最大边距
    
    for i, (x1, y1, node1) in enumerate(nodes):
        connections = 0
        for j, (x2, y2, node2) in enumerate(nodes[i+1:], i+1):
            dist = math.hypot(x1 - x2, y1 - y2)
            
            # 在连接半径内且距离 >= 最小间距
            if dist <= connection_radius and dist >= min_dist:
                # 随机边距离（5-15米），但不能超过实际距离
                edge_dist = random.uniform(min_dist, min(max_edge_dist, dist))
                G.add_edge(node1, node2, weight=round(edge_dist, 2))
                connections += 1
                if connections >= 4:  # 限制节点度数
                    break
        
        if (i + 1) % 20 == 0:
            print(f"  进度: {i+1}/{len(nodes)}")
    
    # 3. 确保路网连通
    if not nx.is_connected(G):
        print("  修复连通性...")
        components = list(nx.connected_components(G))
        for i in range(len(components) - 1):
            min_dist_comp = float('inf')
            best_pair = None
            for n1 in components[i]:
                for n2 in components[i+1]:
                    dist = math.hypot(n1[0] - n2[0], n1[1] - n2[1])
                    if dist < min_dist_comp:
                        min_dist_comp = dist
                        best_pair = (n1, n2)
            if best_pair:
                edge_dist = random.uniform(min_dist, min(max_edge_dist, min_dist_comp))
                G.add_edge(best_pair[0], best_pair[1], weight=round(edge_dist, 2))
    
    # 4. 统计信息
    print("\n" + "="*50)
    print("Generation Complete")
    print("="*50)
    print(f"  Nodes: {G.number_of_nodes()}")
    print(f"  Edges: {G.number_of_edges()}")
    
    # 边距离统计
    edge_weights = [d['weight'] for _, _, d in G.edges(data=True)]
    if edge_weights:
        print(f"  Edge distance - Min: {min(edge_weights):.2f}m, Max: {max(edge_weights):.2f}m, Avg: {sum(edge_weights)/len(edge_weights):.2f}m")
    
    # 总长度
    total_length = sum(edge_weights)
    print(f"  Total length: {total_length:.2f}m ({total_length/1000:.2f}km)")
    
    # 坐标范围
    xs = [n[0] for n in G.nodes()]
    ys = [n[1] for n in G.nodes()]
    print(f"  X range: [{min(xs):.0f}, {max(xs):.0f}]m")
    print(f"  Y range: [{min(ys):.0f}, {max(ys):.0f}]m")
    
    return G

def save_roadmap(graph, filename="road_graph_100.pkl"):
    """保存路网到文件"""
    with open(filename, 'wb') as f:
        pickle.dump(graph, f)
    print(f"\nSaved to: {filename}")

def show_roadmap(graph):
    """显示路网（全英文）"""
    if graph.number_of_nodes() == 0:
        print("No nodes to display")
        return
    
    print("\nDisplaying road network...")
    pos = {node: (node[0], node[1]) for node in graph.nodes()}
    
    plt.figure(figsize=(12, 10))
    
    # Draw edges with color based on distance
    edges = list(graph.edges(data=True))
    edge_weights = [d['weight'] for _, _, d in edges]
    
    if edge_weights:
        # Normalize edge colors
        norm = plt.Normalize(min(edge_weights), max(edge_weights))
        edge_colors = [plt.cm.RdYlBu_r(norm(w)) for w in edge_weights]
        
        for (u, v, d), color in zip(edges, edge_colors):
            plt.plot([pos[u][0], pos[v][0]], [pos[u][1], pos[v][1]], 
                    color=color, linewidth=1.5, alpha=0.7)
        
        # Add colorbar for edges
        sm = plt.cm.ScalarMappable(cmap=plt.cm.RdYlBu_r, norm=norm)
        sm.set_array([])
        cbar = plt.colorbar(sm, label='Edge Distance (m)', ax=plt.gca())
    
    # Draw nodes (color by degree)
    degrees = dict(graph.degree())
    node_colors = [degrees[n] for n in graph.nodes()]
    
    nodes_draw = nx.draw_networkx_nodes(graph, pos, node_size=40, node_color=node_colors,
                                        cmap=plt.cm.viridis, alpha=0.8)
    
    # Colorbar for node degrees
    if nodes_draw:
        cbar2 = plt.colorbar(nodes_draw, label='Node Degree')
        min_deg = min(node_colors)
        max_deg = max(node_colors)
        if max_deg - min_deg <= 10:
            cbar2.set_ticks(range(min_deg, max_deg + 1))
    
    # Draw origin marker
    plt.plot(0, 0, 'r+', markersize=15, markeredgewidth=2, label='Origin (0,0)')
    
    # Axis and grid
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.2)
    plt.axvline(x=0, color='k', linestyle='-', alpha=0.2)
    plt.grid(True, alpha=0.3, linestyle='--')
    
    # Labels (English)
    plt.title(f'Random Road Network\n({graph.number_of_nodes()} nodes, {graph.number_of_edges()} edges)', 
              fontsize=14, fontweight='bold')
    plt.xlabel('X (meters)', fontsize=12)
    plt.ylabel('Y (meters)', fontsize=12)
    plt.axis('equal')
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    import sys
    print("Python version:", sys.version.split()[0])
    print("NetworkX version:", nx.__version__)
    print()
    
    # 固定随机种子，保证可重复
    random.seed(42)
    
    # 生成100节点路网，边距离5-15米随机
    roadmap = generate_random_roadmap(num_nodes=100, min_dist=5.0, max_edge_dist=15.0)
    
    # 保存
    save_roadmap(roadmap, "road_graph_100.pkl")
    
    # 显示
    show_roadmap(roadmap)