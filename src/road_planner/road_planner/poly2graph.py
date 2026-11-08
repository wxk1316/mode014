import numpy as np
import networkx as nx
import pickle
import matplotlib.pyplot as plt

def read_poly_file(filepath):
    """读取 CloudCompare 导出的 .poly 文件"""
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    polylines = []
    current = []
    
    for line in lines:
        line = line.strip()
        if not line:
            if current:
                polylines.append(np.array(current))
                current = []
        else:
            parts = line.split()
            if len(parts) >= 3:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                current.append([x, y, z])
    
    if current:
        polylines.append(np.array(current))
    
    return polylines

def build_graph(polylines):
    """将折线列表转换为 NetworkX 图，并对长线段进行自动插值密集化"""
    G = nx.Graph()
    
    # 插值密度：每隔 0.5 米产生一个节点
    interpolate_dist = 0.2
    
    for polyline in polylines:
        for i in range(len(polyline) - 1):
            p1 = polyline[i]
            p2 = polyline[i+1]
            
            dist = np.linalg.norm(p2 - p1)
            num_segments = max(1, int(np.ceil(dist / interpolate_dist)))
            
            # 生成插值节点序列
            prev_node = tuple(p1)
            G.add_node(prev_node, pos=prev_node)
            
            for j in range(1, num_segments + 1):
                # 线性插值
                interp_pt = p1 + (p2 - p1) * (j / num_segments)
                curr_node = tuple(interp_pt)
                
                G.add_node(curr_node, pos=curr_node)
                
                # 连接前后两个节点
                segment_dist = np.linalg.norm(np.array(curr_node) - np.array(prev_node))
                G.add_edge(prev_node, curr_node, weight=segment_dist)
                
                prev_node = curr_node
    
    return G

def merge_close_nodes(G, threshold=0.1):
    """合并距离小于阈值的节点"""
    nodes = list(G.nodes())
    merged = {}
    
    for i, node1 in enumerate(nodes):
        if node1 in merged:
            continue
        group = [node1]
        for node2 in nodes[i+1:]:
            if node2 in merged:
                continue
            if np.linalg.norm(np.array(node1) - np.array(node2)) < threshold:
                group.append(node2)
        
        center = tuple(np.mean(group, axis=0))
        for node in group:
            merged[node] = center
    
    new_G = nx.Graph()
    for node, new_node in merged.items():
        new_G.add_node(new_node, pos=new_node)
    
    for u, v, data in G.edges(data=True):
        new_u = merged.get(u, u)
        new_v = merged.get(v, v)
        if new_u != new_v:
            new_G.add_edge(new_u, new_v, weight=data['weight'])
    
    return new_G

def visualize_pkl(pkl_filepath):
    """可视化 pkl 文件中的路网图"""
    with open(pkl_filepath, 'rb') as f:
        G = pickle.load(f)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 绘制所有边
    for u, v in G.edges():
        x = [u[0], v[0]]
        y = [u[1], v[1]]
        ax.plot(x, y, 'gray', linewidth=2, alpha=0.8)
    
    # 绘制所有节点
    nodes = np.array(list(G.nodes()))
    ax.scatter(nodes[:,0], nodes[:,1], c='blue', s=20, zorder=2)
    
    # 高亮交叉点（度数 >= 3）
    intersections = [n for n, d in G.degree() if d >= 3]
    if intersections:
        inter_xy = np.array(intersections)
        ax.scatter(inter_xy[:,0], inter_xy[:,1], c='red', s=80, zorder=3, label='Intersection')
    
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Road Network - {pkl_filepath}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # 打印图信息
    print(f"\n图信息:")
    print(f"  节点数: {G.number_of_nodes()}")
    print(f"  边数: {G.number_of_edges()}")
    print(f"  连通分量数: {nx.algorithms.components.number_connected_components(G)}")

# ========== 主程序 ==========
if __name__ == "__main__":
    input_poly = "Polyline.poly"           # 你的 poly 文件名
    output_pkl = "road_graph.pkl"     # 输出的 pkl 文件名
    
    # 读取并转换
    polylines = read_poly_file(input_poly)
    print(f"读取到 {len(polylines)} 条折线")
    
    G = build_graph(polylines)
    print(f"初始图: {G.number_of_nodes()} 节点, {G.number_of_edges()} 边")
    
    G = merge_close_nodes(G, threshold=0.1)
    print(f"合并后: {G.number_of_nodes()} 节点, {G.number_of_edges()} 边")
    
    # 保存
    with open(output_pkl, "wb") as f:
        pickle.dump(G, f)
    print(f"✅ 已保存为 {output_pkl}")
    
    # 可视化
    visualize_pkl(output_pkl)