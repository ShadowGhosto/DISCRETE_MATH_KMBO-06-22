import numpy as np
import networkx as nx
import cv2

def image_to_graph(image, threshold=127):
    height, width = image.shape
    G = nx.DiGraph()

    source = (-1, -1)
    sink = (-2, -2)

    G.add_node(source)
    G.add_node(sink)

    for i in range(height):
        for j in range(width):
            node = (i, j)
            capacity = image[i, j]
            G.add_edge(source, node, capacity=capacity)
            G.add_edge(node, sink, capacity=capacity)

            if i > 0:
                neighbor = (i - 1, j)
                G.add_edge(node, neighbor, capacity=min(capacity, image[i - 1, j]))
            if j > 0:
                neighbor = (i, j - 1)
                G.add_edge(node, neighbor, capacity=min(capacity, image[i, j - 1]))

    return G, source, sink

def segment_image(image_path, threshold=127):

    original_image = cv2.imread(image_path)
    cv2.imshow('Original Image', original_image)

    image_gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    cv2.imshow('Gray Image', image_gray)
    _, binary_image = cv2.threshold(image_gray, threshold, 255, cv2.THRESH_BINARY)

    G1, source, sink = image_to_graph(image_gray)
    G2, source, sink = image_to_graph(binary_image)

    _1, flow_dict1 = nx.maximum_flow(G1, source, sink)
    _2, flow_dict2 = nx.maximum_flow(G2, source, sink)

    segmented_image_gray = np.zeros_like(image_gray)
    for i in range(image_gray.shape[0]):
        for j in range(image_gray.shape[1]):
            if flow_dict1[source][i, j] > 0:
                segmented_image_gray[i, j] = 255
    
    segmented_image_binary = np.zeros_like(binary_image)
    for i in range(binary_image.shape[0]):
        for j in range(binary_image.shape[1]):
            if flow_dict2[source][i, j] > 0:
                segmented_image_binary[i, j] = 255

    # Отображение результатов
    cv2.imshow('Binary Image', binary_image)
    cv2.imshow('segmented_image_gray', segmented_image_gray)
    cv2.imshow('segmented_image_binary', segmented_image_binary)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Пример использования
image_path = '1.jpg'
segment_image(image_path)
