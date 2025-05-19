import math

def generate_half_cylinder(radius=0.05, height=0.02, segments=8):
    points = []
    indices = []

    # Bottom half-circle (Z = 0)
    for i in range(segments + 1):
        angle = math.pi * i / segments  # Half circle
        x = radius * math.cos(angle + (math.pi / 2))
        y = radius * math.sin(angle + (math.pi / 2))
        points.append((x, y, 0))

    # Top half-circle (Z = height)
    for i in range(segments + 1):
        angle = math.pi * i / segments
        x = radius * math.cos(angle + (math.pi / 2))
        y = radius * math.sin(angle + (math.pi / 2))
        points.append((x, y, height))

    # Add center points for the caps
    points.append((0, 0, 0))       # Bottom center
    points.append((0, 0, height))  # Top center

    bottom_center = len(points) - 2
    top_center = len(points) - 1

    # Faces: bottom cap
    for i in range(segments):
        # indices.append([bottom_center, i, i + 1])
        indices.append([bottom_center, i + 1, i])

    # Faces: top cap
    for i in range(segments):
        # indices.append([top_center, segments + 1 + i + 1, segments + 1 + i])
        indices.append([top_center, segments + 1 + i, segments + 1 + i + 1])

    # Side faces
    for i in range(segments):
        a = i
        b = i + 1
        c = segments + 1 + i + 1
        d = segments + 1 + i
        indices.append([a, c, d])
        indices.append([a, b, c])

    # Flat side walls (closing the straight edges)
    # First vertical face (between point 0 and top half 0)
    indices.append([0, segments + 1, bottom_center])
    indices.append([segments + 1, top_center, bottom_center])

    # Second vertical face (last point on arc)
    indices.append([segments, bottom_center, segments + 1 + segments])
    indices.append([segments + 1 + segments, bottom_center, top_center])

    return points, indices

if __name__ == "__main__":
    points, indices = generate_half_cylinder(radius=0.14, height=0.01, segments=16)
    # print(points)
    # print()
    # print(indices)
    for x, y, z in points:
        print(f"{x} {y} {z} ", end="")
    print()
    for x, y, z in indices:
        print(f"{x}, {y}, {z}, -1, ", end="\n")