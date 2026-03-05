#!/usr/bin/env python3
"""
world_to_map.py
---------------
Automatically generates a .pgm map and .yaml from a Gazebo .world file.
Only includes static models (ignores people, dynamic objects).

Usage:
  python3 world_to_map.py --world /path/to/my_world.world --output /path/to/maps/
"""

import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import argparse
import os
import math

RESOLUTION = 0.05  # meters per pixel
PADDING = 1.0      # extra meters around the world bounds


def parse_size(size_str):
    parts = size_str.strip().split()
    return float(parts[0]), float(parts[1])


def parse_pose(pose_str):
    parts = pose_str.strip().split()
    x = float(parts[0]) if len(parts) > 0 else 0.0
    y = float(parts[1]) if len(parts) > 1 else 0.0
    return x, y


def get_static_boxes(world_file):
    """Parse all static box models from the world file."""
    tree = ET.parse(world_file)
    root = tree.getroot()
    world = root.find('world') or root

    boxes = []

    for model in world.findall('model'):
        # Skip dynamic models
        static_el = model.find('static')
        if static_el is None or static_el.text.strip().lower() != 'true':
            continue

        # Skip ground plane
        name = model.get('name', '')
        if 'ground' in name.lower():
            continue

        # Get model pose
        pose_el = model.find('pose')
        mx, my = parse_pose(pose_el.text) if pose_el is not None else (0.0, 0.0)

        # Find all box geometries
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geom = collision.find('geometry')
                if geom is None:
                    continue

                box = geom.find('box')
                if box is not None:
                    size_el = box.find('size')
                    if size_el is not None:
                        sx, sy = parse_size(size_el.text)
                        boxes.append({
                            'name': name,
                            'cx': mx,
                            'cy': my,
                            'sx': sx,
                            'sy': sy,
                        })

                cylinder = geom.find('cylinder')
                if cylinder is not None:
                    r_el = cylinder.find('radius')
                    if r_el is not None:
                        r = float(r_el.text) * 2
                        boxes.append({
                            'name': name,
                            'cx': mx,
                            'cy': my,
                            'sx': r,
                            'sy': r,
                        })

    return boxes


def generate_map(world_file, output_dir):
    world_name = os.path.splitext(os.path.basename(world_file))[0]
    boxes = get_static_boxes(world_file)

    if not boxes:
        print(f'[WARN] No static boxes found in {world_file}')
        return

    print(f'[INFO] Found {len(boxes)} static objects:')
    for b in boxes:
        print(f'  {b["name"]}: center=({b["cx"]:.1f}, {b["cy"]:.1f}) size=({b["sx"]:.1f} x {b["sy"]:.1f})')

    # Calculate world bounds from all static objects
    all_x = [b['cx'] - b['sx']/2 for b in boxes] + [b['cx'] + b['sx']/2 for b in boxes]
    all_y = [b['cy'] - b['sy']/2 for b in boxes] + [b['cy'] + b['sy']/2 for b in boxes]

    min_x = min(all_x) - PADDING
    max_x = max(all_x) + PADDING
    min_y = min(all_y) - PADDING
    max_y = max(all_y) + PADDING

    width_px  = int((max_x - min_x) / RESOLUTION)
    height_px = int((max_y - min_y) / RESOLUTION)

    print(f'[INFO] World bounds: x=[{min_x:.1f}, {max_x:.1f}] y=[{min_y:.1f}, {max_y:.1f}]')
    print(f'[INFO] Map size: {width_px} x {height_px} pixels')

    # Build the map
    grid = np.full((height_px, width_px), 254, dtype=np.uint8)

    def fill_box(cx, cy, sx, sy):
        x0 = cx - sx / 2
        x1 = cx + sx / 2
        y0 = cy - sy / 2
        y1 = cy + sy / 2
        px0 = max(0, int((x0 - min_x) / RESOLUTION))
        px1 = min(width_px, int((x1 - min_x) / RESOLUTION) + 1)
        py0 = max(0, int((y0 - min_y) / RESOLUTION))
        py1 = min(height_px, int((y1 - min_y) / RESOLUTION) + 1)
        fy0 = height_px - py1
        fy1 = height_px - py0
        grid[fy0:fy1, px0:px1] = 0

    for b in boxes:
        fill_box(b['cx'], b['cy'], b['sx'], b['sy'])

    # Save .pgm
    os.makedirs(output_dir, exist_ok=True)
    pgm_path  = os.path.join(output_dir, f'{world_name}.pgm')
    yaml_path = os.path.join(output_dir, f'{world_name}.yaml')

    Image.fromarray(grid, mode='L').save(pgm_path)
    print(f'[OK] Saved map: {pgm_path}')

    # Save .yaml
    with open(yaml_path, 'w') as f:
        f.write(f'image: {world_name}.pgm\n')
        f.write(f'resolution: {RESOLUTION}\n')
        f.write(f'origin: [{min_x:.4f}, {min_y:.4f}, 0.0]\n')
        f.write(f'negate: 0\n')
        f.write(f'occupied_thresh: 0.65\n')
        f.write(f'free_thresh: 0.196\n')
    print(f'[OK] Saved yaml: {yaml_path}')


def main():
    parser = argparse.ArgumentParser(description='Convert Gazebo .world to Nav2 .pgm map')
    parser.add_argument('--world',  required=True, help='Path to .world file')
    parser.add_argument('--output', required=True, help='Output directory for .pgm and .yaml')
    args = parser.parse_args()

    if not os.path.isfile(args.world):
        print(f'[ERROR] World file not found: {args.world}')
        return

    generate_map(args.world, args.output)


if __name__ == '__main__':
    main()
    