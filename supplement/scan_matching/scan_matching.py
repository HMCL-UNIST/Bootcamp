import open3d as o3d
import copy
import numpy as np

def draw_registration_result(source, target, transformation):
    # Perform registration
    # Visualize registration result
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0]) 
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    target_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
