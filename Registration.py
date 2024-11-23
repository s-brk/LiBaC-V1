import open3d as o3d
import numpy as np
import copy
import time
import tempfile


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def save_registration_result(source, transformation):
    source_temp = copy.deepcopy(source)
    temp_file = tempfile.NamedTemporaryFile(suffix=".ply", delete=False)
    registered = source_temp.transform(transformation)
    o3d.io.write_point_cloud(temp_file.name , registered, write_ascii=True)
    return temp_file.name
    


def prepare_data(source_path, target_path):
    pcd_data = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target, np.identity(4))
    return source, target


def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def scale_pointclouds_viacorrespondencies(source_points, target_points):

    return None



def register_via_correspondences(source, target, source_points, target_points):
    corr = np.zeros((len(source_points), 2))
    corr[:, 0] = source_points
    corr[:, 1] = target_points

    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))
    
    #override trans init with transform from cloud compare to show if it is working (Musikzimmer)
    # trans_init = np.array([
    #     [-4.753,  1.162, -9.471,  3.797],
    #     [ 3.121, 10.188, -0.316, -12.449],
    #     [ 9.017, -2.914, -4.882, -12.342],
    #     [ 0.000,  0.000,  0.000,   1.000]
    # ])
    
    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.3  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    temp_registration = save_registration_result(source, reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)
    return temp_registration

def downsample_point_cloud(pcd, voxel_size):
    
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
   
    return pcd_down

def demo_manual_registration():

    print("Demo for manual ICP")
    pcd_data = o3d.data.DemoICPPointClouds()
    source, target = prepare_data(pcd_data.paths[0], pcd_data.paths[2])

    # pick points from two point clouds and builds correspondences
    source_points = pick_points(source)
    target_points = pick_points(target)
    assert (len(source_points) >= 3 and len(target_points) >= 3)
    assert (len(source_points) == len(target_points))
    register_via_correspondences(source, target, source_points, target_points)
    print("")

def manual_registration(lidar_path, gaussiansplat_path):

    print("manual ICP")
    
    source, target = prepare_data(lidar_path, gaussiansplat_path)

    lidar_points = len(source.points)
    gaussian_points = len(target.points)

    voxel_size = (lidar_points//gaussian_points)*0.001
    print(voxel_size)
    source_down = downsample_point_cloud(source, voxel_size)



    # pick points from two point clouds and builds correspondences
    source_points = pick_points(source_down)
    target_points = pick_points(target)
    assert (len(source_points) >= 3 and len(target_points) >= 3)
    assert (len(source_points) == len(target_points))
    registered = register_via_correspondences(source_down, target, source_points, target_points)
    print("")
    return registered

