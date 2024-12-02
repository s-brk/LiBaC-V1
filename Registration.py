import open3d as o3d
import numpy as np
import copy
import time
import tempfile


def draw_registration_result(source, target, transformation):
    """
    Visualizes the alignment of two 3D point clouds after applying a transformation.

    This function takes two point clouds (`source` and `target`), applies a 
    given transformation matrix to the `source` point cloud, and renders both 
    point clouds together to visualize the alignment. The `source` is displayed 
    in orange, and the `target` is displayed in blue for easy distinction.

    Parameters:
    ----------
    source : open3d.geometry.PointCloud
        The source point cloud to be transformed.
    target : open3d.geometry.PointCloud
        The target point cloud used for alignment reference.
    transformation : numpy.ndarray
        A 4x4 transformation matrix to apply to the source point cloud.

    Returns:
    ----------
    None
    """

    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def save_registration_result(source, transformation):
    """
    Applies a transformation to a source point cloud and saves the result to a temporary `.ply` file.

    This function transforms the `source` point cloud using the given `transformation` matrix 
    and stores the transformed point cloud in a temporary file for further processing or visualization.

    Parameters:
    ----------
    source : open3d.geometry.PointCloud
        The point cloud to be transformed.
    
    target : open3d.geometry.PointCloud
        (Unused) The reference point cloud for alignment.
    
    transformation : numpy.ndarray
        A 4x4 matrix defining the transformation to apply to the source point cloud.

    Returns:
    ----------
    str
        The name of the temporary `.ply` file containing the transformed point cloud.
    """

    source_temp = copy.deepcopy(source)
    temp_file = tempfile.NamedTemporaryFile(suffix=".ply", delete=False)
    registered = source_temp.transform(transformation)
    o3d.io.write_point_cloud(temp_file.name , registered, write_ascii=True)
    return temp_file.name
    


def prepare_data(source_path, target_path):
    """
    Loads two point clouds from file and visualizes them before manual alignment.

    This function reads point clouds from the given file paths, `source_path` and 
    `target_path`, and then displays both point clouds using the `draw_registration_result` 
    function with an identity transformation, allowing for visualization before any alignment.

    Parameters:
    ----------
    source_path : str
        The file path to the source point cloud.
    
    target_path : str
        The file path to the target point cloud.

    Returns:
    ----------
    tuple : (open3d.geometry.PointCloud, open3d.geometry.PointCloud)
        The loaded source and target point clouds.
    """

    pcd_data = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target, np.identity(4))
    return source, target


def pick_points(pcd):
    """
    Allows the user to manually pick points on a point cloud for correspondence.

    This function opens a visualization window where the user can select at least three points 
    on the given point cloud using shift + left-click. The user can undo selections with shift + 
    right-click and close the window by pressing 'Q'. The picked points are returned as a list 
    of indices.

    Parameters:
    ----------
    pcd : open3d.geometry.PointCloud
        The point cloud from which the user will pick points.

    Returns:
    ----------
    vis.get_picked_points() : list
        A list of indices of the picked points in the point cloud.
    """

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

def scale_pointclouds_viacorrespondencies(sp, tp):
        
    """
    Calculates the scaling factor between two point clouds based on corresponding points.

    This function computes the scaling factor between two point clouds by comparing the 
    distances from each point to the origin (or any fixed reference point). The distances 
    of corresponding points in the source (`sp`) and target (`tp`) point clouds are 
    calculated, and the scaling factor is derived as the ratio of the distances. The 
    scaling factor is then used to construct a scaling transformation matrix.

    Parameters:
    ----------
    sp : np.array
        An Nx3 array representing the picked source points.
    tp : np.array
        An Nx3 array representing the picked target points.

    Returns:
    ----------
    scaled_init : np.array
        A 4x4 scaling transformation matrix
    """

    print("Calculating scale...")
    """
    #Mittlere distanz
    dist = 0
    for i in range(len(tp)):
        d = np.linalg.norm(tp[i]- sp[i])
        dist += d
    print("Distanz addiert:", dist)
    scale_factor = dist/len(tp) 

    print(f"Skalierungsfaktor: {scale_factor}")
    sf = np.mean(np.linalg.norm(tp, axis=1)) / np.mean(np.linalg.norm(sp, axis=1))
    print("SF", sf)

    """
    # Berechne die Distanzen zwischen Paaren korrespondierender Punkte
    distances_source = []
    distances_target = []
    for i in range(len(sp)):
        for j in range(i + 1, len(sp)):  # Vermeide doppelte Berechnung (i,j) und (j,i)
            d_source = np.linalg.norm(sp[i] - sp[j])
            d_target = np.linalg.norm(tp[i] - tp[j])
            distances_source.append(d_source)
            distances_target.append(d_target)

    # Berechne die Skalierungsfaktoren für jedes Punktpaar
    scale_factors = [d_target / d_source for d_source, d_target in zip(distances_source, distances_target)]

    # Mittelwert der Skalierungsfaktoren
    scale_factor = np.mean(scale_factors)

    scaled_init = np.array([
         [ scale_factor, 0.000, 0.000, 0.000],
         [ 0.000, scale_factor, 0.000, 0.000],
         [ 0.000, 0.000, scale_factor, 0.000],
         [ 0.000, 0.000, 0.000, 1.000]
     ])
    return scaled_init


def register_via_correspondences(source, target, source_points, target_points):
    """
    Registers two point clouds using user-provided correspondences and performs ICP refinement.

    This function computes an initial transformation based on point correspondences between the 
    `source` and `target` point clouds. It then refines this transformation using point-to-point ICP 
    for better alignment. The final result is visualized, and the transformed `source` point cloud 
    is saved to a temporary file.

    Parameters:
    ----------
    source : open3d.geometry.PointCloud
        The source point cloud to be aligned with the target.

    target : open3d.geometry.PointCloud
        The target point cloud used for alignment.

    source_points : list of int
        Indices of points in the source point cloud selected as correspondences.

    target_points : list of int
        Indices of points in the target point cloud selected as correspondences.

    Returns:
    ----------
    temp_registration : str
        The name of the temporary file containing the aligned source point cloud.
    """
     
    corr = np.zeros((len(source_points), 2))
    corr[:, 0] = source_points
    corr[:, 1] = target_points

    # Extract corresponding points based on the indices saved in `corr`
    sp_corr_points = np.asarray(source.points)[corr[:, 0].astype(int), :]  # Source points
    tp_corr_points = np.asarray(target.points)[corr[:, 1].astype(int), :]  # Target points

    print("corresponding points")
    print("source points:", sp_corr_points)
    print("target points:",tp_corr_points)

    # Calculate scaling
    scaling_matrix = scale_pointclouds_viacorrespondencies(sp_corr_points, tp_corr_points)
    print("scaling Matrix:", scaling_matrix)
    source.transform(scaling_matrix)

    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))
    #source.transform(trans_init)
    o3d.visualization.draw_geometries([source, target])

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.3  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    temp_registration = save_registration_result(source, reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)

    #ohne ICP  Ergebniss
    #temp_registration = save_registration_result(source, trans_init)
    #draw_registration_result(source, target, trans_init)
    return temp_registration

def downsample_point_cloud(pcd, voxel_size):
    """
    Downsamples a point cloud using voxel grid filtering.

    This function reduces the number of points in the given point cloud by applying voxel 
    downsampling with a specified voxel size. This helps to reduce the computational load 
    for further processing.

    Parameters:
    ----------
    pcd : open3d.geometry.PointCloud
        The point cloud to be downsampled.

    voxel_size : float
        The size of the voxel grid for downsampling. Smaller values preserve more details.

    Returns:
    ----------
    pcd_down : open3d.geometry.PointCloud
        The downsampled point cloud.
    """

    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
   
    return pcd_down

def demo_manual_registration():
    """
    Demonstrates manual point cloud registration using user-provided correspondences.

    This function loads a demo dataset, allows the user to pick correspondences between
    the source and target point clouds, and then performs a manual ICP registration.

    Returns:
    ----------
    None
    """

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
    """
    Performs manual ICP registration for two point clouds with downsampling.

    This function loads the point clouds from the given paths, downsample the source 
    point cloud based on the number of points, allows the user to pick correspondences 
    between the source and target point clouds, and performs ICP registration.

    Parameters:
    ----------
    lidar_path : str
        The file path to the LIDAR point cloud (source).

    gaussiansplat_path : str
        The file path to the Gaussian-splatted point cloud (target).

    Returns:
    ----------
    registered : str
        The name of the temporary file containing the registered source point cloud.
    """

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


lidar_path = "/Users/swenja/Desktop/Studium/3_Semester/Masterprojekt/Masterkueche_LIDAR_CloudCompare_unalligned.ply"
pointcloud_path = "/Users/swenja/Desktop/Studium/3_Semester/Masterprojekt/Masterkueche_GaussianSplat_CloudCompare_scaled.ply"
manual_registration(lidar_path, pointcloud_path)
