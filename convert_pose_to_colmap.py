import numpy as np
import open3d as o3d
import os
from scipy.spatial.transform import Rotation as R

def save_colmap_files(camera_intrinsics, extrinsics_list, image_names, output_dir):
    """
    Saves camera intrinsics and extrinsics in COLMAP-compatible format.

    Writes:
      - cameras.txt
      - images.txt

    Parameters:
      - camera_intrinsics: tuple (fx, fy, cx, cy, width, height)
      - extrinsics_list: list of 4x4 matrices (camera-to-world transformations)
      - image_names: list of corresponding image file names
      - output_dir: directory where the files will be saved
    """
    fx, fy, cx, cy, width, height = camera_intrinsics
    os.makedirs(output_dir, exist_ok=True)
    
    # Paths for the output files
    cameras_path = os.path.join(output_dir, "cameras.txt")
    images_path = os.path.join(output_dir, "images.txt")
    
    # Overwrite files if they exist
    if os.path.exists(cameras_path):
        os.remove(cameras_path)
    if os.path.exists(images_path):
        os.remove(images_path)
    import ipdb; ipdb.set_trace()
    # Write cameras.txt (using PINHOLE model: parameters = fx, fy, cx, cy)
    with open(cameras_path, "w") as f:
        f.write("# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n")
        f.write(f"1 PINHOLE {width} {height} {fx} {fy} {cx} {cy}\n")
    
    # Write images.txt
    with open(images_path, "w") as f:
        f.write("# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID IMAGE_NAME\n")
        for idx, (extrinsics, img_name) in enumerate(zip(extrinsics_list, image_names), start=1):
            # Convert camera-to-world to world-to-camera (COLMAP expects world-to-camera)
            extrinsics_inv = np.linalg.inv(extrinsics)
            R_matrix = extrinsics_inv[:3, :3]
            t_vector = extrinsics_inv[:3, 3]
            
            # Convert rotation matrix to quaternion; scipy returns (x, y, z, w)
            quat = R.from_matrix(R_matrix).as_quat()
            # Rearrange to (w, x, y, z)
            qw, qx, qy, qz = quat[3], quat[0], quat[1], quat[2]
            
            line = f"{idx} {qw} {qx} {qy} {qz} {t_vector[0]} {t_vector[1]} {t_vector[2]} 1 {img_name}"
            f.write(line + "\n")
            f.write("\n")  # Empty second line (COLMAP format)
    
    print(f"Saved COLMAP files to {output_dir}: cameras.txt and images.txt")

def convert_ply_to_colmap_points3D(ply_file, output_dir, offset=np.array([0.0, 0.0, 0.0])):
    """
    Converts a PLY point cloud file to COLMAP's points3D.txt format with an optional offset.

    Writes:
      - points3D.txt

    Parameters:
      - ply_file: path to the input .ply file
      - output_dir: directory to save points3D.txt
      - offset: numpy array of shape (3,) specifying translation along X, Y, Z.
    """
    os.makedirs(output_dir, exist_ok=True)
    # Overwrite points3D.txt if it exists
    points3D_path = os.path.join(output_dir, "points3D.txt")
    if os.path.exists(points3D_path):
        os.remove(points3D_path)
        
    pcd = o3d.io.read_point_cloud(ply_file)
    if not pcd.has_points():
        raise ValueError("The PLY file does not contain any valid points.")
    
    # Apply offset to the point coordinates
    points = np.asarray(pcd.points) + offset
    
    # Get RGB colors if available; otherwise, use black
    if pcd.has_colors():
        colors = (np.asarray(pcd.colors) * 255).astype(int)
    else:
        colors = np.zeros_like(points, dtype=int)
    
    with open(points3D_path, "w") as f:
        f.write("# 3D point list\n")
        f.write("# POINT3D_ID X Y Z R G B ERROR TRACKS[]\n")
        for idx, (point, color) in enumerate(zip(points, colors), start=1):
            x, y, z = point
            r, g, b = color
            error = 0  # Default error value
            f.write(f"{idx} {x} {y} {z} {r} {g} {b} {error}\n")
    
    print(f"Converted {len(points)} points to COLMAP format: {points3D_path}")
    print(f"Applied Offset: {offset}")

# def visualize_pcd_with_cameras(ply_file, extrinsics_list):
#     """
#     Visualizes the point cloud with the camera poses in Open3D.
    
#     Parameters:
#       - ply_file: path to the PLY file.
#       - extrinsics_list: list of 4x4 camera-to-world transformation matrices.
#     """
#     pcd = o3d.io.read_point_cloud(ply_file)
#     if pcd.is_empty():
#         raise ValueError("Point cloud is empty!")
    
#     # Create coordinate frames for each camera pose.
#     camera_frames = []
#     for extrinsics in extrinsics_list:
#         # Invert the transform to visualize the camera (world-to-camera -> camera-to-world)
#         camera = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
#         camera.transform(np.linalg.inv(extrinsics))
#         camera_frames.append(camera)
    
#     o3d.visualization.draw_geometries([pcd] + camera_frames)

def main():
    # Intrinsics
    fx = 64
    fy = 64
    width = 128
    height = 128
    cx = width / 2
    cy = height / 2
    camera_intrinsics = (fx, fy, cx, cy, width, height)
    
    # Extrinsics
    extrinsics_list = [np.array([
        [0, 1, 0, 0],          
        [0.78086, 0, -0.62469, 0.14055],  
        [-0.624695, 0, -0.78086, 0.655929],
        [0, 0, 0, 1]           
    ])]
    image_names = ["image1.jpg"] # This is to comply with colmap data
    
   
    output_dir = "/Users/haoranchang/Projects/gs_world/cam_poses_txt"
    

    # Save cameras.txt and images.txt
    save_colmap_files(camera_intrinsics, extrinsics_list, image_names, output_dir)
    
    # Convert a PLY file to points3D.txt
    ply_file = "/Users/haoranchang/Projects/gs_world/assets/fr3_umi0208.ply"
    # This is to shift the robot, similar to maniskill where you shift robot in different envs
    offset_matrix = np.array([0, 0, 0])
    convert_ply_to_colmap_points3D(ply_file, output_dir, offset_matrix)
    

if __name__ == "__main__":
    # Basically you would have to determine 1. intrinsics (camera pose)
    #                                       2. extrinsics (camera pose)
    #                                       3. ply files (pcd)
    #                                       4. offset (same as in maniskill)
    #                                       5. output dir
    #
    # After this, run the following command and then you can use colmap GUI to open:
    # colmap model_converter --input_path ./cam_poses_txt --output_path ./cam_poses_bin --output_type BIN   

    main()

