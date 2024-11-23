import pandas as pd
import open3d as o3d
import numpy as np
import argparse
import os
import subprocess
import tempfile
import Registration

class Reduce:
    """
    A class for processing point clouds based on LIDAR data.

    This class provides methods to reduce a point cloud by removing points 
    that are not within a specified distance of a LIDAR point cloud. It 
    includes functionality to read and write PLY files, manipulate DataFrames, 
    and manage workspace directories.

    Attributes:
    ----------
    threshold (float): Distance threshold for filtering points.
    pointcloud_path (str): Path to the input point cloud file.
    lidar_path (str): Path to the LiDAR point cloud file.
    output_path (str): Path to the workspace directory for output files.
    ply_path (str): Path to save the final PLY file.
    cc_pc_reduced (open3d.geometry.PointCloud): The reduced point cloud.
    ind (numpy.ndarray): Indices of retained points after filtering.
    df2_filtered (pd.DataFrame): Filtered DataFrame from the second point cloud.

    Methods:
    -------
    start(args): Initializes processing based on command line arguments.
    delete_points_based_on_lidar(): Removes points from the point cloud based on LIDAR proximity.
    read_ply_to_dataframe(file_path): Reads a PLY file into a pandas DataFrame.
    merge_dataframes_with_header(df1, header_lines1, df2, header_lines2): Merges two DataFrames and updates headers.
    change_rgb_and_normal_position(df, insert_index, start_index, end_index, header): Repositions specified columns in the DataFrame.
    convert_color_columns_to_int(df): Converts color columns in the DataFrame to integers.
    dataframe_to_ply(df, header_lines, ply_path): Writes a DataFrame to a PLY file with the given header.
    """

    threshold = 0.1

    # workspace_patch=None # braucht man glaube ich nicht ????
    pointcloud_path = None
    lidar_path = None
    output_path = None #intermediate
    ply_path = None 

    #saving_path = None         #ist auskommentiert unten

    cc_pc_reduced = None
    ind = None
    df2_filtered = None

    @classmethod
    def create_temp_ply(cls):
        """
        Creates temporary files, which will be modified in the process.

        Parameters:
        ----------
        None

        Returns:
        ----------
        None
        """

        temp_file_ply = tempfile.NamedTemporaryFile(suffix=".ply", delete=False)
        temp_file_output = tempfile.NamedTemporaryFile(suffix=".ply", delete=False)
        cls.ply_path = temp_file_ply.name
        cls.output_path = temp_file_output.name

    @classmethod
    def delete_temp_ply(cls):
        """
        Deletes existing temporary files.

        Parameters:
        ----------
        None

        Returns:
        ----------
        None
        """
        if cls.ply_path and os.path.exists(cls.ply_path):
            os.remove(cls.ply_path)

        if cls.output_path and os.path.exists(cls.output_path):
            os.remove(cls.output_path)

        if os.path.exists("filtered.csv"):
            os.remove("filtered.csv")
        else:
            print("The file does not exist in current working directory:", os.getcwd())
        

    @classmethod
    def delete_points_based_on_lidar(cls):
        """
        Removes points from the point cloud if no LiDAR point is within the threshold.

        Reads point clouds from `cls.pointcloud_path` and `cls.lidar_path`, computes distances 
        between them, and keeps only the points in the point cloud that are within `cls.threshold` 
        distance from any LiDAR point. Saves the reduced point cloud to `cls.output_path` if provided.
        
        Returns:
        ----------
        None
        """

        cc_pc = o3d.io.read_point_cloud(cls.pointcloud_path)

        lidar_pc = o3d.io.read_point_cloud(cls.lidar_path)

        # Delete points of pointcloud if no lidar point is within the distance of the threshold
        dists = cc_pc.compute_point_cloud_distance(lidar_pc)
        dists = np.asarray(dists)

        cls.ind = np.where(dists < cls.threshold)[0]
        cls.cc_pc_reduced = cc_pc.select_by_index(cls.ind)

        if cls.output_path:
            o3d.io.write_point_cloud(cls.output_path , cls.cc_pc_reduced, write_ascii=True)

    @classmethod
    def read_ply_to_dataframe(cls, file_path):
        """
        Reads a PLY file and converts it to a pandas DataFrame.

        Parses the header and data of a PLY file from `file_path`, extracts column names from 
        the header, and creates a DataFrame with the parsed data. The data types are 
        converted where possible. Returns the DataFrame and header lines.

        Parameters:
        ----------
        file_path (str): Path to the PLY file.

        Returns:
        ----------
        df (pd.DataFrame): DataFrame containing the PLY data.
        header_lines (list): List of lines from the PLY header.
        """

        with open(file_path, 'r') as file:
            lines = file.readlines()
        
        header_ended = False
        header_lines = []
        data_lines = []
        
        for line in lines:
            if header_ended:
                data_lines.append(line.strip())
            else:
                header_lines.append(line.strip())
                if line.strip() == "end_header":
                    header_ended = True
    
        # Extract column names from the header
        columns = []
        for line in header_lines:
            if line.startswith("property"):
                columns.append(line.split()[-1])
        
        # Create DataFrame
        data = [line.split() for line in data_lines]
        df = pd.DataFrame(data, columns=columns)
        
        # Convert data types
        for column in df.columns:
            try:
                df[column] = pd.to_numeric(df[column])
            except ValueError:
                pass
        
        return df, header_lines

    @classmethod
    def dataframe_to_ply(cls, df, header_lines, ply_path):
        """
        Writes a pandas DataFrame to a PLY file, including the header.

        Parameters:
        ----------
        df (pd.DataFrame): The DataFrame containing the data to be written.
        header_lines (list): List of header lines from the PLY file.
        ply_path (str): Path where the PLY file will be saved.

        Returns:
        ----------
        None
        """
        with open(ply_path, 'w') as file:
            for line in header_lines:
                file.write(line + '\n')
            
            # Write the data
            for i in range(len(df)):
                row = df.iloc[i]
                # Convert each element to string and format integer values correctly
                row_str = ' '.join([f"{int(val) if isinstance(val, float) and val.is_integer() else val}" for val in row])
                file.write(row_str + '\n')

    @classmethod
    def merge_dataframes_with_header(cls, df1, header_lines1, df2, header_lines2):
        """
        Merges two pandas DataFrames and updates the header from the second DataFrame.

        Identifies new columns in `df2` that are not present in `df1`, filters `df2` based on 
        valid indices stored in `cls.ind`, and updates `df1` and `header_lines1` accordingly. 
        The filtered DataFrame `cls.df2_filtered` is saved to a CSV file for verification.

        Parameters:
        ----------
        df1 (pd.DataFrame): The primary DataFrame to be updated.
        header_lines1 (list): Header lines associated with `df1`.
        df2 (pd.DataFrame): The secondary DataFrame to merge with `df1`.
        header_lines2 (list): Header lines associated with `df2`.

        Returns:
        ----------
        cls.df2_filtered (pd.DataFrame): The filtered version of `df2` based on indices.
        header_lines1 (list): Updated header lines for the merged DataFrame.
        """

        # Identify columns in df2 that are not in df1
        new_columns = [col for col in df2.columns if col not in df1.columns]

        # Ensure 'ind' contains valid indices
        cls.ind = [i for i in cls.ind if i < len(df2)]

        # Filter df2 based on indices in ind
        cls.df2_filtered = df2.iloc[cls.ind]

        #print('Datentyp von df_filtered: ',type(cls.df2_filtered))
        print('Remaining Vertices: ',len(cls.df2_filtered))

        cls.df2_filtered.to_csv('filtered.csv')
        # Add new columns to df1

        # Update header lines
        for line in header_lines2:
            if line.startswith("property") and line.split()[-1] in new_columns:
                header_lines1.insert(-1, line)  # Insert before 'end_header'

        # Update the 'element vertex' line in header_lines1
        for i, line in enumerate(header_lines1):
            if line.startswith("element vertex"):
                header_lines1[i] = f"element vertex {len(cls.ind)}"
                break  # Exit the loop once the line is found and updated
        
        return cls.df2_filtered, header_lines1

    @classmethod
    def change_rgb_and_normal_position(cls, df, insert_index, start_index, end_index, header):
        """
        Repositions specified columns (e.g., RGB or normals) in a pandas DataFrame and updates the header.

        Moves columns from `start_index` to `end_index` to a new position at `insert_index` in the DataFrame.
        The header is updated accordingly by adjusting the order of lines.

        Parameters:
        ----------
        df (pd.DataFrame): The DataFrame containing the columns to be moved.
        insert_index (int): The target index where the columns will be inserted.
        start_index (int): The start index of the columns to be moved.
        end_index (int): The end index (exclusive) of the columns to be moved.
        header (list): The header lines associated with the DataFrame.

        Returns:
        ----------
        df (pd.DataFrame): The DataFrame with the columns repositioned.
        header (list): The updated header reflecting the new column order.
        
        Raises:
        ----------
        ValueError: If the provided indices are invalid.
        """

        # Validate the input indices
        if not (0 <= insert_index <= len(df.columns)) or not (0 <= start_index < end_index <= len(df.columns)):
            raise ValueError("Invalid indices provided.")

        # Extract the columns to be moved
        cols_to_move = df.iloc[:, start_index:end_index].copy()

        # Drop the columns from their original positions
        df = df.drop(df.columns[start_index:end_index], axis=1)

        # Concatenate the columns to be moved at the insert_index
        df = pd.concat([df.iloc[:, :insert_index], cols_to_move, df.iloc[:, insert_index:]], axis=1)

        header = cls.move_lines(header)
        return df, header

    @staticmethod
    def move_lines(lst):
        """
        Moves specified lines in a list to a new position based on a target line.

        Removes predefined lines (`nx`, `ny`, `nz`) from their original positions and 
        inserts them after a specified target line (`property uchar blue`).

        Parameters:
        ----------
        lst (list): The list of header lines to be modified.

        Returns:
        ----------
        lst_copy (list): A new list with the lines moved to their target positions.
        """

        # Define the lines to move and their target position
        lines_to_move = ['property double nx', 'property double ny', 'property double nz']
        target_line = 'property uchar blue'

        # Create a copy of the original list
        lst_copy = lst.copy()

        # Remove the lines to be moved from their original positions in the copy
        for line in lines_to_move:
            if line in lst_copy:
                lst_copy.remove(line)

        # Find the index of the target line
        if target_line in lst_copy:
            target_index = lst_copy.index(target_line)

            # Insert the lines to be moved after the target line
            for line in reversed(lines_to_move):
                lst_copy.insert(target_index + 1, line)

        return lst_copy

    @staticmethod
    def convert_color_columns_to_int(df):
        """
        Converts color columns in a DataFrame to integer type.

        Checks for the existence of 'red', 'green', and 'blue' columns in the DataFrame 
        and converts them to integers if they exist.

        Parameters:
        ----------
        df (pd.DataFrame): The DataFrame containing color columns.

        Returns:
        ----------
        df (pd.DataFrame): The DataFrame with color columns converted to integers.
        """

        # Check if columns exist in the DataFrame
        if 'red' in df.columns:
            df['red'] = df['red'].astype(int)
        if 'green' in df.columns:
            df['green'] = df['green'].astype(int)
        if 'blue' in df.columns:
            df['blue'] = df['blue'].astype(int)
        
        return df
    
    @classmethod
    def execute_3dgsconverter_to3DGS(cls):       
        """
        Executes the 3dgsconverter command with the LIDAR-based cleaned PLY file.

        This function runs the 3dgsconverter tool, passing in the cleaned PLY file,
        output file name, and format arguments. It captures and prints the output,
        and handles any errors during execution.

        Parameters:
        ----------
        None

        Returns:
        ----------
        None

        """


        command = f'3dgsconverter -i {cls.ply_path} -o "output_lidarreduced_{cls.threshold}.ply" -f 3dgs'
        print("Starting execution of 3dgsconverter...")

        # Execute the command and capture the output
        try:
            print("command: ",command)
            result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
            print(f"Command executed successfully:\n{result.stdout}")  # Print the output of the command
        except subprocess.CalledProcessError as e:
            print(f"Error occurred while executing command:\n{e.stderr}")
            print(f"Return code: {e.returncode}")

    @classmethod
    def execute_3dgsconverter_toProcessable(cls):       
        """
        Executes the 3dgsconverter command with the plain GaussianSplat.

        This function runs the 3dgsconverter tool, passing in the trained GaussianSplat

        Parameters:
        ----------
        None

        Returns:
        ----------
        None

        """


        command = f'3dgsconverter -i {cls.pointcloud_path} -o "{cls.pointcloud_path}_cc.ply" -f cc --rgb'
        print("Starting execution of 3dgsconverter to produce cc-format...")

        # Execute the command and capture the output
        try:
            result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
            print(f"Command executed successfully:\n{result.stdout}")  # Print the output of the command
        except subprocess.CalledProcessError as e:
            print(f"Error occurred while executing command:\n{e.stderr}")
            print(f"Return code: {e.returncode}")
        new_path = f'{cls.pointcloud_path}_cc.ply'
        return new_path

    @classmethod
    def start(cls, args):
        """
        Initializes the point cloud processing pipeline based on command line arguments.

        This method sets up various parameters, including threshold and file paths,
        creates a workspace directory if it does not exist, and sequentially calls 
        methods to clean the point cloud, read PLY files, merge DataFrames, 
        rearrange color and normal data, and save the final PLY file.

        Parameters:
        ----------
        args (argparse.Namespace): Command line arguments containing:
            - threshold (float): Distance threshold for filtering points.
            - pointcloud (str): Path to the input point cloud file.
            - lidar (str): Path to the LiDAR point cloud file.
            - result (str): Path to save the final PLY file.
            - workspace (str): Path to the workspace directory.

        Returns:
        ----------
        None
        """
        cls.threshold = args.threshold
        cls.pointcloud_path = args.pointcloud
        cls.lidar_path = args.lidar

        #If File is not in cc format, convert to cc format
        if args.convert_from_gaussiansplat:
            cls.pointcloud_path = cls.execute_3dgsconverter_toProcessable()

        #Manual Alligning Starts
        alligned_lidar = Registration.manual_registration(cls.lidar_path, cls.pointcloud_path)
        cls.lidar_path = alligned_lidar

        #Processing Starts
        print("cleaning starts")

        cls.create_temp_ply()

        cls.delete_points_based_on_lidar()

        #Buliding correct PLY for 3dgsconverter
        print("cleaning succesfull, building PLY")

        df_o3d, header_lines_o3d = cls.read_ply_to_dataframe(cls.output_path)
        df_cc, header_lines_cc = cls.read_ply_to_dataframe(cls.pointcloud_path)
        
        df_merged, header_lines_merged = cls.merge_dataframes_with_header(df_o3d, header_lines_o3d, df_cc, header_lines_cc)
        
        changed_df, header_result = cls.change_rgb_and_normal_position(df_merged, 3, 6, 9, header_lines_merged)
        df_result = cls.convert_color_columns_to_int(changed_df)

        cls.dataframe_to_ply(df_result, header_result, cls.ply_path)
        cls.execute_3dgsconverter_to3DGS()
        cls.delete_temp_ply()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Reduce point cloud based on LIDAR data.")
    parser.add_argument('-r', '--threshold', type=float, required=True, help="Float value for the threshold")
    parser.add_argument('-p', '--pointcloud', type=str, required=True, help="Path to pointcloud.ply")
    parser.add_argument('-l', '--lidar', type=str, required=True, help="Path to LIDAR pointcloud.ply")
    parser.add_argument('--convert_from_gaussiansplat', action='store_true', help="Convert from Gaussian Splat if set")
    args = parser.parse_args()

    Reduce.start(args)


   