import pandas as pd
import open3d as o3d
import numpy as np
import argparse
import os

class Reduce:
    threshold = 0.1

    workspace_patch=None
    pointcloud_path = None
    lidar_path = None
    output_path = None #Hier sind dann nur noch die Punkte im Umkreis vom Threshold vom Lidar drin
    ply_path = None 

    saving_path = None

    

    cc_pc_reduced = None
    ind = None
    df2_filtered = None


    @classmethod
    def delete_points_based_on_lidar(cls):
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
    def save_dataframe_to_csv(cls, df, csv_path):
        df.to_csv(csv_path, index=False)

    @classmethod
    def dataframe_to_ply(cls, df, header_lines, ply_path):
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

        '''
        for col in new_columns:
            df1[col] = cls.df2_filtered[col]
        '''
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
        # Validate the input indices
        if not (0 <= insert_index <= len(df.columns)) or not (0 <= start_index < end_index <= len(df.columns)):
            raise ValueError("Invalid indices provided.")

        # Extract the columns to be moved
        cols_to_move = df.iloc[:, start_index:end_index].copy()

        #print("to be moved", cols_to_move)

        # Drop the columns from their original positions
        df = df.drop(df.columns[start_index:end_index], axis=1)

        # Concatenate the columns to be moved at the insert_index
        df = pd.concat([df.iloc[:, :insert_index], cols_to_move, df.iloc[:, insert_index:]], axis=1)

        header = cls.move_lines(header)
        return df, header

    @staticmethod
    def move_lines(lst):
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
        # Check if columns exist in the DataFrame
        if 'red' in df.columns:
            df['red'] = df['red'].astype(int)
        if 'green' in df.columns:
            df['green'] = df['green'].astype(int)
        if 'blue' in df.columns:
            df['blue'] = df['blue'].astype(int)
        
        return df

    @classmethod
    def start(cls, args):
        cls.threshold = args.threshold
        cls.pointcloud_path = args.pointcloud
        cls.lidar_path = args.lidar

        if args.workspace:
            cls.output_path = args.workspace
        else:
            cls.output_path = os.path.join(os.path.dirname(__file__), 'workspace')
        
        if not os.path.exists(cls.output_path):
            os.makedirs(cls.output_path)

        cls.ply_path = args.result

        #Processing Starts
        print("Cleaning Starts")

        cls.delete_points_based_on_lidar()

        #Buliding correct PLY for 3dgsconverter
        print("cleaning succesful, building PLY")

        df_o3d, header_lines_o3d = cls.read_ply_to_dataframe(cls.output_path)
        df_cc, header_lines_cc = cls.read_ply_to_dataframe(cls.pointcloud_path)
        #df_o3d.to_csv('current_o3d.csv')
        #df_cc.to_csv('current_cc.csv')
        
        df_merged, header_lines_merged = cls.merge_dataframes_with_header(df_o3d, header_lines_o3d, df_cc, header_lines_cc)
        #df_merged.to_csv(os.path.join(cls.output_path, 'current_merged.csv'))
        
        changed_df, header_result = cls.change_rgb_and_normal_position(df_merged, 3, 6, 9, header_lines_merged)
        df_result = cls.convert_color_columns_to_int(changed_df)
        #Saving PLY
        print("saving PLY as", cls.ply_path)
        cls.dataframe_to_ply(df_result, header_result, cls.ply_path)

        #cls.saving_path = os.path.join(os.path.dirname(cls.ply_path), f'Lidar_Reduced_{cls.threshold}.ply')
        
        print("done, use 3dgsconverter")

        print(f'Navigate to destination and in anaconda prompt Command: 3dgsconverter -i {cls.ply_path} -o "output_lidarreduced_{cls.threshold}.ply" -f 3dgs')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Reduce point cloud based on LIDAR data.")
    parser.add_argument('-r', '--threshold', type=float, required=True, help="Float value for the threshold")
    parser.add_argument('-p', '--pointcloud', type=str, required=True, help="Path to pointcloud.ply")
    parser.add_argument('-l', '--lidar', type=str, required=True, help="Path to LIDAR pointcloud.ply")
    parser.add_argument('-o', '--result', type=str, required=True, help="Path to save the final PLY file")
    parser.add_argument('-w', '--workspace', type=str, help="Path to workspace directory (optional)")
    args = parser.parse_args()

    Reduce.start(args)


   