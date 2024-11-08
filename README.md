# Lidar based Gaussian Cleaner (LiBaC) - Version 1.0
A command linetool to advance gaussian splats with Lidardata


## Software Requirements
3DGS Converter by francesco fugazzi: https://github.com/s-brk/3dgsconverter.git

CloudCompare v2.13.0

### Python Packages:


| Package                  | Version   |
|--------------------------|-----------|
| asttokens                | 2.4.1     |
| attrs                    | 23.2.0    |
| backcall                 | 0.2.0     |
| blinker                  | 1.8.2     |
| certifi                  | 2024.6.2  |
| charset-normalizer       | 3.3.2     |
| click                    | 8.1.7     |
| colorama                 | 0.4.6     |
| dash                     | 2.17.1    |
| dash-core-components     | 2.0.0     |
| dash-html-components     | 2.0.0     |
| dash-table               | 5.0.0     |
| debugpy                  | 1.6.7     |
| decorator                | 5.1.1     |
| executing                | 2.0.1     |
| fastjsonschema           | 2.20.0    |
| flask                    | 3.0.3     |
| gsconverter              | 0.2       |
| idna                     | 3.7       |
| importlib-metadata       | 8.0.0     |
| ipykernel                | 6.29.5    |
| ipython                  | 8.12.3    |
| ipywidgets               | 8.1.3     |
| itsdangerous             | 2.2.0     |
| jinja2                   | 3.1.4     |
| joblib                   | 1.4.2     |
| jsonschema               | 4.22.0    |
| jupyter_client           | 8.6.2     |
| jupyter_core             | 5.7.2     |
| jupyterlab-widgets       | 3.0.11    |
| matplotlib-inline        | 0.1.7     |
| numpy                    | 1.24.4    |
| open3d                   | 0.18.0    |
| pandas                   | 2.0.3     |
| plotly                   | 5.22.0    |
| plyfile                  | 1.0.3     |
| psutil                   | 5.9.0     |
| pyarrow                  | 16.1.0    |
| python-dateutil          | 2.9.0     |
| requests                 | 2.32.3    |
| scikit-learn             | 1.3.2     |
| scipy                    | 1.10.1    |
| stack-data               | 0.6.3     |
| tornado                  | 6.2       |
| traitlets                | 5.14.3    |
| urllib3                  | 2.2.2     |
| werkzeug                 | 3.0.3     |
| widgetsnbextension       | 4.0.11    |

     
## Instructions 

**1. Preperation**

Download and install 3DGSconverter from our forked repository.

To make use of our tool you have to prepare a gaussian splat and a Lidar-Scan-Pointcloud of the same scene in a CloudCompare readable format. You archive this, by converting the trained gaussian splat with the 3dgsconverter before importing it to CloudCompare and the Lidarscan by just importing the PLY with CloudCompare. At current state it is needed to manually allign both representations with four identical points inside CloudCompare.

Export the alligned Pointclouds seperatly from CloudCompare as ASCII.

**2. Clone LiBaC-V1 repository**

**3. Activate conda environment (if used)**

**4. Navigate to repository folder**
```bash
   cd \directory\yourFolder\LiBaC-V1
   ```

**5. execute script**

| flag              | explained |
|-------------------|-----------|
| -r                | *float*  - the distance threshold, which defines the maximum distance of the GaussianSplat points from the Lidar-Scan (start with 1, check the result and iterate)     |
| -p                | *string* - the path to the GaussianSplat- PointCloud converted for CloudCompare   |
| -l                |*string* - the path to the alligned Lidarscan converted for CloudCompare     |

**6. import the result in Unreal and make a joyful noise**


### For testing we provide two alligned pointclouds

These are allready alligned and in the correct format. We invite you to play with the Radius to archive the best result.

Link to MI-Cloud: https://im-www.hdm-stuttgart.de/nextcloud/index.php/s/FMT3MT5F2bQ69Zz

Password: Gaussian24?
