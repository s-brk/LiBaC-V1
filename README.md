# Lidar based Gaussian Cleaner (LiBaC) - Version 1.0
A command linetool to advance gaussian splats with Lidardata
## Usage - Prepping the .ply file for Unreal

3DGS Converter by francesco fugazzi: https://github.com/francescofugazzi/3dgsconverter


**1. Open Anaconda Prompt**

**2. d**:

   ```bash
   d:
   ```

**3. Activate conda environment**:

   ```bash
   conda activate pointcloudedit
   ```

**4. Conversion from 3DGS to Cloud Compare format with RGB addition:**

   ```bash
   3dgsconverter -i input_3dgs.ply -o output_cc.ply -f cc --rgb
   ```

**5. Open in Cloud Compare (import all) and delete Points**

**6. Save as `ASCII`**

**7. Conversion from Cloud Compare format back to 3DGS:**

   ```bash
   3dgsconverter -i input_cc.ply -o output_3dgs.ply -f 3dgs
   ```

**8. Open .ply in Unreal with 3Dgs-Plugin**



## Unreal `.ply` editing
https://www.youtube.com/watch?v=uaWChilLESI

## Unreal Render Workflow

#### Image: https://www.youtube.com/watch?v=bpmO79igrWE

#### Video: https://www.youtube.com/watch?v=GHFq4Dj7sVs

