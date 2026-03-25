import pandas as pd
import ast
import numpy as np
import math
from scipy.spatial.distance import cdist

# Parallax & FOV Correction Function
def map_yolo_to_thermal(yolo_x, yolo_y, yolo_h, 
                        rgb_width=640, rgb_height=480, 
                        thermal_width=320, thermal_height=240):
    
    # FOV Specifications (120 -> 102 * 67 for Picam3 Wide, 133 -> 110 * 75 for ThermalCam)
    rgb_fov_h = math.radians(102) 
    rgb_fov_v = math.radians(67)  
    therm_fov_h = math.radians(110) 
    therm_fov_v = math.radians(75)  
    
    # Physical Offsets (Let the PiCam3 is origin)
    OFFSET_X_CM = -1.7  
    OFFSET_Y_CM = -0.7  

    REAL_HEIGHT_CM = 170.0
    
    rgb_fy = (rgb_height / 2) / math.tan(rgb_fov_v / 2)
    rgb_fx = (rgb_width / 2) / math.tan(rgb_fov_h / 2)
    
    # Estimate Z distance based on bounding box height
    if yolo_h > 0:
        distance_z_cm = (REAL_HEIGHT_CM * rgb_fy) / yolo_h
    else:
        distance_z_cm = 200 

    # 2D to 3D
    centered_rgb_x = yolo_x - (rgb_width / 2)
    centered_rgb_y = yolo_y - (rgb_height / 2)
    
    real_x_cm = (centered_rgb_x * distance_z_cm) / rgb_fx
    real_y_cm = (centered_rgb_y * distance_z_cm) / rgb_fy

    # Apply physical translation
    therm_real_x_cm = real_x_cm + OFFSET_X_CM
    therm_real_y_cm = real_y_cm + OFFSET_Y_CM

    # 3D back to 2D
    therm_fy = (thermal_height / 2) / math.tan(therm_fov_v / 2)
    therm_fx = (thermal_width / 2) / math.tan(therm_fov_h / 2)
    
    therm_pixel_x = (therm_real_x_cm * therm_fx) / distance_z_cm
    therm_pixel_y = (therm_real_y_cm * therm_fy) / distance_z_cm
    
    final_thermal_x = therm_pixel_x + (thermal_width / 2)
    final_thermal_y = therm_pixel_y + (thermal_height / 2)
    
    return final_thermal_x, final_thermal_y


# Main Accuracy Evaluation
def calculate_accuracy(csv_file):
    df = pd.read_csv(csv_file)
    
    df['thermal_data'] = df['thermal_data'].apply(ast.literal_eval)
    df['yolo_data'] = df['yolo_data'].apply(ast.literal_eval)

    # Difference between the number of people detected -> mean average error
    df['count_diff'] = abs(df['thermal_count'] - df['yolo_count'])
    mae_count = df['count_diff'].mean()

    all_distances = []
    
    # Tracking variables for ML metrics
    TP = 0 # True Positive (Matched)
    FP = 0 # False Positive (Ghost)
    FN = 0 # False Negative (Miss)

    for index, row in df.iterrows():
        thermal_pts = []
        yolo_pts = []
        
        # Extract Thermal Centroids
        for t in row['thermal_data']:
            thermal_pts.append([t['x'], t['y']])
            
        # Extract YOLO Centroids & Apply Parallax Correction
        for y in row['yolo_data']:
            cx = (y['x1'] + y['x2']) / 2 
            cy = (y['y1'] + y['y2']) / 2
            h = abs(y['y2'] - y['y1']) # Bounding box height
            
            # Use the advanced 3D projection instead of simple 0.5 scaling
            mapped_x, mapped_y = map_yolo_to_thermal(cx, cy, h)
            yolo_pts.append([mapped_x, mapped_y])
            
        # ---------------------------------------------------------
        # Greedy Matching Algorithm for TP / FP / FN
        # ---------------------------------------------------------
        temp_yolo = list(yolo_pts)
        temp_thermal = list(thermal_pts)
        
        # While there are still points in BOTH lists, find the closest pair
        while temp_yolo and temp_thermal:
            dists = cdist(temp_yolo, temp_thermal, metric='euclidean')
            min_idx = np.argmin(dists)
            
            # Convert flat index back to 2D row/col indices
            y_idx, t_idx = np.unravel_index(min_idx, dists.shape)
            
            # Log the distance of this matched pair
            all_distances.append(dists[y_idx, t_idx])
            TP += 1
            
            # Remove the matched pair from the temporary lists
            temp_yolo.pop(y_idx)
            temp_thermal.pop(t_idx)
            
        # Any YOLO points left over are Misses (False Negatives)
        FN += len(temp_yolo)
        # Any Thermal points left over are Ghosts (False Positives)
        FP += len(temp_thermal)

    # Calculate Final Metrics
    avg_pixel_offset = np.mean(all_distances) if all_distances else 0
    
    # Protect against Division by Zero
    precision = (TP / (TP + FP)) if (TP + FP) > 0 else 0.0
    recall = (TP / (TP + FN)) if (TP + FN) > 0 else 0.0
    f1_score = (2 * precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

    # Display Results
    print(f"\n\nFrom {len(df)} timestamps in {csv_file}")
    print("-" * 35)
    print(f"1. COUNT ACCURACY")
    print(f"   - Mean Absolute Error: {mae_count:.2f} people")
    print(f"   - True Positives (Matched): {TP}")
    print(f"   - Thermal Ghost Detections (FP): {FP}")
    print(f"   - Thermal Missed Detection (FN): {FN}")
    print("-" * 35)
    print(f"2. MODEL PERFORMANCE")
    print(f"   - Precision: {precision:.2f} ")
    print(f"   - Recall:    {recall:.2f}  ")
    print(f"   - F1 Score:  {f1_score:.2f} ")
    print("-" * 35)
    print(f"3. SPATIAL ACCURACY (Parallax Corrected)")
    print(f"   - Avg. Distance Offset: {avg_pixel_offset:.2f} pixels\n\n")

if __name__ == "__main__":
    calculate_accuracy('detections.csv')
