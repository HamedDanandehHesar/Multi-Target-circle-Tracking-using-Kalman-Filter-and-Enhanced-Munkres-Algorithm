<img width="640" height="480" alt="228" src="https://github.com/user-attachments/assets/47a71571-5bcd-49f2-95e5-ac609e2c9926" />
<img width="640" height="480" alt="185" src="https://github.com/user-attachments/assets/d1864cf1-933e-4435-b88c-2718a918ca7f" />
<img width="640" height="480" alt="348" src="https://github.com/user-attachments/assets/35186605-fc13-4a0d-9fdf-611609d04b93" />

# Multi-Target Circle Tracking using Kalman Filter and Enhanced Munkres Algorithm

This repository provides a MATLAB implementation for **multi-target circle tracking** in image sequences using:

- **Kalman filtering**
- **enhanced data association**
- **Munkres (Hungarian) assignment algorithm**
- **trajectory management for multiple moving objects**

The framework is designed for tracking multiple circular targets across sequential image frames, especially in scenarios where detections may be noisy, ambiguous, or temporarily missing.

---

## Overview

Multi-target tracking is a fundamental problem in computer vision and image analysis.  
When several targets move simultaneously, the tracker must:

- predict the motion of each target,
- associate new detections with existing tracks,
- recover from missed detections,
- handle close interactions or trajectory crossings,
- and maintain consistent target identities over time.

This project addresses these challenges using a **Kalman filter-based tracking framework** combined with an **enhanced Munkres assignment strategy** that incorporates both:

- **Euclidean distance**
- **motion direction consistency**

The implementation supports different motion models, including:

- **velocity-based Kalman model**
- **acceleration-based Kalman model**

---

## Features

- Multi-target tracking in image sequences
- Detection-to-track assignment using the **Munkres/Hungarian algorithm**
- Kalman-based state prediction and correction
- Support for:
  - **constant velocity model**
  - **constant acceleration model**
- Track creation, update, recovery, and deletion
- Trajectory visualization on output frames
- CSV export of tracked object positions
- Frame-by-frame visualization with object IDs
- Robust association using directional penalty to reduce ID switching

---

## Repository Structure

```text
Multi-Target-circle-Tracking-using-Kalman-Filter-and-Enhanced-Munkres-Algorithm/
├── Kalman_filter_Multi_tracking_velocity_model_v3.m
├── Kalman_filter_Multi_tracking_accelleration_model_v3.m
├── input images/
│   ├── 1.jpg
│   ├── 2.jpg
│   ├── 3.jpg
│   └── ...

└── README.md
```

> `input images/` contains the input image sequence, where circular white objects appear on a black background.

---

## Tracking Pipeline

The general processing pipeline is:

1. Load the image sequence
2. Load target detections from `detections.mat`
3. Initialize Kalman filter state vectors for newly detected targets
4. Predict target positions in the next frame
5. Compute the cost matrix between predicted tracks and detections
6. Apply the **Munkres algorithm** for optimal assignment
7. Refine association using motion-direction consistency
8. Update assigned tracks with Kalman correction
9. Increase strike count for unassigned tracks
10. Remove tracks with prolonged missing detections
11. Create new tracks for unmatched detections
12. Draw trajectories and save output frames
13. Export tracking results to CSV

---

## Motion Models

This repository includes two alternative tracking models.

### 1. Velocity Model
Implemented in:

```matlab
Kalman_filter_Multi_tracking_velocity_model_v3.m
```

This model assumes that each target moves with approximately constant velocity.

Typical state form:

```math
x_k =
\begin{bmatrix}
x \\
\dot{x} \\
y \\
\dot{y}
\end{bmatrix}
```

This model is suitable when target motion is smooth and acceleration is limited.

---

### 2. Acceleration Model
Implemented in:

```matlab
Kalman_filter_Multi_tracking_accelleration_model_v3.m
```

This model assumes that the target motion includes acceleration terms.

Typical state form:

```math
x_k =
\begin{bmatrix}
x \\
\dot{x} \\
\ddot{x} \\
y \\
\dot{y} \\
\ddot{y}
\end{bmatrix}
```

This version is more flexible for dynamic targets with changing motion patterns.

---

## Kalman Filter Formulation

The tracking framework is based on the standard linear state-space model:

```math
x_k = A x_{k-1} + w_k
```

```math
z_k = H x_k + v_k
```

where:

- $x_k$ is the hidden target state
- $z_k$ is the observed detection
- $A$ is the state transition matrix
- $H$ is the measurement matrix
- $w_k$ is process noise
- $v_k$ is measurement noise

The Kalman filter operates in two steps:

### Prediction
```math
\hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1}
```

```math
P_{k|k-1} = A P_{k-1|k-1} A^T + Q
```

### Update
```math
y_k = z_k - H \hat{x}_{k|k-1}
```

```math
S_k = H P_{k|k-1} H^T + R
```

```math
K_k = P_{k|k-1} H^T S_k^{-1}
```

```math
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k y_k
```

```math
P_{k|k} = (I - K_k H) P_{k|k-1}
```

---

## Enhanced Data Association

A key part of the tracker is the association between predicted tracks and new detections.

Instead of using only Euclidean distance, this implementation introduces an **enhanced cost function** that also considers the **consistency between predicted velocity direction and detection direction**.

### Cost strategy
For each track-detection pair:

- compute Euclidean distance,
- compute the cosine similarity between:
  - track velocity vector,
  - vector from predicted position to detection,
- penalize assignments that are not aligned with the expected motion direction.

This helps reduce identity switches in cases such as:

- close targets,
- intersecting trajectories,
- ambiguous detections,
- temporary occlusions or missed observations.

---

## Track Management

The tracker maintains a dynamic list of active tracks.

Each track includes:

- state estimate
- covariance matrix
- track ID
- strike count
- class label
- last known position
- trajectory history

### Track lifecycle

#### New Track Creation
If a detection cannot be assigned to an existing track, a new track is created.

#### Track Update
If a detection is assigned successfully, the corresponding Kalman state is corrected.

#### Missed Detection Handling
If no detection is assigned to a track in the current frame, its strike count increases.

#### Track Deletion
If the strike count exceeds a predefined threshold, the track is removed.

#### Track Recovery
Unassigned tracks may be re-associated with nearby detections if conditions are favorable.

---

## Input Data

The code expects:

### 1. Image sequence
A folder containing `.jpg` frames, such as:

```text
input images/
├── 1.jpg
├── 2.jpg
├── 3.jpg
└── ...
```

Images are assumed to be ordered numerically by filename.



## Output

The tracker produces:

- annotated output images
- tracked trajectories drawn on each frame
- target IDs shown next to tracked circles
- a CSV file containing tracking results

Example CSV format:

```text
Frame,Track_ID,Class,X,Y
1,0,target,120,85
1,1,target,240,134
2,0,target,124,87
...
```

---

## How to Run

### Step 1
Place your input frames in the folder:

```text
input images/
```

 
### Step 2
Run one of the following MATLAB scripts depending on the desired motion model:

#### Velocity model
```matlab
Kalman_filter_Multi_tracking_velocity_model_v3
```

#### Acceleration model
```matlab
Kalman_filter_Multi_tracking_accelleration_model_v3
```

### Step 3
Select:

- the input image folder,
- the output folder for saving results,
- the starting frame index.

---

## Main Parameters

Some important parameters in the code include:

- `euclidean_dist_thresh`  
  Distance threshold for track-to-detection assignment

- `max_track_strikes`  
  Maximum allowed number of missed detections before deleting a track

- `sensor_noise`  
  Measurement noise level

- `trajectory_length`  
  Number of previous positions retained for trajectory visualization

- `initial_estimate_covariance`  
  Initial covariance of each new track

- `dt`  
  Time step between frames

These parameters can be tuned depending on:

- target speed
- detection quality
- image resolution
- frame rate
- target density

---

## Applications

This code can be useful in a variety of tracking scenarios, including:

- particle tracking
- cell tracking
- object tracking in microscopy
- circular target motion analysis
- synthetic benchmark tracking
- educational demonstrations of Kalman-based multi-target tracking

---

## Notes

- The input images in this repository consist of **white circular objects on a black background**.
- The detections are assumed to be available beforehand in `detections.mat`.
- The implementation is focused on **tracking**, not on the detection stage itself.
- The enhanced assignment step is particularly helpful when targets move close to one another.

---

## Requirements

- MATLAB
- Computer Vision Toolbox (for functions such as `insertShape`, `insertText`)
- A MATLAB implementation of the **Munkres/Hungarian algorithm**


## Acknowledgment

This repository was developed for multi-target tracking experiments using Kalman filtering and robust assignment methods for circular object trajectories in image sequences.
```
