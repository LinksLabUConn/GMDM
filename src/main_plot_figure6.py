# Copyright (c) 2025 LINKS Lab UConn. All rights reserved.
# SPDX-License-Identifier: MIT
# Author: James P. Wilson (james.wilson@uconn.edu)

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import math # For sin and cos

def plot_single_path(ax, df):
    """
    Plots a single GMDM path from a DataFrame onto the given axes.
    Segments are colored based on a colormap of velocity, and orientation arrows are drawn.
    """
    # 1. Set up the hard-coded colormap.
    # The colormap transitions from Red (at v=0.3) to Blue (at v=0.65) to Green (at v=1.0).
    colors = [(1, 0, 0), (0, 0, 1), (0, 1, 0)] # Red, Blue, Green
    nodes = [0.0, 0.5, 1.0] # Corresponding nodes for the colors
    cmap = mcolors.LinearSegmentedColormap.from_list("velocity_map", list(zip(nodes, colors)))
    
    # Normalize velocities from 0.3 to 1.0. Values outside this range will be clamped.
    norm = mcolors.Normalize(vmin=0.3, vmax=1.0)


    # 2. Iterate through the points to draw each segment with the correct color
    for i in range(1, len(df)):
        prev_point = df.iloc[i-1]
        curr_point = df.iloc[i]
        
        # Get color from the colormap based on the velocity of the end point
        color = cmap(norm(curr_point['v']))
        
        ax.plot(
            [prev_point['x'], curr_point['x']],
            [prev_point['y'], curr_point['y']],
            color=color,
            linestyle='-',
            linewidth=2
        )

    # 3. Add orientation arrows for the start and end points
    if not df.empty:
        arrow_length = 0.5  # The length of the arrow in plot units
        arrow_color = 'k'   # Black color for arrows for better contrast
        arrow_head_width = 0.15

        # --- Arrow for the first point ---
        first_point = df.iloc[0]
        ax.arrow(
            first_point['x'],
            first_point['y'],
            arrow_length * math.cos(first_point['theta']),
            arrow_length * math.sin(first_point['theta']),
            head_width=arrow_head_width,
            head_length=arrow_head_width * 1.5,
            fc=arrow_color,
            ec=arrow_color,
            linewidth=1.5,
            zorder=10 # Draw arrows on top of other elements
        )

        # --- Arrow for the last point (if there is more than one point) ---
        if len(df) > 1:
            last_point = df.iloc[-1]
            ax.arrow(
                last_point['x'],
                last_point['y'],
                arrow_length * math.cos(last_point['theta']),
                arrow_length * math.sin(last_point['theta']),
                head_width=arrow_head_width,
                head_length=arrow_head_width * 1.5,
                fc=arrow_color,
                ec=arrow_color,
                linewidth=1.5,
                zorder=10
            )

def main():
    """
    Main function to load multiple CSV files and plot them on the same graph.
    """
    # 1. Set up the plot once
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, ax = plt.subplots(figsize=(10, 8))
    

    # 2. Loop through the 8 possible CSV files
    for i in range(8):
        filename = f"results/gmdm_path_states_{i}.csv"
        try:
            # Read the data from the CSV file
            df = pd.read_csv(filename)
            print(f"Successfully loaded {filename}.")
            # Plot this path onto the shared axes
            plot_single_path(ax, df)
        except FileNotFoundError:
            # Silently skip files that don't exist
            continue


    # 3. Finalize the plot after all paths are drawn
    ax.set_title('GMDM Paths Matching Figure 6 in IEEE TRO Paper', fontsize=16)
    ax.set_xlabel('X Coordinate (meters)', fontsize=12)
    ax.set_ylabel('Y Coordinate (meters)', fontsize=12)
    ax.set_aspect('equal', adjustable='box')

    # 4. Show the final plot
    plt.show()

# Run the main function
if __name__ == "__main__":
    main()
