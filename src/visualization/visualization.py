import xml.etree.ElementTree as ET
import matplotlib
import matplotlib.pyplot as plt
import json
import numpy as np
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import pandas as pd
import SumoNetVis
import imageio, os
from IPython.display import display, Image


def read_density_file_and_gen_colormap(density_file):
    """Computes a consistent colormap across all time frames."""
    df = pd.read_csv(density_file)
    min_density, max_density = df.iloc[:, 1:].min().min(), df.iloc[:, 1:].max().max()
    norm = mcolors.Normalize(vmin=min_density, vmax=max_density)
    # cmap = plt.get_cmap("RdYlGn_r")
    cmap = plt.get_cmap("viridis")
    return df, cmap, norm
    
def read_lanewise_density_file_and_gen_colormap(lane_density_file):
    """Reads a lane-wise density file and generates a consistent colormap across all time frames."""
    tree = ET.parse(lane_density_file)
    root = tree.getroot()

    # Extract lane density values over time
    time_series_density = []
    all_lanes = set()
    
    for interval in root.findall("interval"):
        time_step = {}
        for edge in interval.findall("edge"):
            edge_id = edge.get("id")
            density = float(edge.get("laneDensity", 0))
            time_step[edge_id] = density
            all_lanes.add(edge_id)
        
        time_series_density.append(time_step)
    
    # Convert to DataFrame for easier processing
    df = pd.DataFrame(time_series_density)
    df = df.fillna(0)  # Fill missing lane densities with zero

    # Compute min/max density for colormap
    min_density, max_density = df.min().min(), df.max().max()
    # print(f"min_density = {min_density}, max_density={max_density}")
    norm = mcolors.Normalize(vmin=min_density, vmax=max_density)
    # cmap = plt.get_cmap("RdYlGn_r")  # Green (low) to Red (high congestion)
    cmap = plt.get_cmap("inferno")
    minval = 0.0
    maxval = 0.6
    n = 256
    cmap = mcolors.LinearSegmentedColormap.from_list(f'trunc({cmap.name},{minval:.2f},{maxval:.2f})',
        cmap(np.linspace(minval, maxval, n))
    )
    
    return df, cmap, norm

    
def get_lane_coord(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    lane_coords = {}
    
    for edge in root.findall("edge"):
            edge_id = edge.get("id")
            # print(edge_id)
            for lane in edge.findall("lane"):
                shape = lane.get("shape")
                if shape:
                    points = [tuple(map(float, p.split(','))) for p in shape.split()]
                    lane_coords[edge_id] = points
    
    return lane_coords

def get_actuator_coord(json_file):
    with open(json_file, "r") as f:
        data = json.load(f)
    
    actuator_coords = data['actuators']
    
    return actuator_coords
    
def get_lane_region(regions_file):
    with open(regions_file, "r") as f:
        regions = json.load(f)
    
    lane_regions = {}
    for region, edges in regions.items():
        if region in ['0', '1', '2', '3', '4']:
            for edge in edges:
                lane_regions[edge] = int(region)
    
    return lane_regions

def get_region_density_and_color(density_data, cmap, norm):
    """Maps region densities to colors using a consistent colormap."""
    region_density_color = []
    for region, density in density_data.items():
        color = cmap(norm(density))
        region_density_color.append((density, color))
    return region_density_color

def get_lane_color_from_region_density(lane_coords, lane_regions, region_density_color):
    lane_colors = {}
    
    for lane_id, coords in lane_coords.items():
        region = lane_regions.get(lane_id, None)
        if region is not None:
            _, color = region_density_color[region]
        else:
            color = 'black'  # Default color for undefined lanes
        
        lane_colors[lane_id] = color
    
    return lane_colors

def get_lane_color_from_lane_density(lane_coords, lane_density, cmap, norm):
    """Assigns colors to lanes based on their density values."""
    lane_colors = {}
    
    for lane_id, coords in lane_coords.items():
        density = lane_density.get(lane_id, 0)  # Default to zero if lane not found
        lane_colors[lane_id] = cmap(norm(density))
    
    return lane_colors

def get_lane_color_from_region(lane_coord, actuator_coord, lane_regions, region_colormap):
    lane_colors = {}
    
    for lane_id in lane_coord.keys():
        if (lane_id in lane_regions) & (lane_id not in actuator_coord):
            lane_region = lane_regions[lane_id]
            lane_colors[lane_id] = region_colormap[lane_region]
        else:
            lane_colors[lane_id] = 'black'
    
    return lane_colors
    
# def plot_lanes_single_time(xml_file, regions_file, density_data):
#     lane_coords = get_lane_coord(xml_file)
#     lane_regions = get_lane_region(regions_file)
#     region_density_color = get_region_density_and_color(density_data)
#     print("Plotting: ", density_data)
#     lane_colors = get_lane_color(lane_coords, lane_regions, region_density_color)
    
#     plt.figure(figsize=(10, 10))
    
#     for lane_id, coords in lane_coords.items():
#         x, y = zip(*coords)
#         plt.plot(x, y, marker='o', linestyle='-', linewidth=3, color=lane_colors[lane_id])
    
#     plt.axis("equal")
#     plt.grid(True)
#     plt.show()

def plot_lanes_time_series_images(xml_file, regions_file, region_density_file, lane_density_file, output_folder, title=None):
    # Create output foler
    os.makedirs(output_folder, exist_ok=True)  # Ensure output directory exists

    # Get lane coordinates and regions
    lane_coords = get_lane_coord(xml_file)

    # For region data
    useRegion = False
    if useRegion:
        lane_regions = get_lane_region(regions_file)
        df, cmap, norm = read_density_file_and_gen_colormap(region_density_file)
    else:
        df, cmap, norm = read_lanewise_density_file_and_gen_colormap(lane_density_file)

    for frame in range(len(df)):
        print(f"Generating image {frame+1}/{len(df)}",end='\r')

        if useRegion:
            density_data = df.iloc[frame, 1:].to_dict()
            region_density_color = get_region_density_and_color(density_data, cmap, norm) # region to color map
            lane_colors = get_lane_color_from_region_density(lane_coords, lane_regions, region_density_color)
        else:
            lanewise_density_data = df.iloc[frame, 1:].to_dict()  # Skip first column if it's a timestamp
            lane_colors = get_lane_color_from_lane_density(lane_coords, lanewise_density_data, cmap, norm)

        plt.figure(figsize=(5, 5))
        for lane_id, coords in lane_coords.items():
            x, y = zip(*coords)
            # print(f"Lane {lane_id}: {x}, {y}")
            # if lane upwards, move to right, else move to left
            if coords[0][1] < coords[-1][1]:
                x = [i + 6 for i in x]
            elif coords[0][1] > coords[-1][1]:
                x = [i - 6 for i in x]
            # if lane from left to right, move to dpwn, else move to up
            if coords[0][0] > coords[-1][0]:
                y = [i + 9 for i in y]
            elif coords[0][0] < coords[-1][0]:
                y = [i - 9 for i in y]
            plt.plot(x, y, marker='', linestyle='-', linewidth=2, color=lane_colors[lane_id])

        # print(min(x_coords), max(x_coords), min(y_coords), max(y_coords))
        # plot nodes only once
        for i in np.linspace(-1.6, 1401.6, 8):
            for j in np.linspace(-1.6, 1401.6, 8):
                plt.plot(i, j, marker='o', color='k', markersize=8)

        plt.title(title)
        # plt.title(f"SUMO Road Network - Frame {frame}")
        x_ticks = plt.xticks()[0]
        y_ticks = plt.yticks()[0]
        x_labels = ['', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I']
        y_labels = [str(i-1) for i in range(len(y_ticks))]

        plt.xticks(x_ticks, x_labels)
        plt.yticks(y_ticks, y_labels)
        plt.axis("equal")  # Keep aspect ratio
        # plt.show()

        output_path = os.path.join(output_folder, f"frame_{frame:03d}.png")
        plt.savefig(output_path)
        plt.close()

    print(f"All {len(df)} images saved to {output_folder}")
    return

def cocoCity_plot_generate_density_gif(output_dir, output_gif_path):
    '''
    Wrapper function to generate density heatmap for the cocoCity project
    Returns cmap, norm for plotting colorbar legend
    '''
    # Hard-coded constants !!!
    net_file = "dep/sumo_files/cocoCity/network/cocoCity.net.xml"
    regions_file = "dep/sumo_files/cocoCity/network/regions.json"
    actuators_file = "dep/sumo_files/cocoCity/control/edge/edge.json"


    region_density_file = ""
    lane_density_file = output_dir+"/edge_data_new.xml"
    output_image_dir = output_dir+"/img"
    
    
    plot_lanes_time_series_images(net_file, regions_file, region_density_file, lane_density_file, output_image_dir, title="COCO-City road network density")
    create_gif_from_images(output_image_dir, output_gif_path)
    df, cmap, norm = read_lanewise_density_file_and_gen_colormap(lane_density_file)
    return cmap, norm 

def cocoCity_plot_static_city_map():
    net_file = "dep/sumo_files/cocoCity/network/cocoCity.net.xml"
    regions_file = "dep/sumo_files/cocoCity/network/regions.json"
    actuators_file = "dep/sumo_files/cocoCity/control/edge/edge.json"

    lane_coords = get_lane_coord(net_file)
    lane_regions = get_lane_region(regions_file)
    actuator_coords = get_actuator_coord(actuators_file)
    region_colormap =  {0:'red', 1:'blue', 2:'green', 3:'purple', 4:'orange'}
    lane_colors = get_lane_color_from_region(lane_coords, actuator_coords, lane_regions, region_colormap)
    COCOcityImage = plot_lanes(lane_coords, actuator_coords, lane_colors, region_colormap, title="COCO-City traffic network")

    COCOcityImage.savefig("COCOcityImage.png", dpi=300, bbox_inches='tight')
    return
    
def create_gif_from_images(image_folder, output_gif, duration=0.1):
    images = []
    file_list = sorted([img for img in os.listdir(image_folder) if img.endswith(".png")])

    for filename in file_list:
        img_path = os.path.join(image_folder, filename)
        images.append(imageio.imread(img_path))

    imageio.mimsave(output_gif, images, duration=duration)
    print(f"GIF saved as {output_gif}")


def plot_color_legend(cmap, norm):
    """Displays the color legend for the heatmap without extra white space."""
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.subplots_adjust(bottom=0.5)  # Adjust layout to fit only colorbar

    cbar = fig.colorbar(sm, ax=ax, orientation='horizontal', label='Traffic Density [# vehicles/km]')
    cbar.ax.tick_params(labelsize=10)  # Adjust tick size for clarity
    ax.remove()  # Remove the extra Axes

    plt.show()

def plot_lanes(lane_coord, actuator_coord, lane_colors, region_colormap, title=None):
    fig = plt.figure(figsize=(5, 5))

    actuator_dict = {actuator_id: i for i, actuator_id in enumerate(actuator_coord)}

    for lane_id in lane_coord:
        ## Offset bi-directional lanes
        coords = lane_coord[lane_id]
        x, y = zip(*lane_coord[lane_id])
        if coords[0][1] < coords[-1][1]:
            x = [i + 6 for i in x]
        elif coords[0][1] > coords[-1][1]:
            x = [i - 6 for i in x]
        # if lane from left to right, move to down, else move to up
        if coords[0][0] > coords[-1][0]:
            y = [i + 9 for i in y]
        elif coords[0][0] < coords[-1][0]:
            y = [i - 9 for i in y]
        if lane_id not in actuator_coord:
            # print(lane_coord[lane_id])
            color = lane_colors[lane_id]
            # print(x,y, color)
            plt.plot(x, y, marker='o', linestyle='-', color=color)
        else:
            coords = lane_coord[lane_id]
            plt.plot(x, y, marker='o', linestyle='-', color='black', linewidth=3, zorder=10)

            # add text saying the actuator_dict[lane_id]
            actuator_id = actuator_dict[lane_id]
            # make sure to print it in the middle of the line and slightly off set
            mid_x = (x[0] + x[-1]) / 2
            mid_y = (y[0] + y[-1]) / 2
            plt.text(mid_x+ 10, mid_y, str(actuator_id), fontsize=10, color='black', ha='left', va='bottom')

    plt.title(title)
    x_ticks = plt.xticks()[0]
    y_ticks = plt.yticks()[0]
    x_labels = ['', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I']
    y_labels = [str(i-1) for i in range(len(y_ticks))]

    # plot legend of region colors
    for region, color in region_colormap.items():
        plt.plot([], [], color=color, label=f"Region {region}")
    plt.plot([], [], linewidth=3, color='black', label="Actuator Lanes")
    # black are for the actuators
    # for actuator in actuator_coord:
    #     plt.plot([], [], color='black', label=f"Actuator {actuator}")
    fig.legend(loc='outside lower center', fontsize=10, bbox_to_anchor=(0.5, -0.05), ncol=5)

    plt.xticks(x_ticks, x_labels)
    plt.yticks(y_ticks, y_labels)
    plt.axis("equal")  # Keep aspect ratio
    plt.show()

    return fig
