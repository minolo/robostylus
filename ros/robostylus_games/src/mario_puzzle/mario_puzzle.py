#!/usr/bin/env python

import rospy

import itertools
import math
import cv2
import cv_bridge
import numpy
import collections
import os
import datetime

import rospkg

from sensor_msgs.msg import Image
from robostylus_planner.msg import Point2D

# Cell info class
class Cell:
    def __init__(self, type, accurate, x, y):
        self.type = type
        self.accurate = accurate
        self.x = x
        self.y = y
    def __repr__(self):
        return str(self.type)

# Puzzle cell size in pixels
CELL_SIZE = 60

# Path to game package
PACKAGE_PATH = rospkg.RosPack().get_path('robostylus_games') + "/src/mario_puzzle/"

# Path to neural network
NEURAL_NETWORK_FILENAME = "nn"
NET_PATH = PACKAGE_PATH + NEURAL_NETWORK_FILENAME

# Path to the directory where inaccurate cells will be saved
INACCURATE_PATH = PACKAGE_PATH + "inaccurate/"

# Create directories for neural network data
if not os.path.exists(INACCURATE_PATH):
    os.makedirs(INACCURATE_PATH)

# Create neural network and load from file
net = cv2.ANN_MLP()
net.load(NET_PATH)

# All combinations of the elements of an array up to num elements per combination
def powerset(array, num):
    return list(itertools.chain.from_iterable(itertools.combinations(array, n) for n in xrange(num + 1)))

# Paint a cell depending on its accuracy
colors = [(255, 0, 0), (0, 255, 0)]
def paint_cell(image, cell):
    if cell.accurate:
        cv2.circle(image, (cell.x, cell.y), 10, colors[cell.type], 4)
    else:
        cv2.circle(image, (cell.x, cell.y), 18, (255, 255, 0), 4)

######################################################################
# SOLVE PUZZLE
######################################################################
def solve_puzzle(top_grid, bottom_grid):

    # Calculate offsets for adjacent cells in 2 dimensions
    adjacent_offsets = [(x,y) for (x,y) in itertools.product(xrange(-1,2), xrange(-1,2))]

    # Calculate differences between both grids
    differences_vector = [top ^ bottom for (top, bottom) in zip(top_grid, bottom_grid)]

    # Calculate grid size
    grid_size = int(math.sqrt(len(differences_vector)))
    size_range = xrange(grid_size)
    
    # Create the list of all candidate solutions to the puzzle
    all_cells = list(itertools.product(size_range, size_range))
    all_solutions = powerset(all_cells, 3)

    # Check every possible solution
    for toggle_cells in all_solutions:
        # Generate the list of positions that are toggled
        toggle_positions = [(i+x,j+y) for (x,y) in adjacent_offsets for (i,j) in toggle_cells]
        toggle_positions = [(x,y) for (x,y) in toggle_positions if x >= 0 and y >= 0]
        toggle_positions = [(x,y) for (x,y) in toggle_positions if x < grid_size and y < grid_size]

        # Remove even number of occurrences of a position, as they cancel each other
        counter = collections.Counter(toggle_positions)
        toggle_positions_odd = [position for position,count in counter.iteritems() if count % 2 != 0]

        # Create the final list of positions that get toggled
        toggle_vector = [1 if (x,y) in toggle_positions_odd else 0 for (x,y) in all_cells]

        # If the list coincides with the differences vector, the solution has been found, as toggling them would make them all 0
        if toggle_vector == differences_vector:
            return [1 if (x,y) in toggle_cells else 0 for (x,y) in all_cells]

######################################################################
# DETECT PUZZLE GRID
######################################################################
def detect_puzzle_grid(image):

    ##################################
    # IMAGE TRANSFORMATIONS
    ##################################
    # Convert to grayscale
    image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    
    # Apply gaussian blur
    image_gray = cv2.GaussianBlur(image_gray, (7,7), 0)

    # Adaptive histogram equalization
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(13,13))
    image_eq = clahe.apply(image_gray)

    # Binarize
    image_bin = cv2.adaptiveThreshold(image_eq, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 61, 0)

    # Mask image margins
    width_margin = 0.13
    height_margin = 0.01

    image_height = image.shape[0]
    image_width = image.shape[1]

    left_margin = int(width_margin * image_width)
    right_margin = int((1-width_margin) * image_width)
    top_margin = int(height_margin * image_height)
    bottom_margin = int((1-height_margin) * image_height)

    image_bin_masked = numpy.zeros(image_gray.shape, numpy.uint8)
    image_bin_masked[top_margin:bottom_margin, left_margin:right_margin] = image_bin[top_margin:bottom_margin, left_margin:right_margin]

    # Open
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(81,81))
    image_open = cv2.morphologyEx(image_bin_masked, cv2.MORPH_OPEN, element)

    # Dilate
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    image_dilate = cv2.dilate(image_open, element)

    ##################################
    # FIND CELL CENTROIDS
    ##################################
    # Get contours
    image_contours = image_dilate.copy()
    contours, hierarchy = cv2.findContours(image_contours, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Compute convex hulls
    contours = [cv2.convexHull(cnt) for cnt in contours]

    # Check that there is at least one contour
    if len(contours) == 0:
        return None, None

    # Remove biggest contour (the one surrounding all the cells)
    contour_areas = [cv2.contourArea(contour) for contour in contours]
    index_max = contour_areas.index(max(contour_areas))
    contours.pop(index_max)

    # Check if the number of contours is a perfect square and one of the possible sizes in this puzzle
    num_grid_contours = len(contours)
    grid_size = math.sqrt(abs(num_grid_contours))

    if not grid_size.is_integer() or grid_size not in [2,3,4]:
        return None, None

    grid_size = int(grid_size)

    # Check if convex hulls have four edges
    poly = [cv2.approxPolyDP(curve, cv2.arcLength(curve, True) * 0.02, True) for curve in contours]

    if any([len(p) != 4 for p in poly]):
        return None, None

    # Collect poly centroids
    moments = [cv2.moments(p) for p in poly]
    centroids_x = [int(M['m10'] / M['m00']) for M in moments]
    centroids_y = [int(M['m01'] / M['m00']) for M in moments]
    centroids = list(zip(centroids_x, centroids_y))

    # Get corner centroids
    foo = [sum(c) for c in centroids]
    foo_min_index = foo.index(min(foo))
    foo_max_index = foo.index(max(foo))

    centroid_top_left = centroids[foo_min_index]
    centroid_bottom_right = centroids[foo_max_index]

    # Get distances between corner centroids
    distance_x = abs(centroid_top_left[0] - centroid_bottom_right[0]) / (grid_size - 1)
    distance_y = abs(centroid_top_left[1] - centroid_bottom_right[1]) / (grid_size - 1)

    # Extrapolate to calculate all centroids (itertools.product is transposed with (j, i) to avoid the need of a matrix transpose later)
    range_squares = [(i - 1) for i in xrange(grid_size + 2)]

    extended_centroids = [(centroid_top_left[0] + distance_x * i, centroid_top_left[1] + distance_y * j) for (j, i) in itertools.product(range_squares, range_squares)]

    # Fix centroid positions so cells fall completely inside the image
    extended_centroids = [(max(CELL_SIZE/2, x), max(CELL_SIZE/2, y)) for (x, y) in extended_centroids]
    extended_centroids = [(min(image_width - CELL_SIZE/2 - 1, x), min(image_height - CELL_SIZE/2 - 1, y)) for (x, y) in extended_centroids]

    ##################################
    # EXTRACT CELLS
    ##################################
    # Gather cell coordinates
    cell_coords_min_y = [(centroid_y - CELL_SIZE/2) for (centroid_x, centroid_y) in extended_centroids]
    cell_coords_max_y = [(centroid_y + CELL_SIZE/2) for (centroid_x, centroid_y) in extended_centroids]
    cell_coords_min_x = [(centroid_x - CELL_SIZE/2) for (centroid_x, centroid_y) in extended_centroids]
    cell_coords_max_x = [(centroid_x + CELL_SIZE/2) for (centroid_x, centroid_y) in extended_centroids]
    cell_coords = zip(cell_coords_min_y, cell_coords_max_y, cell_coords_min_x, cell_coords_max_x)

    # Split cell images and equalize
    cell_images = [image_gray[min_y:max_y,min_x:max_x] for (min_y,max_y,min_x,max_x) in cell_coords]
    cell_equalized = [cv2.equalizeHist(cell) for cell in cell_images]

    # Convert to arrays and predict cell type using the neural network
    cell_arrays = [cell.reshape(-1) / 255.0 for cell in cell_equalized]
    ret, cell_predictions = net.predict(numpy.array(cell_arrays))

    # Check prediction quality
    cell_prediction_score = [(max(prediction) - min(prediction)) for prediction in cell_predictions]
    cell_prediction_accuracy = [score > 1.9 for score in cell_prediction_score]

    # Get inaccurate cells
    inaccurate_cells = [cell for cell,accurate in zip(cell_equalized, cell_prediction_accuracy) if not accurate]

    # Save inaccurate cells to disk
    for cell in inaccurate_cells:
        time = datetime.datetime.now().isoformat().replace(":", "-").replace(".", "-")
        cv2.imwrite("{0}/{1}.jpg".format(INACCURATE_PATH, time), cell)

    # Extract grid
    grid = [Cell(numpy.argmax(prediction), accuracy, centroid_x, centroid_y) for (prediction, accuracy, (centroid_x, centroid_y)) in zip(cell_predictions, cell_prediction_accuracy, extended_centroids)]

    # Paint centroids and cell type
    image_centroids = numpy.zeros(image.shape, numpy.uint8)

    for cell in grid:
        paint_cell(image_centroids, cell)

    ##################################
    # DRAW DEBUG IMAGE
    ##################################
    # Transform grayscale images to RGB
    image_dilate_rgb = cv2.cvtColor(image_dilate, cv2.COLOR_GRAY2RGB)

    # Calculate mask for the background image
    image_dilate_mask = cv2.inRange(image_dilate_rgb, (0,0,0), (0,0,0))
    image_centroids_mask = cv2.inRange(image_centroids, (0,0,0), (0,0,0))

    all_masks = cv2.bitwise_and(image_dilate_mask, image_centroids_mask)

    # Apply mask to background image
    debug_image = cv2.bitwise_and(image_eq, all_masks)

    # Transform masked image to RGB
    debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2RGB)

    # Add the original images used for the mask
    debug_image = cv2.add(debug_image, image_dilate_rgb)
    debug_image = cv2.add(debug_image, image_centroids)
    
    # Draw the cell contours
    cv2.drawContours(debug_image, poly, -1, (0,0,255), 2)
    
    return grid, debug_image

######################################################################
# MAIN FUNCTION
######################################################################
def MarioPuzzle():

    # ROS OpenCV bridge
    cvBridge = cv_bridge.CvBridge();

    # Create debug publishers
    pub_top_debug = rospy.Publisher("/robostylus_gui/top_debug", Image)
    pub_bottom_debug = rospy.Publisher("/robostylus_gui/bottom_debug", Image)

    # Create planner publisher
    pub_planner_points = rospy.Publisher("/robostylus_planner/points", Point2D)

    # Main loop
    while not rospy.is_shutdown():
        # Reset solution
        solution_cells = []

        # Get images from top and bottom screens
        image_top = rospy.wait_for_message("/robostylus_camera/top_screen", Image)
        image_bottom = rospy.wait_for_message("/robostylus_camera/bottom_screen", Image)

        # Transform image messages to OpenCV images
        cv_image_top = cvBridge.imgmsg_to_cv2(image_top, "rgb8")
        cv_image_bottom = cvBridge.imgmsg_to_cv2(image_bottom, "rgb8")

        # Find both grids in the images
        grid_top, debug_image_top = detect_puzzle_grid(cv_image_top)
        grid_bottom, debug_image_bottom = detect_puzzle_grid(cv_image_bottom)

        # If the function returned grids of the same size, perform additional checks
        if grid_top and grid_bottom and len(grid_top) == len(grid_bottom):

            # Collect inaccurate cells in both grids
            inaccurate_cells_top = [index for (cell, index) in zip(grid_top, xrange(len(grid_top))) if not cell.accurate]
            inaccurate_cells_bottom = [index for (cell, index) in zip(grid_bottom, xrange(len(grid_bottom))) if not cell.accurate]

            # Produce all combinations of positions that have to be toggled in every inaccurate board to consider all options in case of an error in neural network prediction
            all_combinations_top = powerset(inaccurate_cells_top, len(inaccurate_cells_top))
            all_combinations_bottom = powerset(inaccurate_cells_bottom, len(inaccurate_cells_bottom))

            # Generate the actual boards
            all_grids_top = [[cell.type ^ (index in combination) for (cell, index) in zip(grid_top, xrange(len(grid_top)))] for combination in all_combinations_top]
            all_grids_bottom = [[cell.type ^ (index in combination) for (cell, index) in zip(grid_bottom, xrange(len(grid_bottom)))] for combination in all_combinations_bottom]

            # Generate all combinations of grids that would make a puzzle
            all_grid_combinations = list(itertools.product(all_grids_top, all_grids_bottom))

            # Solve all combinations
            all_solutions = [solve_puzzle(top, bottom) for (top, bottom) in all_grid_combinations]

            # Collect indexes of valid ones
            all_valid_solutions_indexes = [i for i in xrange(len(all_solutions)) if all_solutions[i] is not None]

            # Count the number of valid solutions and act accordingly
            num_valid_solutions = len(all_valid_solutions_indexes)

            if num_valid_solutions == 0:
                print("No solution found")
            elif num_valid_solutions > 1:
                print("Multiple solutions found")
            else:
                # Collect index of valid solution
                valid_solution_index = all_valid_solutions_indexes[0]

                # Collect top and bottom boards used for that solution
                grid_top_bin, grid_bottom_bin = all_grid_combinations[valid_solution_index]

                # Get the real type of cells for those that were inaccurate in the detection stage
                corrected_cells_top = [Cell(cell_type, True, cell.x, cell.y) for (cell, cell_type) in zip(grid_top, grid_top_bin) if not cell.accurate]
                corrected_cells_bottom = [Cell(cell_type, True, cell.x, cell.y) for (cell, cell_type) in zip(grid_bottom, grid_bottom_bin) if not cell.accurate]

                # Paint the real type in the debug image
                for cell in corrected_cells_top:
                    paint_cell(debug_image_top, cell)

                for cell in corrected_cells_bottom:
                    paint_cell(debug_image_bottom, cell)

                # Paint solution in the debug image
                valid_solution = all_solutions[valid_solution_index]
                solution_cells = [cell for (cell, cell_bin) in zip(grid_bottom, valid_solution) if cell_bin == 1]
                
                for cell in solution_cells:
                    cv2.circle(debug_image_bottom, (cell.x, cell.y), 23, (255, 93, 0), 4)

                # Send data to the robot
                for cell in solution_cells:
                    pub_planner_points.publish(Point2D(cell.x, cell.y))

            # Transform debug images from OpenCV format to ROS messages
            image_top = cvBridge.cv2_to_imgmsg(debug_image_top, "rgb8")
            image_bottom = cvBridge.cv2_to_imgmsg(debug_image_bottom, "rgb8")

            # Publish the debug images
            pub_top_debug.publish(image_top)
            pub_bottom_debug.publish(image_bottom)

            # Sleep for a while if a solution was sent to the robot
            if len(solution_cells) != 0:
                rospy.sleep(4.0 + 2 * len(solution_cells))

if __name__ == '__main__':
    try:
        rospy.init_node('mario_puzzle')
        MarioPuzzle()
    except rospy.ROSInterruptException: 
        pass
