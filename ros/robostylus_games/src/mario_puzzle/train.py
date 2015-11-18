#!/usr/bin/python

import cv2
import numpy as np
import os

OUTPUT_NET_FILENAME = "nn"

TRAINING_SET_PATH = "training"
directories = ["/leaf", "/mushroom"]
directories = [TRAINING_SET_PATH + s for s in directories]

# Check if the directories exist
for directory in directories:
    if not os.path.exists(directory):
        print("No training set")
        quit()

# Build training set
inputs = []
outputs = []
last_shape = None
for directory_index in xrange(len(directories)):
    directory = directories[directory_index]

    # Get file list
    files = [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]

    # Add each file to the training set
    for file_path in files:
        image = cv2.imread(file_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)

        if last_shape and last_shape != image.shape:
            print("The images dont have the same shape")
            quit()

        last_shape = image.shape

        # Reshape to a one dimension array and normalize
        input_array = image.reshape(-1) / 255.0

        # Generate output values
        output_array = [-1.0] * len(directories)
        output_array[directory_index] = 1.0

        # Add to the set
        inputs.append(input_array)
        outputs.append(output_array)

# Convert to numpy arrays
inputs = np.array(inputs)
outputs = np.array(outputs)

# Define network topology
layers = np.array([len(inputs[0]), 10, len(outputs[0])])

# Create a neural network with sigmoid activation function
net = cv2.ANN_MLP(layers, activateFunc=cv2.ANN_MLP_SIGMOID_SYM, fparam1=1, fparam2=1)

# Define training parameters
step_size = 0.01
momentum = 0.1

nsteps = 100
max_err = 0.00001
condition = cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS

training_params = dict(term_crit = (condition, nsteps, max_err),
                train_method = cv2.ANN_MLP_TRAIN_PARAMS_BACKPROP, 
                bp_dw_scale = step_size, 
                bp_moment_scale = momentum)

# Train the network
num_iter = net.train(inputs, outputs, None, params=training_params)

# Check network performance
ret, predictions = net.predict(inputs)

# Print stats
print("Took {} iterations".format(num_iter))

stats = [max(p) - min(p) for p in predictions]
print("Max diff: {}".format(max(stats)))
print("Min diff: {}".format(min(stats)))
print("Avg diff: {}".format(sum(stats) / len(stats)))

rs = predictions.reshape(-1)
rs = np.abs(rs)
print("Max out: {}".format(max(rs)))
print("Min out: {}".format(min(rs)))

# Save trained network
net.save(OUTPUT_NET_FILENAME)
