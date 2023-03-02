import glob
import matplotlib.pyplot as plt
import os


def create_image(x, y):
    # Create a new figure
    plt.figure()

    # Plot the points and draw a line through them
    plt.plot(x, y, 'ro', markersize=1)

    # Remove the tick marks
    plt.xticks([])
    plt.yticks([]) 

    # Remove the spines
    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)

    # Set the path to the output folder
    output_folder_path = os.path.join('.', 'images')

    # Get the list of images in the output folder and sort them in ascending order
    image_files = glob.glob(os.path.join(output_folder_path, '*.png'))
    image_files.sort()

    # If there are no images in the output folder, start the counting from 1
    if not image_files:
        image_number = 1
    # If there are images in the output folder, start the counting from the number of the latest image
    else:
        latest_image_file = image_files[-1]
        latest_image_number = int(os.path.splitext(os.path.basename(latest_image_file))[0].split('_')[-1])
        image_number = latest_image_number + 1

    # Save the plot as an image with a name that includes the image number
    plt.savefig(os.path.join(output_folder_path, f'image_{image_number}.png'))
