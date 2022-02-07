import cv2  # Import the OpenCV library to enable computer vision
import numpy as np  # Import the NumPy scientific computing library

# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: A collection of methods to detect help with edge detection


def binary_array(array, thresh, value=0):
    """
    Return a 2D binary array (mask) in which all pixels are either 0 or 1

    :param array: NumPy 2D array that we want to convert to binary values
    :param thresh: Values used for thresholding (inclusive)
    :param value: Output value when between the supplied threshold
    :return: Binary 2D array...
             number of rows x number of columns = 
             number of pixels from top to bottom x number of pixels from
               left to right 
    """
    if value == 0:
        # Create an array of ones with the same shape and type as
        # the input 2D array.
        binary = np.ones_like(array)

    else:
        # Creates an array of zeros with the same shape and type as
        # the input 2D array.
        binary = np.zeros_like(array)
        value = 1

    # If value == 0, make all values in binary equal to 0 if the
    # corresponding value in the input array is between the threshold
    # (inclusive). Otherwise, the value remains as 1. Therefore, the pixels
    # with the high Sobel derivative values (i.e. sharp pixel intensity
    # discontinuities) will have 0 in the corresponding cell of binary.
    binary[(array >= thresh[0]) & (array <= thresh[1])] = value

    return binary


def blur_gaussian(channel, ksize=3):
    """
    Implementation for Gaussian blur to reduce noise and detail in the image

    :param image: 2D or 3D array to be blurred
    :param ksize: Size of the small matrix (i.e. kernel) used to blur
                  i.e. number of rows and number of columns
    :return: Blurred 2D image
    """
    return cv2.GaussianBlur(channel, (ksize, ksize), 0)


def mag_thresh(image, sobel_kernel=3, thresh=(0, 255)):
    """
    Implementation of Sobel edge detection

    :param image: 2D or 3D array to be blurred
    :param sobel_kernel: Size of the small matrix (i.e. kernel) 
                         i.e. number of rows and columns
    :return: Binary (black and white) 2D mask image
    """
    # Get the magnitude of the edges that are vertically aligned on the image
    sobelx = np.absolute(sobel(image, orient='x', sobel_kernel=sobel_kernel))

    # Get the magnitude of the edges that are horizontally aligned on the image
    sobely = np.absolute(sobel(image, orient='y', sobel_kernel=sobel_kernel))

    # Find areas of the image that have the strongest pixel intensity changes
    # in both the x and y directions. These have the strongest gradients and
    # represent the strongest edges in the image (i.e. potential lane lines)
    # mag is a 2D array .. number of rows x number of columns = number of pixels
    # from top to bottom x number of pixels from left to right
    mag = np.sqrt(sobelx ** 2 + sobely ** 2)

    # Return a 2D array that contains 0s and 1s
    return binary_array(mag, thresh)


def sobel(img_channel, orient='x', sobel_kernel=3):
    """
    Find edges that are aligned vertically and horizontally on the image

    :param img_channel: Channel from an image
    :param orient: Across which axis of the image are we detecting edges?
    :sobel_kernel: No. of rows and columns of the kernel (i.e. 3x3 small matrix)
    :return: Image with Sobel edge detection applied
    """
    # cv2.Sobel(input image, data type, prder of the derivative x, order of the
    # derivative y, small matrix used to calculate the derivative)
    if orient == 'x':
        # Will detect differences in pixel intensities going from
        # left to right on the image (i.e. edges that are vertically aligned)
        sobel = cv2.Sobel(img_channel, cv2.CV_64F, 1, 0, sobel_kernel)
    if orient == 'y':
        # Will detect differences in pixel intensities going from
        # top to bottom on the image (i.e. edges that are horizontally aligned)
        sobel = cv2.Sobel(img_channel, cv2.CV_64F, 0, 1, sobel_kernel)

    return sobel


def threshold(channel, thresh=(128, 255), thresh_type=cv2.THRESH_BINARY):
    """
    Apply a threshold to the input channel

    :param channel: 2D array of the channel data of an image/video frame
    :param thresh: 2D tuple of min and max threshold values
    :param thresh_type: The technique of the threshold to apply
    :return: Two outputs are returned:
               ret: Threshold that was used
               thresholded_image: 2D thresholded data.
    """
    # If pixel intensity is greater than thresh[0], make that value
    # white (255), else set it to black (0)
    return cv2.threshold(channel, thresh[0], thresh[1], thresh_type)
