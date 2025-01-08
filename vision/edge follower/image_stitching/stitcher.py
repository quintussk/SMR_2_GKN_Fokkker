import cv2
import numpy as np
image1_path = "/image.png" 
image2_path = "/image2.png"

def stitch_images(image1_path, image2_path, direction='horizontal', output_path=None):
    """
    Stitch two images edge-to-edge.

    Args:
        image1_path (str): Path to the first image.
        image2_path (str): Path to the second image.
        direction (str): 'horizontal' for side-by-side, 'vertical' for top-to-bottom.
        output_path (str): Path to save the output image (optional).

    Returns:
        np.ndarray: The stitched image.
    """
    # Load the images
    img1 = cv2.imread(image1_path)
    img2 = cv2.imread(image2_path)

    # Resize images to match dimensions
    if direction == 'horizontal':
        # Match heights
        height = min(img1.shape[0], img2.shape[0])
        img1 = cv2.resize(img1, (int(img1.shape[1] * height / img1.shape[0]), height))
        img2 = cv2.resize(img2, (int(img2.shape[1] * height / img2.shape[0]), height))
        # Concatenate horizontally
        stitched = np.hstack((img1, img2))
    elif direction == 'vertical':
        # Match widths
        width = min(img1.shape[1], img2.shape[1])
        img1 = cv2.resize(img1, (width, int(img1.shape[0] * width / img1.shape[1])))
        img2 = cv2.resize(img2, (width, int(img2.shape[0] * width / img2.shape[1])))
        # Concatenate vertically
        stitched = np.vstack((img1, img2))
    else:
        raise ValueError("Direction must be 'horizontal' or 'vertical'")

    # Save output if specified
    if output_path:
        cv2.imwrite(output_path, stitched)

    return stitched

# Example Usage
if __name__ == "__main__":
    stitched_image = stitch_images("image1.png", "image2.png", direction='horizontal', output_path="stitched_output.jpg")
    cv2.imshow("Stitched Image", stitched_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
