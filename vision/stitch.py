import cv2
import os

def load_images_from_folder(folder, num_images):
    """
    Loads a specified number of images from a folder.
    
    :param folder: The folder path containing the images.
    :param num_images: Number of images to load.
    :return: List of loaded images.
    """
    images = []
    # List all image files in the folder
    image_files = [f for f in os.listdir(folder) if f.endswith(('.jpg', '.png', '.jpeg'))]
    
    if len(image_files) < num_images:
        print("Not enough images in the folder. Exiting.")
        return []

    # Load the specified number of images
    for i in range(num_images):
        image_path = os.path.join(folder, image_files[i])
        print(f"Loading image {i + 1}: {image_path}")
        img = cv2.imread(image_path)
        if img is None:
            print(f"Error: Unable to load image {image_path}.")
            continue
        images.append(img)
        cv2.imshow(f"Loaded Image {i + 1}", img)
        cv2.waitKey(1000)  # Show the frame for 1 second before clearing
        cv2.destroyWindow(f"Loaded Image {i + 1}")

    return images


def stitch_images(images):
    """
    Stitches a list of images into a panorama.
    
    :param images: List of images to stitch.
    :return: Stitched panorama or None if stitching failed.
    """
    if len(images) < 2:
        print("Need at least 2 images to perform stitching.")
        return None

    # Create a stitcher object and enable GPU if available
    stitcher = cv2.Stitcher_create()
    stitcher.setConfig(cv2.Stitcher_PANORAMA)
    
    # Enable GPU usage for stitching (if available)
    try:
        stitcher.setGpuUsage(cv2.Stitcher_GPU)
    except AttributeError:
        print("GPU usage is not supported in this OpenCV version.")
    
    status, panorama = stitcher.stitch(images)

    if status == cv2.Stitcher_OK:
        print("Image stitching completed successfully.")
        return panorama
    else:
        print("Error during stitching. Status code:", status)
        return None


def main():
    folder = r"C:\Users\ihsan\Captured_Images"  # Path to the folder containing captured images
    num_images = 3  # Number of images to load from the folder

    # Step 1: Load images from the folder
    images = load_images_from_folder(folder, num_images)
    if len(images) < num_images:
        print("Not enough images loaded. Exiting.")
        return

    # Step 2: Stitch the images
    panorama = stitch_images(images)
    if panorama is not None:
        # Step 3: Display the stitched panorama
        cv2.imshow("Stitched Panorama", panorama)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Failed to create a panorama.")


if __name__ == "__main__":
    main()
