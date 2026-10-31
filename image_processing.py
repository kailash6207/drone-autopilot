from PIL import Image, ImageFilter
from config import MAP_WIDTH, WINDOW_HEIGHT

def process_floorplan(image_path):
    """
    Loads, resizes, and inflates the floorplan wall properties to 
    keep the drone away from partitions and narrow edges.
    """
    img = Image.open(image_path).convert('L')
    
    # Scale strictly to the map area boundary, leaving space for the UI
    img = img.resize((MAP_WIDTH, WINDOW_HEIGHT), Image.Resampling.NEAREST)
    
    # Balanced wall inflation for centering
    img = img.filter(ImageFilter.MinFilter(3))
    img = img.filter(ImageFilter.MinFilter(3))
    
    try:
        import numpy as np
        img_array = np.array(img)
        binary_map = np.where(img_array < 50, 1, 0).tolist()
    except Exception:
        pix = img.load()
        w, h = img.size
        binary_map = [[1 if pix[x, y] < 50 else 0 for x in range(w)] for y in range(h)]
        
    return binary_map
