import numpy as np
import cv2
import matplotlib.colors



tile_size = 50
world_size = 200
scale = 10

# Create gradient blurrer
cir = np.zeros((tile_size, tile_size, 3), dtype=float)
cv2.circle(cir,(cir.shape[1]//2,cir.shape[0]//2), 10, color=(1,1,1), thickness=-1)
cir = cv2.blur(cir, (10,10))#(20,20))


def perspect_transform(img, src, dst):
    """
    Convert an image from robot view, to birds-eye view.
    Uses OpenCV perspective transformations. Transforms the src box into the dst box
    
    Inputs:
        img:  nparray RGB image (doesn't have to be RGB)
        src:  pixel coordinates of initial box (Array of (x,y) pixel coords, cast as floats)
        dst:  pixel coordinates of destination box
    Returns:
        warped: nparray RGB image of bird-eye view
    """
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


def color_thresh(img, nav_hi=(256,50,256),nav_lo=(0,0,190), #nav_hi=(256,70,256),nav_lo=(0,0,180),
                      obs_hi=(256,256,90),obs_lo=(0,0,0),
                      rock_hi=(40,256,256),rock_lo=(25,140,120) ):
    """
    Classifies pixels as obstacle, rock, or navigable terrain using HSV color thresholding
    Thresholding in HSV space for better performance.
    Default values of high and low ranges experimentally determined using helper script.
    
    Inputs:
        img:       nparray of image (3 channel RGB)
    Kwargs:
        Tuples of high and low Hue, Saturaion, Value for each Obstacle, Rock, and Navigable calssification
    Retuns:
        color_select:  3 chan RGB image binary image, with Red = Obs, Green = Rock, Blue = Navigable
    """
    # Hack to account for odd matplotlib HSV settings (H and S are 0.0-1.0, but V is 0-255 (?))
    nav_hi = (nav_hi[0]/255.0, nav_hi[1]/255.0, nav_hi[2])
    nav_lo = (nav_lo[0]/255.0, nav_lo[1]/255.0, nav_lo[2])
    obs_hi = (obs_hi[0]/255.0, obs_hi[1]/255.0, obs_hi[2])
    obs_lo = (obs_lo[0]/255.0, obs_lo[1]/255.0, obs_lo[2])
    rock_hi = (rock_hi[0]/255.0, rock_hi[1]/255.0, rock_hi[2])
    rock_lo = (rock_lo[0]/255.0, rock_lo[1]/255.0, rock_lo[2])

    # Convert RGB image to HSV (using matplotlib function)
    hsv_img = matplotlib.colors.rgb_to_hsv(img)

    # Apply threshholding (Low and High)
    nav = (hsv_img[:,:,0] >= nav_lo[0]) & (hsv_img[:,:,0] <= nav_hi[0]) & \
          (hsv_img[:,:,1] >= nav_lo[1]) & (hsv_img[:,:,1] <= nav_hi[1]) & \
          (hsv_img[:,:,2] >= nav_lo[2]) & (hsv_img[:,:,2] <= nav_hi[2])
    obs = (hsv_img[:,:,0] >= obs_lo[0]) & (hsv_img[:,:,0] <= obs_hi[0]) & \
          (hsv_img[:,:,1] >= obs_lo[1]) & (hsv_img[:,:,1] <= obs_hi[1]) & \
          (hsv_img[:,:,2] >= obs_lo[2]) & (hsv_img[:,:,2] <= obs_hi[2])
        # Don't include pure black pixels (just non-data from transform)
    obs_nonzero = (img[:,:,0] != 0) \
                | (img[:,:,1] != 0) \
                | (img[:,:,2] != 0) 
    obs = obs & obs_nonzero
    rock= (hsv_img[:,:,0] >= rock_lo[0]) & (hsv_img[:,:,0] <= rock_hi[0]) & \
          (hsv_img[:,:,1] >= rock_lo[1]) & (hsv_img[:,:,1] <= rock_hi[1]) & \
          (hsv_img[:,:,2] >= rock_lo[2]) & (hsv_img[:,:,2] <= rock_hi[2])

    # 3 Channel bool nparray of classification (obstacle, rock, navigable)
    color_select = np.dstack([obs, rock, nav])

    return color_select


def show_thresh(color_select):
    """
    Convert Boolean array to uint8 to show as image
    Inputs:
        color_select: 3 Channel Boolean array with Red = Obs, Green = Rock, Blue = Nav
    Returns:
        thresh:       3 Channel uint8 array  "         "          "      "
    """
    thresh = np.zeros_like(color_select,dtype=np.uint8)
    thresh[color_select] = 255
    return thresh


def rover_coords(color_sel):
    """
    Convert image view to robot coordinate system.
    
    Inputs:
        color_sel:  nparray RGB Bool array classiffying Obs, Rock, Nav in respective chans
    Returns:
        xnav_pix, ynav_pix:     Pixel indices of navigable terrain (in robot coord frame)
        xobs_pix, yobs_pix:        " "           obstacle    "       "
        xrock_pix, yrock_pix:      " "           rock        "       "
    """
    # Extract True indices
    ynav, xnav = np.where(color_sel[:,:,2])
    yobs, xobs = np.where(color_sel[:,:,0])
    yrock, xrock = np.where(color_sel[:,:,1])
    
    # Rotate to robot coordinate frame ("Forwards" is +x-axis, "Left" is +y-axis)
    xnav_pix = -(ynav - color_sel.shape[0]).astype(np.float)
    ynav_pix = -(xnav - color_sel.shape[1]/2).astype(np.float)
    xobs_pix = -(yobs - color_sel.shape[0]).astype(np.float)
    yobs_pix = -(xobs - color_sel.shape[1]/2).astype(np.float)
    xrock_pix = -(yrock - color_sel.shape[0]).astype(np.float)
    yrock_pix = -(xrock - color_sel.shape[1]/2).astype(np.float)
    
    return xnav_pix, ynav_pix, xobs_pix, yobs_pix, xrock_pix, yrock_pix

def get_front_dist(obsx, obsy, scale):
    """
    Return distance to closest obstacle in front of Rover
    Considers 1.2 meter wide channel in front of rover.
    Takes the minimum obstacle distance.

    """
    front_inds = (obsy < 5) & (obsy > -5)
    if(np.any(front_inds)):
        front_dist = np.min(obsx[front_inds]) / scale
        return front_dist
    else:
        return 16 # Max 16 meters view

def get_right_dist(ang_nav, dist_nav, scale):
    """
    Return right distance along right diagonal (~45 degrees)
    """ 
    right_inds = ang_nav < np.deg2rad(-40)

    if(np.any(right_inds)):
        mean_right_dist = np.mean(dist_nav[right_inds]) / scale # Account for pixel scale
        return mean_right_dist 
    else:
        return 0

def get_left_dist(ang_nav, dist_nav, scale):
    """
    Return left distance along left diagonal (~45 degrees)
    """ 
    left_inds = ang_nav > np.deg2rad(40)

    if(np.any(left_inds)):
        mean_left_dist = np.mean(dist_nav[left_inds]) / scale # Account for pixel scale
        return mean_left_dist 
    else:
        return 0


def to_polar_coords(x_pixel, y_pixel):
    """
    Convert Cartesian to Polar coordinates (r,theta)
    (Useful for getting mean angle of naviagable terrain)
    Inputs:
        x_pixel:  nparray of x pixels
        y_pixel:  nparray of y pixels
    Returns:
        r, th:    Radius and angle (rad)
    """
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    r = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    th = np.arctan2(y_pixel, x_pixel) # in radians
    return r, th


def pix_to_world(navx, navy, obsx, obsy, rockx, rocky, xpos, ypos, yaw, scale, tile_size):
    """
    Converts Rover-centric to global position (rotate and scale to fit worldmap, no translation yet)
    Inputs:
        xnav_pix, ynav_pix:     Pixel indices of navigable terrain (in robot coord frame)
        xobs_pix, yobs_pix:        " "           obstacle    "       "
        xrock_pix, yrock_pix:      " "           rock        "       "
        xpos, ypos:             Absolute location of rover (bounded within world size, so 0-200)
        yaw:                    Robot heading (radians)
        scale:                  Scaling factor from pixels to meters (10 pixels = 1 meter)
        tile_size:              Size of return tile in pixels (equivalently meters)
    Returns:
        tile:                   RGB nparray of classifications. Robot centered in tile, which is 
                                typically 50m x 50m. Need to add tile to worldmap_sum by translating,
                                and clipping
    """
    # Conver to radians
    th = yaw * np.pi / 180
    # Rotate
    navx_rot = navx*np.cos(th) - navy*np.sin(th)
    navy_rot = navx*np.sin(th) + navy*np.cos(th)
    obsx_rot = obsx*np.cos(th) - obsy*np.sin(th)
    obsy_rot = obsx*np.sin(th) + obsy*np.cos(th)
    rockx_rot = rockx*np.cos(th) - rocky*np.sin(th)
    rocky_rot = rockx*np.sin(th) + rocky*np.cos(th)

    # Scale
    navx_tran = (navx_rot / scale)
    navy_tran = (navy_rot / scale)
    obsx_tran = (obsx_rot / scale)
    obsy_tran = (obsy_rot / scale)
    rockx_tran = (rockx_rot / scale)
    rocky_tran = (rocky_rot / scale)

    # Shift center to middle of small tile
    nx = np.int_(navx_tran + tile_size//2)
    ny = np.int_(navy_tran + tile_size//2)
    ox = np.int_(obsx_tran + tile_size//2)
    oy = np.int_(obsy_tran + tile_size//2)
    rx = np.int_(rockx_tran + tile_size//2)
    ry = np.int_(rocky_tran + tile_size//2)
    
    # Create tile (custom, hard-coded weights for classifications)
    tile = np.zeros((tile_size, tile_size, 3),dtype=np.uint8)
    tile[ny,nx,2] = 10
    tile[oy,ox,0] = 10
    tile[ry,rx,1] = 255 # Don't see rocks often, so weight it higher to not miss it

    return tile


def apply_tile_blur(tile, circle_blur):
    """
    Blur the classification tile by radial distance from robot.
    (Less confident in pixels "further" away)
    Inputs:
        tile:  Classification tile (nparray 3 chan RGB)
        circle_blur:   "kernel" tile of same shape, with circular gradient (1 in center, fades to 0)
    Returns:
        blur:  Blurred Classification tile
    """
    blur = np.multiply(tile,circle_blur).astype(np.uint8)
    return blur


def update_worldmap(worldmap, worldmap_sum, tile, xpos, ypos, roll, pitch):
    """
    Given a tile of the newest observation, update the worldmap by accumulating the classifications, then
    taking the dominant class (Obstacle, Rock, or Navigable) for the update pixels.
    Inputs:
        worldmap:      3 chan RGB nparray image, with binary (technically trinary(?)) classification of terrain
        worldmap_sum:  3 chan RGB nparray image with accumulated sum of previous classifications
        tile:          most recent observation/classification of terrian
        xpos, ypos:    Global location of robot (bounded by worldsize, ex 0-200m)
    Returns:
        worldmap:      Newly adjust classifications of the given region of interest
    """

    # Constants
    tile_size = tile.shape[0]
    offset = tile_size//2
    world_size = worldmap_sum.shape[0]
    
    # Define slicing index for worldmap
    x1 = np.int_(np.floor(xpos-offset))
    x2 = np.int_(np.floor(xpos+offset))+tile_size%2 # Hack for odd sizes
    y1 = np.int_(np.floor(ypos-offset))
    y2 = np.int_(np.floor(ypos+offset))+tile_size%2 
    
    # Define slicing index for tile (in case part of tile lies off the edge of worldmap)
    x1t, y1t = 0,0
    x2t, y2t = tile_size, tile_size
   
    # Adjust for edges
    if(x1 < 0):
        x1t = -x1
        x1 = 0
    if(y1 < 0):
        y1t = -y1
        y1 = 0
    if(x2 > world_size):
        x2t = -(x2-world_size)
        x2 = world_size
    if(y2 > world_size):
        y2t = -(y2-world_size)
        y2 = world_size
    


    # Accumulate classification
    #    Sum th
    tile_slice = tile[y1t:y2t , x1t:x2t]
    worldmap_sum[y1:y2 , x1:x2] += tile_slice
    
    # Only need to consider non-zero indexes
    indy, indx = np.where( (tile_slice[:,:,0]!=0) \
                         | (tile_slice[:,:,1]!=0) \
                         | (tile_slice[:,:,2]!=0) )    
    # Adjust offset
    indy +=  int(y1)
    indx +=  int(x1)

    min_conf_thresh = 100
                     
    # Loop through each pixel, and chose most dominant color value
    # Ex: If the worldmap_sum pixel (25,37) has RGB = (100,534,23),
    #     then color it Green, because most dominant color
    for i in range(len(indx)):
        yi, xi = indy[i], indx[i]
        chan = np.argmax(worldmap_sum[yi,xi])
        # if(worldmap_sum[yi,xi,chan] > min_conf_thresh):
        worldmap[yi,xi] = (0,0,0)
        worldmap[yi,xi,chan] = 255
    
    return worldmap


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    # 


    if(Rover.picking_up):
        return Rover

    # Constants for perspective transform
    dst_size = 5 # The destination box will be 2*dst_size on each side
    bottom_offset = 6 # Account 
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    xpos, ypos = Rover.pos
    yaw = Rover.yaw

    warped = perspect_transform(Rover.img, source, destination)
    col_sel = color_thresh(warped)
    Rover.vision_image = show_thresh(col_sel)

    # Calculate pixel values in rover-centric coords and distance/angle to all pixels
    navx, navy, obsx, obsy, rockx, rocky = rover_coords(col_sel)
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navx, navy)

    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rockx, rocky)

    # Only update dists if driving flat
    # if( (abs(Rover.roll-180) > 178.5) ):
    if (abs(Rover.pitch-180) > 179 ):
        Rover.front_dist = get_front_dist(obsx, obsy, scale)
        Rover.right_dist = get_right_dist(Rover.nav_angles, Rover.nav_dists, scale)
        Rover.left_dist  = get_left_dist (Rover.nav_angles, Rover.nav_dists, scale)

    # Only update map if actually moving (don't keep updating when stationary) and flat (not tilted)
    if( (abs(Rover.vel) > 0.09) & (abs(Rover.roll-180) > 179) & (abs(Rover.pitch-180) > 179) ):
        # Extract world coordinates of navigable, obstacle, and rock
        tile = pix_to_world(navx, navy, obsx, obsy, rockx, rocky, xpos, ypos, yaw, scale, tile_size)
        # Blur tile
        tile_blur = apply_tile_blur(tile, cir)
        # tile_blur = tile
        # Update worldmap with tile
        Rover.worldmap = update_worldmap(Rover.worldmap, Rover.worldmap_sum, tile_blur, xpos, ypos, Rover.roll, Rover.pitch)

    return Rover
