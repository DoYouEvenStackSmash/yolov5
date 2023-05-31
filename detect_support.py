# from geometry_msgs.msg import Twist
from SensingAgent import *
from drawing_functions import *

POSE_ADJUST_FLAG = False
POSE_TRANSLATE_ADJUST_FLAG = False
POSE_ROTATE_ADJUST_FLAG = True


def gen_twist(direction=None):
    """
    Translates a direction to a movement command
    """
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    if direction == "LEFT":
        twist.angular.z = 0.5
    elif direction == "RIGHT":
        twist.angular.z = -0.5
    return twist


def drive(origin, pt):
    """
    prints a direction to turn
    """
    rad, r = mfn.car2pol(origin, pt)
    if rad < np.pi / 2 and rad > -np.pi / 2:
        print("turn right")
        # pafn.frame_draw_bold_line(screen, (origin, pt), pafn.colors["green"])
    else:
        print("turn left")
        # pafn.frame_draw_bold_line(screen, (origin, pt), pafn.colors["red"])


def normalize(rel_x, rel_y, abs_x_max=1000):
    """
    Normalizes the coordinate system to x = [0,1000] and y = 500
    """
    rel_x = rel_x / 100 * abs_x_max
    rel_y = 500
    return (rel_x, rel_y)


def do_rel_detection(sensing_agent):
    """
    Perform a relative detection. DOES NOT ADJUST POSE
    used for debugging
    """
    curr_pt, pred_pt = sensing_agent.estimate_rel_next_detection()
    print((curr_pt, pred_pt))
    if not len(pred_pt):
        return
    if len(pred_pt):
        prev_pt = [pred_pt[0], pred_pt[1]]
        curr_pt = normalize(curr_pt[0], curr_pt[1])
        pred_pt = normalize(pred_pt[0], pred_pt[1])
        status, flags = sensing_agent.is_detectable(prev_pt)
        if flags == Sensor.ANGULAR:
            drive((500, 500), pred_pt)


def publish_rotation(radians):
    """
    placeholder for publish of angular geometry/twist
    """
    if POSE_ROTATE_ADJUST_FLAG:
        ## do rotation publish
        # gen_twist
        pass


def publish_translation(distance):
    """
    placeholder for publish of linear geometry/twist
    """
    if POSE_TRANSLATE_ADJUST_FLAG:
        ##do translation publish
        # gen_twist
        pass


def get_odometry_update():
    """
    placeholder for odometry query
    """
    pass

def get_range():
    """
    placeholder for range query
    """
    return None
    pass

def agent_update(sensing_agent):
    """
    Updates the pose of a single agent
    """
    est_rotation, est_translation = sensing_agent.estimate_pose_update()

    if est_rotation != None:
        print(f"estimated (degrees): {est_rotation * 180 / np.pi}")
        return
        ###
        publish_rotation(est_rotation)
        get_odometry_update()
        ###
        rotation = sensing_agent.apply_rotation_to_agent(est_rotation)
        sensing_agent.obj_tracker.add_angular_displacement(0, -est_rotation)
        sensing_agent.exoskeleton.rel_theta += rotation

    if est_translation != None:
        ###
        publish_translation(est_translation)
        get_odometry_update()
        ###
        translation = sensing_agent.apply_translation_to_agent(est_translation)
        sensing_agent.obj_tracker.add_linear_displacement(-translation, 0)


def create_detection_with_range(sensor_origin, time_of_detection, detection_cls, x, y, w, h, range_to_target, img_shape_x, img_shape_y, sensor_fov_width):
  """
  Creates a detection when a range component is present
  """
  # compute position
  center_x = img_shape_x / 2
  det_center_x = img_shape_x * x + (w * img_shape_x) / 2

  theta = mfn.euclidean_dist((center_x,0), (det_center_x,0)) / img_shape_x * sensor_fov_width
  r = range_to_target

  detection_coord = mfn.pol2car(sensor_origin, range_to_target, theta)
  posn = Position(detection_coord[0], detection_coord[1])

  # compute yolobox
  sensor_theta, sensor_range_to_target = mfn.car2pol(sensor_origin, detection_coord)
  ratio = sensor_theta / sensor_fov_width
  sensor_x = sensor_fov_width * ratio + 50
  sensor_y = sensor_range_to_target
  sensor_w = w * img_shape_x
  sensor_h = h * img_shape_y

  sensor_bbox = [sensor_x, sensor_y, sensor_w, sensor_h]
  yb = sann.register_annotation(detection_cls, sensor_bbox, time_of_detection)

  # create detection
  det = Detection(posn, yb)
  return det

def create_detection_without_range(sensing_agent, sensor_origin, time_of_detection, detection_cls, x, y, w, h, img_shape_x, img_shape_y, sensor_fov_width):
    """
    Creates a detection when range is not present
    """
    center_x = img_shape_x / 2
    det_center_x = (img_shape_x * x) + (w * img_shape_x) / 2

    fixed_range = 100
    range_to_target = fixed_range
    rel_y = (det_center_x / img_shape_x) * 100 #- 110
    # theta = mfn.euclidean_dist((center_x,0), (det_center_x,0)) / img_shape_x * sensor_fov_width

    # detection_coord = mfn.pol2car(sensor_origin, range_to_target, theta)
    detection_coord = (100, rel_y)
    print(rel_y)
    print(detection_coord)
    posn = Position(100, rel_y)
    sensor_coord = sensing_agent.transform_to_local_sensor_coord((0,0), detection_coord)

    sensor_x = sensor_coord[0]
    sensor_y = sensor_coord[1]
    sensor_w = w * img_shape_x
    sensor_h = h * img_shape_y

    sensor_bbox = [sensor_x, sensor_y, sensor_w, sensor_h]
    yb = sann.register_annotation(detection_cls, sensor_bbox, time_of_detection)

    # create detection
    det = Detection(posn, yb)
    return det

def agent_action(sensing_agent, layer, screen=None):
    sensing_agent.obj_tracker.add_new_layer(layer)
    sensing_agent.obj_tracker.process_layer(-1)

    r,t = sensing_agent.tracker_query()
    sensing_agent.reposition(r,t)
    # agent_update(sensing_agent)
    curr_pt, pred_pt = (), ()
    arr = sensing_agent.estimate_next_detection()
    if len(arr):
        curr_pt = arr[0][0]
        pred_pt = arr[0][1]
    if screen != None:
        pafn.clear_frame(screen)
        if len(curr_pt):
            pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"], 0, 8)
        if len(pred_pt):
            pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"], 0, 8)
            pafn.frame_draw_line(screen, (curr_pt, pred_pt), pafn.colors["white"])
        draw_sensing_agent(screen, sensing_agent)
        pygame.display.update()