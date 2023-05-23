# from geometry_msgs.msg import Twist
from SensingAgent import *
from drawing_functions import *

POSE_ADJUST_FLAG = False
POSE_TRANSLATE_ADJUST_FLAG = False
POSE_ROTATE_ADJUST_FLAG = False


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
    if POSE_ROTATE_ADJUST_FLAG:
        ## do rotation publish
        # gen_twist
        pass


def publish_translation(distance):
    if POSE_TRANSLATE_ADJUST_FLAG:
        ##do translation publish
        # gen_twist
        pass


def get_odometry_update():
    print("odometry_update_here")


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


def init_sensing_agent(
    sensing_agent=SensingAgent(), origin=(0, 0), _id=0, orientation=(0, 0)
):
    ox, oy = origin
    scale = 2
    opts = [
        (ox - 10 * scale, oy - 10 * scale),
        (ox - 10 * scale, oy + 10 * scale),
        (ox + 30 * scale, oy),
    ]
    # print(opts)

    mpt = gfn.get_midpoint(opts[0], opts[1])
    mpt2 = gfn.get_midpoint(mpt, opts[2])
    ap = Polygon(opts)
    rb = RigidBody(
        parent_agent=sensing_agent,
        ref_origin=mpt,
        ref_center=mpt2,
        endpoint=opts[2],
        rigid_link=ap,
    )
    sensor = Sensor(parent_agent=sensing_agent)
    sensor.fov_width = np.pi / 4

    sensing_agent.exoskeleton = rb
    sensing_agent.exoskeleton.states = []

    sensing_agent.centered_sensor = sensor
    sensing_agent.obj_tracker = ObjectTrackManager()
    sensing_agent.obj_tracker.linked_tracks = []
    sensing_agent.obj_tracker.layers = []
    sensing_agent.obj_tracker.trackmap = []
    sensing_agent.obj_tracker.global_track_store = {}

    sensing_agent.obj_tracker.parent_agent = sensing_agent
    sensing_agent._id = _id
    # rotation = sensing_agent.rotate_agent(orientation)
    return sensing_agent


def create_conformal_yolobox(dims, sa_state_id, max_x=1920, max_y=1080):
    rel_max_x = 100
    rel_fixed_y = 100
    cls, x, y, w, h = dims
    orig_x, orig_w = float(x) * max_x, float(w) * max_x
    orig_y, orig_h = float(y) * max_y, float(h) * max_y

    orig_center_x, orig_center_y = (orig_x + orig_w / 2), (orig_y + orig_h / 2)

    rel_center_x = (orig_center_x / max_x) * 100
    bbox = [rel_center_x, rel_fixed_y, 1, 1]
    conformal_yolobox = sann.register_annotation(0, bbox, sa_state_id)
    return conformal_yolobox


def agent_action(sensing_agent, layer, screen=None):
    sensing_agent.obj_tracker.add_new_layer(layer)
    sensing_agent.obj_tracker.process_layer(-1)

    agent_update(sensing_agent)
    curr_pt, pred_pt = sensing_agent.estimate_next_detection()
    if screen != None:
        pafn.clear_frame(screen)
        if len(pred_pt):
            pafn.frame_draw_dot(screen, curr_pt, pafn.colors["tangerine"])
            pafn.frame_draw_dot(screen, pred_pt, pafn.colors["yellow"])
            pafn.frame_draw_line(screen, (curr_pt, pred_pt), pafn.colors["white"])

        draw_sensing_agent(screen, sensing_agent)
        pygame.display.update()
