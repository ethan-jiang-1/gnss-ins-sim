def parse_motion_def(motion_def_seg, att, vel):
    """
    Parse the command of a segment in motion_def.
    Args:
        motion_def_seg: a segment in motion_def
        att: current attitude
        vel: current velocity
    Returns:
        [0]: Target attitude
        [1]: Target velocity
    """
    if motion_def_seg[0] == 1:
        att_com = [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = [motion_def_seg[4], motion_def_seg[5], motion_def_seg[6]]
    elif motion_def_seg[0] == 2:   # abs att and abs vel
        att_com = [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = [motion_def_seg[4], motion_def_seg[5], motion_def_seg[6]]
    elif motion_def_seg[0] == 3:   # rel att and rel vel
        att_com = att + [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = vel + [motion_def_seg[4], motion_def_seg[5], motion_def_seg[6]]
    elif motion_def_seg[0] == 4:   # abs att and rel vel
        att_com = [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = vel + [motion_def_seg[4], motion_def_seg[5], motion_def_seg[6]]
    elif motion_def_seg[0] == 5:   # rel att and abs vel
        att_com = att + [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = [motion_def_seg[4], motion_def_seg[5], motion_def_seg[6]]
    return att_com, vel_com
