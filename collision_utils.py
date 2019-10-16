from __future__ import print_function
from __future__ import division
import pybullet as p
import numpy as np
from collections import defaultdict, deque, namedtuple
from itertools import product, combinations

INF = np.inf
PI = np.pi
CIRCULAR_LIMITS = -PI, PI
MAX_DISTANCE = 0
CLIENT = 0


def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


def circular_difference(theta2, theta1):
    return wrap_angle(theta2 - theta1)


def get_pose(body):
    return p.getBasePositionAndOrientation(body, physicsClientId=CLIENT)


def get_bodies():
    return [p.getBodyUniqueId(i, physicsClientId=CLIENT)
            for i in range(p.getNumBodies(physicsClientId=CLIENT))]


BodyInfo = namedtuple('BodyInfo', ['body_id', 'base_name', 'body_name'])


def get_body_info(body):
    return BodyInfo(body, *p.getBodyInfo(body, physicsClientId=CLIENT))


def get_base_name(body):
    return get_body_info(body).base_name.decode(encoding='UTF-8')


def get_body_name(body):
    return get_body_info(body).body_name.decode(encoding='UTF-8')


def get_name(body):
    name = get_body_name(body)
    if name == '':
        name = 'body'
    return '{}{}'.format(name, int(body))


def has_body(name):
    try:
        body_from_name(name)
    except ValueError:
        return False
    return True


def body_from_name(name):
    for body in get_bodies():
        if get_body_name(body) == name:
            return body
    raise ValueError(name)


def remove_body(body):
    return p.removeBody(body, physicsClientId=CLIENT)


JOINT_TYPES = {
    p.JOINT_REVOLUTE: 'revolute',  # 0
    p.JOINT_PRISMATIC: 'prismatic',  # 1
    p.JOINT_SPHERICAL: 'spherical',  # 2
    p.JOINT_PLANAR: 'planar',  # 3
    p.JOINT_FIXED: 'fixed',  # 4
    p.JOINT_POINT2POINT: 'point2point',  # 5
    p.JOINT_GEAR: 'gear',  # 6
}


def get_num_joints(body):
    return p.getNumJoints(body, physicsClientId=CLIENT)


def get_joints(body):
    return list(range(get_num_joints(body)))


def get_joint(body, joint_or_name):
    if type(joint_or_name) is str:
        return joint_from_name(body, joint_or_name)
    return joint_or_name


JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
                                     'qIndex', 'uIndex', 'flags',
                                     'jointDamping', 'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                                     'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
                                     'parentFramePos', 'parentFrameOrn', 'parentIndex'])


def get_joint_info(body, joint):
    return JointInfo(*p.getJointInfo(body, joint, physicsClientId=CLIENT))


def get_joint_name(body, joint):
    return get_joint_info(body, joint).jointName.decode('UTF-8')


def joint_from_name(body, name):
    for joint in get_joints(body):
        if get_joint_name(body, joint) == name:
            return joint
    raise ValueError(body, name)


def has_joint(body, name):
    try:
        joint_from_name(body, name)
    except ValueError:
        return False
    return True


def joints_from_names(body, names):
    return tuple(joint_from_name(body, name) for name in names)


JointState = namedtuple('JointState', ['jointPosition', 'jointVelocity',
                                       'jointReactionForces', 'appliedJointMotorTorque'])


def get_joint_state(body, joint):
    return JointState(*p.getJointState(body, joint, physicsClientId=CLIENT))


def get_joint_position(body, joint):
    return get_joint_state(body, joint).jointPosition


def get_joint_torque(body, joint):
    return get_joint_state(body, joint).appliedJointMotorTorque


def get_joint_positions(body, joints=None):
    return tuple(get_joint_position(body, joint) for joint in joints)


def set_joint_position(body, joint, value):
    p.resetJointState(body, joint, value, physicsClientId=CLIENT)


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        set_joint_position(body, joint, value)


def get_configuration(body):
    return get_joint_positions(body, get_movable_joints(body))


def set_configuration(body, values):
    set_joint_positions(body, get_movable_joints(body), values)


def get_full_configuration(body):
    # Cannot alter fixed joints
    return get_joint_positions(body, get_joints(body))


def get_joint_type(body, joint):
    return get_joint_info(body, joint).jointType


def is_movable(body, joint):
    return get_joint_type(body, joint) != p.JOINT_FIXED


def get_movable_joints(body):  # 45 / 87 on pr2
    return [joint for joint in get_joints(body) if is_movable(body, joint)]


def joint_from_movable(body, index):
    return get_joints(body)[index]


def is_circular(body, joint):
    joint_info = get_joint_info(body, joint)
    if joint_info.jointType == p.JOINT_FIXED:
        return False
    if joint_info.jointUpperLimit < joint_info.jointLowerLimit:
        raise ValueError("circular joint, check it out!")


def get_joint_limits(body, joint):
    if is_circular(body, joint):
        return CIRCULAR_LIMITS
    joint_info = get_joint_info(body, joint)
    return joint_info.jointLowerLimit, joint_info.jointUpperLimit


def get_joints_limits(body, joints):
    lower_limit = []
    upper_limit = []
    for joint in joints:
        lower_limit.append(get_joint_info(body, joint).jointLowerLimit)
        upper_limit.append(get_joint_info(body, joint).jointUpperLimit)
    return lower_limit, upper_limit


def get_min_limit(body, joint):
    return get_joint_limits(body, joint)[0]


def get_max_limit(body, joint):
    return get_joint_limits(body, joint)[1]


def get_max_velocity(body, joint):
    return get_joint_info(body, joint).jointMaxVelocity


def get_max_force(body, joint):
    return get_joint_info(body, joint).jointMaxForce


def get_joint_q_index(body, joint):
    return get_joint_info(body, joint).qIndex


def get_joint_v_index(body, joint):
    return get_joint_info(body, joint).uIndex


def get_joint_axis(body, joint):
    return get_joint_info(body, joint).jointAxis


def get_joint_parent_frame(body, joint):
    joint_info = get_joint_info(body, joint)
    return joint_info.parentFramePos, joint_info.parentFrameOrn


def violates_limit(body, joint, value):
    if not is_circular(body, joint):
        lower, upper = get_joint_limits(body, joint)
        if (value < lower) or (upper < value):
            return True
    return False


def violates_limits(body, joints, values):
    return any(violates_limit(body, joint, value) for joint, value in zip(joints, values))


def wrap_joint(body, joint, value):
    if is_circular(body, joint):
        return wrap_angle(value)
    return value


BASE_LINK = -1
STATIC_MASS = 0

get_num_links = get_num_joints
get_links = get_joints


def get_link_name(body, link):
    if link == BASE_LINK:
        return get_base_name(body)
    return get_joint_info(body, link).linkName.decode('UTF-8')


def get_link_parent(body, link):
    if link == BASE_LINK:
        return None
    return get_joint_info(body, link).parentIndex


LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                     'localInertialFramePosition', 'localInertialFrameOrientation',
                                     'worldLinkFramePosition', 'worldLinkFrameOrientation'])

LinkInfo = namedtuple('LinkInfo', ['linkName', 'linkIndex', 'jointName', 'jointIndex'])


def get_link_state(body, link):
    if p.getNumJoints(body) == 0:
        raise ValueError('{} does not have any link!'.format(body))
    return LinkState(*p.getLinkState(body, link))


def get_link_info(body, link):
    jointInfo = get_joint_info(body, link)
    linkName = jointInfo.linkName
    linkIndex = jointInfo.jointIndex
    jointName = jointInfo.jointName
    jointIndex = jointInfo.jointIndex
    return LinkInfo(linkName, linkIndex, jointName, jointIndex)


def get_com_pose(body, link):  # COM = center of mass
    link_state = get_link_state(body, link)
    return link_state.linkWorldPosition, link_state.linkWorldOrientation


def get_link_inertial_pose(body, link):
    link_state = get_link_state(body, link)
    return link_state.localInertialFramePosition, link_state.localInertialFrameOrientation


def get_link_pose(body, link):
    if link == BASE_LINK:
        return get_pose(body)
    # if set to 1 (or True), the Cartesian world position/orientation will be recomputed using forward kinematics.
    link_state = get_link_state(body, link)
    return link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation


def get_all_link_parents(body):
    return {link: get_link_parent(body, link) for link in get_links(body)}


def get_all_link_children(body):
    children = {}
    for child, parent in get_all_link_parents(body).items():
        if parent not in children:
            children[parent] = []
        children[parent].append(child)
    return children


def get_link_children(body, link):
    children = get_all_link_children(body)
    return children.get(link, [])


def get_link_ancestors(body, link):
    parent = get_link_parent(body, link)
    if parent is None:
        return []
    return get_link_ancestors(body, parent) + [parent]


def get_joint_ancestors(body, link):
    return get_link_ancestors(body, link) + [link]


def get_movable_joint_ancestors(body, link):
    return list(filter(lambda j: is_movable(body, j), get_joint_ancestors(body, link)))


def get_link_descendants(body, link):
    descendants = []
    for child in get_link_children(body, link):
        descendants.append(child)
        descendants += get_link_descendants(body, child)
    return descendants


def are_links_adjacent(body, link1, link2):
    return (get_link_parent(body, link1) == link2) or \
           (get_link_parent(body, link2) == link1)


def get_adjacent_links(body):
    adjacent = set()
    for link in get_links(body):
        parent = get_link_parent(body, link)
        adjacent.add((link, parent))
        # adjacent.add((parent, link))
    return adjacent


def get_adjacent_fixed_links(body):
    return list(filter(lambda item: not is_movable(body, item[0]),
                       get_adjacent_links(body)))


def get_fixed_links(body):
    edges = defaultdict(list)
    for link, parent in get_adjacent_fixed_links(body):
        edges[link].append(parent)
        edges[parent].append(link)
    visited = set()
    fixed = set()
    for initial_link in get_links(body):
        if initial_link in visited:
            continue
        cluster = [initial_link]
        queue = deque([initial_link])
        visited.add(initial_link)
        while queue:
            for next_link in edges[queue.popleft()]:
                if next_link not in visited:
                    cluster.append(next_link)
                    queue.append(next_link)
                    visited.add(next_link)
        fixed.update(product(cluster, cluster))
    return fixed


def pairwise_collision(body1, body2, max_distance=MAX_DISTANCE):  # 10000
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                                  physicsClientId=CLIENT)) != 0  # getContactPoints


def pairwise_link_collision(body1, link1, body2, link2, max_distance=MAX_DISTANCE):  # 10000
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                                  linkIndexA=link1, linkIndexB=link2,
                                  physicsClientId=CLIENT)) != 0  # getContactPoints


def single_collision(body1, **kwargs):
    for body2 in get_bodies():
        if (body1 != body2) and pairwise_collision(body1, body2, **kwargs):
            return True
    return False


def all_collision(**kwargs):
    bodies = get_bodies()
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            if pairwise_collision(bodies[i], bodies[j], **kwargs):
                return True
    return False


def get_moving_links(body, moving_joints):
    moving_links = list(moving_joints)
    for link in moving_joints:
        moving_links += get_link_descendants(body, link)
    return list(set(moving_links))


def get_moving_pairs(body, moving_joints):
    moving_links = get_moving_links(body, moving_joints)
    for i in range(len(moving_links)):
        link1 = moving_links[i]
        ancestors1 = set(get_joint_ancestors(body, link1)) & set(moving_joints)
        for j in range(i + 1, len(moving_links)):
            link2 = moving_links[j]
            ancestors2 = set(get_joint_ancestors(body, link2)) & set(moving_joints)
            if ancestors1 != ancestors2:
                yield link1, link2


def get_self_link_pairs(body, joints, disabled_collisions=set()):
    moving_links = get_moving_links(body, joints)
    fixed_links = list(set(get_links(body)) - set(moving_links))
    check_link_pairs = list(product(moving_links, fixed_links))
    if True:
        check_link_pairs += list(get_moving_pairs(body, joints))
    else:
        check_link_pairs += list(combinations(moving_links, 2))
    check_link_pairs = list(filter(lambda pair: not are_links_adjacent(body, *pair), check_link_pairs))
    check_link_pairs = list(filter(lambda pair: (pair not in disabled_collisions) and
                                                (pair[::-1] not in disabled_collisions), check_link_pairs))
    return check_link_pairs


def get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions):
    check_link_pairs = get_self_link_pairs(body, joints, disabled_collisions) if self_collisions else []
    moving_bodies = [body] + [attachment.child for attachment in attachments]
    if obstacles is None:
        obstacles = list(set(get_bodies()) - set(moving_bodies))
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))

    def collision_fn(q):
        if violates_limits(body, joints, q):
            return True
        set_joint_positions(body, joints, q)
        for attachment in attachments:
            attachment.assign()
        for link1, link2 in check_link_pairs:
            if pairwise_link_collision(body, link1, body, link2):
                return True
        return any(pairwise_collision(*pair) for pair in check_body_pairs)

    return collision_fn
