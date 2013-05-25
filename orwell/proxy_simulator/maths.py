def dot_product(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


def cross_product(v1, v2):
    return (
            v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]
            )


def normalise(vector):
    """
    >>> print normalise((12, 0, 0))
    (1.0, 0.0, 0.0)
    >>> print normalise((4, 4, 0))
    (0.7071067811865475, 0.7071067811865475, 0.0)
    >>> print normalise((0, 0.2, 0.2))
    (0.0, 0.7071067811865475, 0.7071067811865475)
    """
    if (all(((0 == coord) for coord in vector))):
        return None
    length_p2 = vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2
    length = length_p2 ** 0.5
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def project_on_plane(normal, distance, point):
    """
    Project a point on a plance defined by a normal and a distance to the
    origin following a line parallel to the normal.
    equation of the plane:
        normal[0] * x + normal[1] * y + normal[2] * z + distance = 0
    `normal`: normal of the plane.
    `distance`: distance to the origin.
    `point`: the point to project (projection parallel to the normal).
    >>> normal = (0, 1, 0)
    >>> distance = 1
    >>> point = (0, 0, 0)
    >>> print project_on_plane(normal, distance, point)
    (0, -1, 0)
    """
    up = (normal[0] * point[0] + normal[1] * point[1] + normal[2] * point[2] +
          distance)
    down = normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2
    t = - up / down
    projected = (
        point[0] + normal[0] * t,
        point[1] + normal[1] * t,
        point[2] + normal[2] * t
    )
    return projected


def add(v1, v2):
    return (v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2])


def vec_mul(vector, factor):
    return (vector[0] * factor,
            vector[1] * factor,
            vector[2] * factor)


def make_rotation_matrix(normal):
    """
    Compute a matrix to rotate vectors in a new base with #normal as the basis
    for the y axis.
    (i, j, k) is the original orthonormal base.
    (i_, j_, k_) is the new orthonormal base.
    >>> n = (0, 1, 0)
    >>> print make_rotation_matrix(n)
    ... # doctest: +NORMALIZE_WHITESPACE
    (1.0, 0.0, 0.0,
     0.0, 1.0, 0.0,
     0.0, 0.0, 1.0)
    >>> n = (0, 1, 0.2)
    >>> print make_rotation_matrix(n)
    ... # doctest: +NORMALIZE_WHITESPACE
    (1.0, 0.0, 0.0, 0.0,
     0.9805806756909201, -0.19611613513818404, -0.0,
     0.19611613513818402, 0.9805806756909202)
    """
    i = (1, 0, 0)
    j_ = normalise(normal)
    k_ = normalise(cross_product(i, j_))
    if (k_ is None):
        k = (0, 0, 1)
        i_ = normalise(cross_product(j_, k))
        k_ = cross_product(i_, j_)
    else:
        i_ = cross_product(j_, k_)
    rotation_matrix = (
        i_[0], j_[0], k_[0],
        i_[1], j_[1], k_[1],
        i_[2], j_[2], k_[2]
    )
    return rotation_matrix


def mat_mul(matrix, vector):
    """
    >>> i = (1, 0, 0, 0, 1, 0 ,0, 0, 1)
    >>> v = (12, 32, 96)
    >>> print mat_mul(i, v)
    (12, 32, 96)
    >>> rot = (0, 0, 1, 0, 1, 0, -1, 0, 0)
    >>> print mat_mul(rot, v)
    (96, 32, -12)
    """
    return (
        matrix[0 + 3 * 0] * vector[0] +
            matrix[1 + 3 * 0] * vector[1] +
            matrix[2 + 3 * 0] * vector[2],
        matrix[0 + 3 * 1] * vector[0] +
            matrix[1 + 3 * 1] * vector[1] +
            matrix[2 + 3 * 1] * vector[2],
        matrix[0 + 3 * 2] * vector[0] +
            matrix[1 + 3 * 2] * vector[1] +
            matrix[2 + 3 * 2] * vector[2]
        )


def transpose(matrix):
    """
    >>> matrix = (11, 12, 13, 21, 22, 23, 31, 32, 33)
    >>> print transpose(matrix)
    (11, 21, 31, 12, 22, 32, 13, 23, 33)
    """
    return (
        matrix[0], matrix[3], matrix[6],
        matrix[1], matrix[4], matrix[7],
        matrix[2], matrix[5], matrix[8]
        )
