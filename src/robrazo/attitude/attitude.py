import numpy as np

class Quaternion:
    _zero_tolerance = 1e-10
    _small_angle_tolerance = 1e-6

    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"Quaternion({self.w}, {self.x}, {self.y}, {self.z})"

    @staticmethod
    def identity():
        return Quaternion(1.0, 0.0, 0.0, 0.0)

    @staticmethod
    def pure(x, y, z):
        return Quaternion(0.0, x, y, z)

    @staticmethod
    def from_vec(v):
        if len(v) == 3:
            return Quaternion.pure(v[0], v[1], v[2])
        elif len(v) == 4:
            return Quaternion(v[0], v[1], v[2], v[3])
        else:
            raise ValueError("Input vector must be of length 3 or 4")
    
    @staticmethod
    def from_axis_angle(axis, angle):
        if len(axis) != 3:
            raise ValueError("Axis must be a 3D vector")
        half_angle = angle / 2.0
        sin_half_angle = np.sin(half_angle)
        return Quaternion(
            np.cos(half_angle),
            axis[0] * sin_half_angle,
            axis[1] * sin_half_angle,
            axis[2] * sin_half_angle
        )

    @property
    def is_unit(self):
        return abs(self.norm() - 1.0) < self._zero_tolerance

    @property
    def is_pure(self):
        return self.w == 0.0

    @property
    def scalar(self):
        return self.w

    @property
    def vector(self):
        return np.array([self.x, self.y, self.z])

    def vecnorm_squared(self):
        return self.x**2 + self.y**2 + self.z**2

    def vecnorm(self):
        return np.sqrt(self.vecnorm_squared())

    def norm_squared(self):
        return self.w**2 + self.x**2 + self.y**2 + self.z**2

    def norm(self):
        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalize(self):
        n = self.norm()
        if n == 0:
            raise ValueError("Cannot normalize a zero-length quaternion")
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def inverse(self):
        n_squared = self.normSquared()
        if n_squared == 0:
            raise ValueError("Cannot invert a zero-length quaternion")
        return self.conjugate().scale(1.0 / n_squared)

    def scale(self, scalar):
        return Quaternion(self.w * scalar, self.x * scalar, self.y * scalar, self.z * scalar)

    def compose(self, other):
        if not isinstance(other, Quaternion):
            raise TypeError("Can only compose by another Quaternion")
        return Quaternion(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        )

    def angle(self):
        return 2 * np.arctan2(self.vecnorm_squared(), self.w)

    def axis(self):
        theta2 = self.vecnorm_squared()
        
        

def lmat(q):
    if not isinstance(q, Quaternion):
        raise TypeError("Input must be a Quaternion")
    return np.array([
        [q.w, -q.x, -q.y, -q.z],
        [q.x, q.w, -q.z, q.y],
        [q.y, q.z, q.w, -q.x],
        [q.z, -q.y, q.x, q.w]
    ])

def rmat(q):
    if not isinstance(q, Quaternion):
        raise TypeError("Input must be a Quaternion")
    return np.array([
        [q.w, -q.x, -q.y, -q.z],
        [q.x, q.w, q.z, -q.y],
        [q.y, -q.z, q.w, q.x],
        [q.z, q.y, -q.x, q.w]
    ])

def _pure_quat_expm(q_v):
    theta2 = q_v[0]**2 + q_v[1]**2 + q_v[2]**2
    theta = np.sqrt(theta2)
    if theta <= Quaternion._small_angle_tolerance:
        c = 1 - theta2 / 6
        return Quaternion(1 - theta2 / 2, q_v[0] * c, q_v[1] * c, q_v[2] * c)
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    scale = sin_theta / theta
    return Quaternion(
        cos_theta,
        q_v[0] * scale,
        q_v[1] * scale,
        q_v[2] * scale
    )

def quat_exp(q):
    if not isinstance(q, Quaternion):
        raise TypeError("Input must be a Quaternion")
    exp_q_v = _pure_quat_expm(q.vector) 
    exp_q_w = np.exp(q.scalar)
    return exp_q_v.scale(exp_q_w)

def expm(q):
    if len(q) == 3:
        return _pure_quat_expm(Quaternion.pure(q[0], q[1], q[2]))
    else:
        raise ValueError("Input must be a vector of length 3. Use `quat_expm` for Quaternion input.")

def _unit_quat_logm(q):
    if not isinstance(q, Quaternion):
        raise TypeError("Input must be a Quaternion")
    w = q.scalar
    v = q.vector
    theta2 = v[0]**2 + v[1]**2 + v[2]**2
    theta = np.sqrt(theta2)
    if theta < Quaternion._small_angle_tolerance:
        scale =  (1 - theta2 / (3 * w**2)) / w
    else:
        scale = np.arctan2(theta, w) / theta    
    return np.array([
        v[0] * scale,
        v[1] * scale,
        v[2] * scale
    ])

def quat_log(q):
    if not isinstance(q, Quaternion):
        raise TypeError("Input must be a Quaternion")
    log_q_unit = _unit_quat_logm(q)
    log_q_norm = np.log(q.norm())
    return Quaternion(log_q_norm, log_q_unit[0], log_q_unit[1], log_q_unit[2])

def logm(q):
    if not isinstance(q, Quaternion):
        raise TypeError("Input must be a Quaternion")
    if not q.is_unit:
        raise ValueError("Input quaternion must be a unit quaternion. For non-unit quaternions, use `quat_log`.")
    return _unit_quat_logm(q)
