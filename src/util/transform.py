import numpy as np

def create_transformation_matrix(rotation_matrix, tvec):
    """
    Function to create a 4x4 transformation matrix.
    
    :param rotation_matrix: 3x3 rotation matrix (numpy.ndarray)
    :param tvec: 1x3 translation vector (numpy.ndarray or list)
    :return: 4x4 transformation matrix (numpy.ndarray)
    """
    # Input validation
    if rotation_matrix.shape != (3, 3):
        raise ValueError("rotation_matrix must be a 3x3 matrix.")
    
    # Convert tvec to 3x1 array if it is a 1x3 array
    if len(tvec) == 3 and isinstance(tvec, list):
        tvec = np.array(tvec).reshape(3, 1)  # Convert 1x3 to 3x1 array
    elif tvec.shape == (1, 3):  # If it is already a 1x3 array
        tvec = tvec.T  # Transpose to convert to 3x1 array
    
    # Check if tvec is a 3x1 array
    if tvec.shape != (3, 1):
        raise ValueError("tvec must be a 3x1 2D array.")
    
    # Create 4x4 transformation matrix
    transformation_matrix = np.eye(4)  # Create 4x4 identity matrix
    transformation_matrix[:3, :3] = rotation_matrix  # Set rotation matrix
    transformation_matrix[:3, 3] = tvec.flatten()  # Set translation vector (flatten to 1D)
    
    return transformation_matrix

def extract_rotation_translation(transformation_matrix):
    """
    Function to extract rotation matrix and translation vector from a 4x4 transformation matrix.
    
    :param transformation_matrix: 4x4 transformation matrix (numpy.ndarray)
    :return: rotation matrix (3x3), translation vector (3x1)
    """
    # Extract rotation matrix (top-left 3x3 part)
    rotation_matrix = transformation_matrix[:3, :3]
    
    # Extract translation vector (top 3 values of the last column)
    tvec = transformation_matrix[:3, 3]
    tvec = np.array(tvec).reshape(1, 3)
    
    return rotation_matrix, tvec

def transformation_3d(rx, ry, rz, tx, ty, tz):
    """
    Function to create a 3D transformation matrix.
    :param rx: rotation around x-axis (radians)
    :param ry: rotation around y-axis (radians)
    :param rz: rotation around z-axis (radians)
    :param tx: translation along x-axis
    :param ty: translation along y-axis
    :param tz: translation along z-axis
    :return: 4x4 transformation matrix
    """
    # Rotation around x-axis
    rot_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(rx), -np.sin(rx), 0],
        [0, np.sin(rx),  np.cos(rx), 0],
        [0, 0, 0, 1]
    ])
    
    # Rotation around y-axis
    rot_y = np.array([
        [np.cos(ry), 0, np.sin(ry), 0],
        [0, 1, 0, 0],
        [-np.sin(ry), 0, np.cos(ry), 0],
        [0, 0, 0, 1]
    ])
    
    # Rotation around z-axis
    rot_z = np.array([
        [np.cos(rz), -np.sin(rz), 0, 0],
        [np.sin(rz),  np.cos(rz), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Translation
    translation = np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])
    
    return np.linalg.multi_dot([rot_z, rot_y, rot_x, translation])

