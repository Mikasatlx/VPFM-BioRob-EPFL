"""
Polygon Obstacle for Avoidance Calculations
"""
# Author: Lukas Huber
# Date: created: 2020-02-28
# Email: lukas.huber@epfl.ch
# License: BSD (c) 2021

import sys
import warnings
import copy
import time

from math import pi

import numpy as np
from numpy import linalg as LA

import shapely
from shapely import geometry
from shapely.geometry.polygon import LinearRing
from shapely.ops import nearest_points

from vartools.directional_space import (
    get_angle_space,
    get_angle_space_of_array,
)
from vartools.directional_space import get_directional_weighted_sum
from vartools.angle_math import (
    angle_is_in_between,
    angle_difference_directional,
)
from vartools.angle_math import *

from dynamic_obstacle_avoidance.utils import get_tangents2ellipse

from ._base import Obstacle, GammaType


def is_one_point(point1, point2, margin=1e-9):
    """Check if it the two points coincide [1-norm]"""
    return np.allclose(point1, point2, rtol=1e-9)


class Polygon(Obstacle):
    """Class to define Star Shaped Polygons

    Many calculations focus on 2D-problem.
    Generalization and extension to higher dimensions is possible, but not complete (yet).

    This class defines obstacles to modulate the DS around it
    At current stage the function focuses on Ellipsoids,
    but can be extended to more general obstacles.

    Attributes
    ----------
    edge_points:

    """

    def __init__(
        self,
        edge_points: np.ndarray,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)

        self.edge_points = np.array(edge_points)

        self.dim = self.center_position.shape[0]
        if self.dim == 2:
            # n_planes not really needed anymore...
            self.n_planes_edge = self.edge_points.shape[1]
            self.n_planes = self.edge_points.shape[1]  # with reference
            self.ind_edge_ref = None
        else:
            raise NotImplementedError(
                "Not yet implemented for dimensions higher than 3"
            )

        # No go zone assuming a uniform margin around the obstacle
        self.edge_margin_points = self.edge_points

        self.edge_points_augmented = np.zeros((self.dim, self.n_planes+1))
        self.edge_points_augmented[:, :-1] = self.edge_points
        self.edge_points_augmented[:,-1] = self.edge_points[:, 0]

    def draw_obstacle(
        self,
        include_margin=False,
        n_curve_points=5,
        numPoints=None,
        point_density=(2 * pi / 50),
    ):
        # Compute only locally
        num_edges = self.edge_points.shape[1]

        self._boundary_points = self.edge_points

        dir_boundary_points = self._boundary_points / np.linalg.norm(
            self._boundary_points, axis=0
        )

        if self.is_boundary:
            self._boundary_points_margin = (
                self._boundary_points - dir_boundary_points * self.margin_absolut
            )

        else:
            self._boundary_points_margin = (
                self._boundary_points + dir_boundary_points * self.margin_absolut
            )
    
    def get_normal_direction_with_distance(
        self,
        position,
        in_global_frame=True,
    ):
        if self.dim > 2:
            raise NotImplementedError("Under construction for d>2.")

        if in_global_frame:
            position = self.transform_global2relative(position)
        
        point = geometry.Point(position[0], position[1])
        edge_list = []

        for idx in range(self.edge_points.shape[1]):
            edge_list.append((self.edge_points[0, idx], self.edge_points[1, idx]))
        
        poly = geometry.Polygon(edge_list)
        dist = point.distance(poly)
        p, _ = nearest_points(poly, point)
        closest_point_coords = list(p.coords)[0]

        normal_vector = position - closest_point_coords
        if LA.norm(normal_vector) != 0:
            normal_vector = normal_vector / LA.norm(normal_vector)

        if in_global_frame:
            normal_vector = self.transform_relative2global_dir(normal_vector)
        
        return normal_vector, dist

    def get_tangents_and_normals_of_edge(self, edge_points: np.ndarray):
        """Returns normal and tangent vector of tiles.
                -> could be an 'abstractmethod'

        Paramters
        ---------
        local_position: position in local_frame
        edge_points: (2D)-array of edge points, with the first equal to the last one

        Returns
        -------
        Normals: Tangent vectors (to the surfaces)
        Tangents: Normal vectors (to the surfaces)

        """
        if self.dim > 2:
            raise NotImplementedError("Higher dimensions lack functionality.")

        # Get tangents and normalize
        tangents = edge_points[:, 1:] - edge_points[:, :-1]
        tangents = tangents / np.tile(LA.norm(tangents, axis=0), (tangents.shape[0], 1))

        if np.cross(edge_points[:, 0], edge_points[:, 1]) > 0:
            normals = np.vstack((tangents[1, :], (-1) * tangents[0, :]))
        else:
            normals = np.vstack(((-1) * tangents[1, :], tangents[0, :]))

        return tangents, normals

    def get_normal_distance_to_surface_pannels(
        self, local_position: np.ndarray, edge_points: np.ndarray, normals: np.ndarray
    ):
        """
        Get the distance to all surfaces panels

        Paramters
        ---------
        local_position: position in local_frame
        edge_points: (2D)-array of edge points, with the first equal to the last one
        normals: Normal vectors (with one dimension less than edge_points)

        Returns
        -------
        (a tuple containing)
        Distances: distance to each of the normal directions
        """
        if self.dim > 2:
            raise NotImplementedError("Higher dimensions lack functionality.")

        distances = np.zeros(normals.shape[1])
        for ii in range(normals.shape[1]):
            # MAYBE: transform to array function
            dist_edge = normals[:, ii].dot(edge_points[:, ii])
            dist_pos = normals[:, ii].dot(local_position)
            distances[ii] = max(dist_pos - dist_edge, 0)
        return distances

    def get_gamma(
        self,
        position,
        in_global_frame=True,
        gamma_type=GammaType.EUCLEDIAN,
        with_reference_point_expansion=False,
        margin_absolut = None,
    ):
        _, distance_to_edge = self.get_normal_direction_with_distance(position)

        if margin_absolut is None:
            margin_absolut = self.margin_absolut
        
        if in_global_frame:
            position = self.transform_global2relative(position)

        projected_dist = distance_to_edge - margin_absolut

        # Choose proporitional
        if gamma_type == GammaType.EUCLEDIAN:
            if projected_dist > 0:
                gamma = projected_dist + 1.0001
            else:
                if margin_absolut != 0:
                    gamma = distance_to_edge / margin_absolut
                else:
                    gamma = 0

        else:
            raise NotImplementedError("Implement othr gamma-types if desire.")

        if self.is_boundary:
            return 1 / gamma

        return gamma
