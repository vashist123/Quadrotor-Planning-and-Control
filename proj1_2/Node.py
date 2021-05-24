import numpy as np

class Node:
    def __init__(self,index_coord,parent=None):
        self.index_coord = index_coord
        self.g = np.inf
        self.h = np.inf
        self.f = np.inf
        self.parent = parent
        self.is_closed = False

    def __repr__(self):
        return f" Node index = {self.index_coord} , g = {self.g} , h = {self.h} ,is_closed = {self.is_closed} ,parent = {self.parent}"

    def __lt__(self, other):
        return self.f < other.f