from dataclasses import dataclass
from numpy import array

@dataclass
class Input():

    t: int
    data: array