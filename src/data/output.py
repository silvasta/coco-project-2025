from dataclasses import dataclass
from numpy import array

@dataclass
class Output():

    t: int
    data : array
