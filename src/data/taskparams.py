import os,sys
parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent)
from dataclasses import dataclass
from models.hankel import Hankel as han
from models.theta import Theta as t

@dataclass()
class TaskParams():

    id : str
    taskname : str
    actuators : str
    gui : bool
    files : dict
    end : int
    scale : float
    deparr : str
    regions : str
    networkname : str
    horizon : int = 0
    Tini : int = 0
    begin : int = 0
    time_to_teleport : int = -1
    output_type : str = 'density'
    freq : int = 3
    n_threads : int = 1
    degree : int = 4
    n_PWA : int = 10

    "Estimate params"
    labels : str | None = None
    mfddata : str | None = None
    control_cycle : int | None = None
    controller_params : str | None = None
    parameter: str | None = None
    n_regions: int | None = None
    clusteringalg: str | None = None
    supersampling: bool | None = None
    superfreq: int | None = None

    "Control and study params" 
    parameters: list | None = None
    paramrange: list | None = None
    Hankel: han | None = None
    Theta: t | None = None
    controller : str | None = None  
    
    "Actuator Selection params"
    selector : str | None = None
    nactuators : int | None = None


  
    